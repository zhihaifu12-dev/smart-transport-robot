from maix import app, camera, display, image, pinmap, time, uart
import cv2
import math
import struct


WIDTH = 320
HEIGHT = 240
FPS = 30
CAMERA_HEIGHT_MM = 130
SCALE = CAMERA_HEIGHT_MM / HEIGHT
MIN_PIXELS = 800
DIGIT_SCAN_INTERVAL = 3
DIGIT_MISS_LIMIT = 2
DIGIT_MIN_GROUP_CIRCLES = 3



COLOR_SPECS = [
    (1, "red", (255, 0, 0), (20, 65, 18, 72, 4, 40)),
    (2, "yellow", (255, 220, 0), (35, 95, -15, 10, 22, 92)),
    (3, "blue", (0, 70, 255), (15, 90, -40, 30, -60, -4)),
    (4, "green", (0, 210, 70), (20, 70, -55, -15, 12, 50)),
]


def read_int(file_path, line_num, default_value=0):
    try:
        with open(file_path, "r") as f:
            lines = f.readlines()
        if line_num < 1 or line_num > len(lines):
            return default_value
        text = lines[line_num - 1]
        content = ""
        for ch in text:
            if ch == "-" or ch.isdigit():
                content += ch
        return int(content) if content and content != "-" else default_value
    except Exception:
        return default_value


ERROR_X = read_int("/root/error.txt", 1, 0)
ERROR_Y = read_int("/root/error.txt", 2, 0)
HAND_X = int(WIDTH / 2 + ERROR_X / SCALE)
HAND_Y = int(HEIGHT / 2 + ERROR_Y / SCALE)


WHITE = image.Color.from_rgb(255, 255, 255)
CENTER_MARK_COLOR = image.Color.from_rgb(255, 0, 255)
RED = image.Color.from_rgb(255, 0, 0)
GREEN = image.Color.from_rgb(0, 255, 0)
YELLOW = image.Color.from_rgb(255, 220, 0)


def clamp_byte(v):
    v = int(v)
    if v > 127:
        return 127
    if v < -128:
        return -128
    return v


def send_packet(serial, x, y, color_id):
    data = (
        b"\xaa"
        + struct.pack("b", clamp_byte(HAND_X - x))
        + struct.pack("b", clamp_byte(y - HAND_Y))
        + struct.pack("b", clamp_byte(color_id))
        + b"\xbb"
    )
    try:
        serial.write(data)
    except Exception:
        pass


def send_digit_packet(serial, x, y, digit):
    data = (
        b"\xac"
        + struct.pack("b", clamp_byte(HAND_X - x))
        + struct.pack("b", clamp_byte(y - HAND_Y))
        + struct.pack("b", clamp_byte(digit))
        + b"\xbc"
    )
    try:
        serial.write(data)
        return True
    except Exception:
        return False


def send_to_all(serial0, serial1, target):
    if target is None:
        send_packet(serial0, HAND_X, HAND_Y, 0)
        if serial1 is not None and serial1 != serial0:
            send_packet(serial1, HAND_X, HAND_Y, 0)
        return
    x, y, _, color_id, _, _ = target[:6]
    send_packet(serial0, x, y, color_id)
    if serial1 is not None and serial1 != serial0:
        send_packet(serial1, x, y, color_id)


def send_digit_to_all(serial0, serial1, target):
    if target is None:
        x, y, digit = HAND_X, HAND_Y, 0
    else:
        x, y, _, digit, _, _ = target[:6]
    serial0_ok = send_digit_packet(serial0, x, y, digit)
    if serial1 is not None and serial1 != serial0:
        return send_digit_packet(serial1, x, y, digit)
    return serial0_ok


def best_blob_from_color(img, spec):
    color_id, name, rgb, lab = spec
    blobs = img.find_blobs([lab], merge=True, pixels_threshold=MIN_PIXELS)
    if not blobs:
        return None

    best = None
    for blob in blobs:
        pixels = blob.pixels()
        w = blob.w()
        h = blob.h()
        if w <= 0 or h <= 0:
            continue
        shape = min(w, h) / max(w, h)
        score = pixels * shape
        if best is None or score > best[0]:
            x = int(blob.x() + w / 2)
            y = int(blob.y() + h / 2)
            r = int((w + h) / 4)
            best = (score, x, y, r, color_id, name, rgb, pixels)
    return best


def find_contours(binary):
    result = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 2:
        return result[0]
    return result[1]


def group_concentric_circles(candidates):
    groups = []
    candidates.sort(key=lambda item: item[2], reverse=True)
    for candidate in candidates:
        matched = None
        for group in groups:
            gx = sum(item[0] for item in group) / len(group)
            gy = sum(item[1] for item in group) / len(group)
            if (candidate[0] - gx) ** 2 + (candidate[1] - gy) ** 2 < 49:
                matched = group
                break
        if matched is None:
            groups.append([candidate])
        else:
            matched.append(candidate)
    return [group for group in groups if len(group) >= DIGIT_MIN_GROUP_CIRCLES]


def reflection_symmetry(mask):
    best = 0.0
    for flip_code in (0, 1):
        flipped = cv2.flip(mask, flip_code)
        intersection = cv2.countNonZero(cv2.bitwise_and(mask, flipped))
        union = cv2.countNonZero(cv2.bitwise_or(mask, flipped))
        if union > 0:
            best = max(best, intersection / union)
    return best


def classify_digit_contour(contour, binary):
    rect = cv2.minAreaRect(contour)
    rw, rh = rect[1]
    if min(rw, rh) < 2:
        return None
    aspect = max(rw, rh) / min(rw, rh)

    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None
    cx = moments["m10"] / moments["m00"]
    cy = moments["m01"] / moments["m00"]
    angle = 0.5 * math.degrees(
        math.atan2(2 * moments["mu11"], moments["mu20"] - moments["mu02"])
    )

    contour_mask = binary.copy()
    contour_mask[:] = 0
    cv2.drawContours(contour_mask, [contour], -1, 255, -1)
    matrix = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
    rotated = cv2.warpAffine(
        contour_mask, matrix, (contour_mask.shape[1], contour_mask.shape[0])
    )
    rotated_contours = find_contours(rotated)
    if not rotated_contours:
        return None
    rotated_digit = max(rotated_contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(rotated_digit)
    if w <= 0 or h <= 0:
        return None
    normalized = cv2.resize(
        rotated[y : y + h, x : x + w], (64, 64), interpolation=cv2.INTER_NEAREST
    )
    symmetry = reflection_symmetry(normalized)

    if aspect >= 2.0:
        digit = 1
    elif symmetry >= 0.56:
        digit = 3
    else:
        digit = 2
    return digit, aspect, symmetry, cx, cy


def detect_circle_digit(img):
    cv_img = image.image2cv(img, ensure_bgr=True, copy=True)
    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    _, binary = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )

    circle_candidates = []
    for contour in find_contours(binary):
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if area < 180 or perimeter <= 0:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        _, _, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            continue
        if circularity < 0.55 or min(w, h) / max(w, h) < 0.75:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        if 10 < radius < 110:
            circle_candidates.append((cx, cy, radius))

    groups = group_concentric_circles(circle_candidates)
    if not groups:
        return None

    def group_distance(group):
        gx = sum(item[0] for item in group) / len(group)
        gy = sum(item[1] for item in group) / len(group)
        return (gx - HAND_X) ** 2 + (gy - HAND_Y) ** 2

    group = min(groups, key=group_distance)
    cx = int(round(sum(item[0] for item in group) / len(group)))
    cy = int(round(sum(item[1] for item in group) / len(group)))
    outer_radius = int(round(max(item[2] for item in group)))
    inner_radius = int(round(min(item[2] for item in group)))

    roi_radius = max(8, int(inner_radius * 0.82))
    x0 = max(0, cx - roi_radius)
    y0 = max(0, cy - roi_radius)
    x1 = min(WIDTH, cx + roi_radius + 1)
    y1 = min(HEIGHT, cy + roi_radius + 1)
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None

    roi = gray[y0:y1, x0:x1]
    _, digit_binary = cv2.threshold(
        roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )
    center_x = cx - x0
    center_y = cy - y0
    center_mask = digit_binary.copy()
    center_mask[:] = 0
    cv2.circle(
        center_mask,
        (center_x, center_y),
        max(4, int(roi_radius * 0.90)),
        255,
        -1,
    )
    digit_binary = cv2.bitwise_and(digit_binary, center_mask)

    minimum_area = max(8, inner_radius * inner_radius * 0.05)
    digit_contours = [
        contour
        for contour in find_contours(digit_binary)
        if cv2.contourArea(contour) >= minimum_area
    ]
    if not digit_contours:
        return None

    contour = max(digit_contours, key=cv2.contourArea)
    result = classify_digit_contour(contour, digit_binary)
    if result is None:
        return None
    digit, aspect, symmetry, digit_x, digit_y = result
    max_offset = inner_radius * 0.25
    if (digit_x - center_x) ** 2 + (digit_y - center_y) ** 2 > max_offset ** 2:
        return None

    return cx, cy, outer_radius, digit, aspect, symmetry


def detect_colors(img):
    targets = []
    best = None
    for spec in COLOR_SPECS:
        hit = best_blob_from_color(img, spec)
        if hit is None:
            continue
        targets.append(hit[1:7])
        if best is None or hit[0] > best[0]:
            best = hit

    display_target = None if best is None else best[1:7]
    return targets, display_target


def draw_overlay(img, target, digit_target, digit_tx_ok=False):
    img.draw_cross(HAND_X, HAND_Y, CENTER_MARK_COLOR, size=8, thickness=2)
    img.draw_string(6, 6, "4 Color + Circle Digit", WHITE, scale=1)
    img.draw_string(220, 6, "FPS:{:.1f}".format(time.fps()), WHITE, scale=1)

    if digit_target is not None:
        x, y, r, digit, _, _ = digit_target[:6]
        dx = clamp_byte(HAND_X - x)
        dy = clamp_byte(y - HAND_Y)
        img.draw_circle(x, y, max(4, r), GREEN, thickness=2)
        img.draw_cross(x, y, RED, size=8, thickness=2)
        img.draw_line(HAND_X, HAND_Y, x, y, YELLOW, thickness=1)
        img.draw_string(6, 26, "digit:{}".format(digit), GREEN, scale=1)
        img.draw_string(6, 42, "center:({},{})".format(x, y), WHITE, scale=1)
        img.draw_string(6, 58, "dx:{} dy:{}".format(dx, dy), WHITE, scale=1)
        tx_color = GREEN if digit_tx_ok else RED
        tx_state = "OK" if digit_tx_ok else "ERR"
        img.draw_string(
            6, 74, "TX digit:{} {}".format(digit, tx_state), tx_color, scale=1
        )
        return

    if target is None:
        img.draw_string(6, 26, "No material", RED, scale=1)
        return

    x, y, r, color_id, name, rgb = target[:6]
    color = image.Color.from_rgb(rgb[0], rgb[1], rgb[2])
    dx = clamp_byte(HAND_X - x)
    dy = clamp_byte(y - HAND_Y)

    img.draw_circle(x, y, max(4, r), color, thickness=2)
    img.draw_cross(x, y, CENTER_MARK_COLOR, size=8, thickness=2)
    img.draw_line(HAND_X, HAND_Y, x, y, YELLOW, thickness=1)
    img.draw_string(6, 26, "{} {}".format(color_id, name), color, scale=1)
    img.draw_string(6, 42, "center:({},{})".format(x, y), WHITE, scale=1)
    img.draw_string(6, 58, "dx:{} dy:{}".format(dx, dy), WHITE, scale=1)


def init_uart():
    serial0 = uart.UART("/dev/ttyS0", 115200)
    try:
        pinmap.set_pin_function("A18", "UART1_RX")
        pinmap.set_pin_function("A19", "UART1_TX")
        serial1 = uart.UART("/dev/ttyS1", 115200)
    except Exception:
        serial1 = None
    return serial0, serial1


def main():
    cam = camera.Camera(WIDTH, HEIGHT, fps=FPS)
    disp = display.Display()
    serial0, serial1 = init_uart()
    frame_index = 0
    digit_target = None
    digit_misses = DIGIT_MISS_LIMIT
    digit_tx_ok = False

    while not app.need_exit():
        img = cam.read()
        if frame_index % DIGIT_SCAN_INTERVAL == 0:
            detected_digit = detect_circle_digit(img)
            if detected_digit is None:
                digit_misses += 1
                if digit_misses >= DIGIT_MISS_LIMIT:
                    digit_target = None
            else:
                digit_target = detected_digit
                digit_misses = 0

        targets, target = detect_colors(img)
        if targets:
            # 逐个发送所有已识别颜色，使STM32能按任务指定颜色筛选目标。
            for detected_target in targets:
                send_to_all(serial0, serial1, detected_target)
        else:
            send_to_all(serial0, serial1, None)
        digit_tx_ok = send_digit_to_all(serial0, serial1, digit_target)

        draw_overlay(img, target, digit_target, digit_tx_ok)
        disp.show(img)
        frame_index += 1


if __name__ == "__main__":
    main()
