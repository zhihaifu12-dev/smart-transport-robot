#include "FinalTask.h"

// 复用PutTest及其内部PickStorageTest/StorageTest的全部实机参数。
#define PUTTEST_SETUP_NAME putTestEmbeddedSetup
#define PUTTEST_LOOP_NAME putTestEmbeddedLoop
#define FINAL_TASK_EMBEDDED
#include "../DigitAreaRuntime/DigitAreaRuntimeImpl.h"
#undef FINAL_TASK_EMBEDDED

void FinalTask_BeginRawAreaArmReset();
void FinalTask_ServiceRawAreaArmReset();
bool FinalTask_RawAreaArmResetComplete();

namespace
{
enum class AreaResetBackend : uint8_t
{
    None,
    Raw,
    Digit
};

AreaResetBackend areaResetBackend = AreaResetBackend::None;
bool digitAreaResetActive = false;
uint32_t digitAreaResetReadyAt = 0;
}
#undef PUTTEST_SETUP_NAME
#undef PUTTEST_LOOP_NAME

namespace
{
constexpr float FINAL_ARM_STOW_ANGLE = 1350.0f; // [实机必调] 1350=135度
constexpr uint8_t FILL_LIGHT_COMMAND_HEADER = 0xA5;
constexpr uint8_t FILL_LIGHT_COMMAND_ON = 0x01;
constexpr uint8_t FILL_LIGHT_COMMAND_OFF = 0x00;
constexpr uint8_t FILL_LIGHT_COMMAND_TAIL = 0x5A;
constexpr uint32_t FILL_LIGHT_ON_SETTLE_MS = 200; // [实机必调] 开灯后等待曝光稳定
constexpr int DIGIT_SCAN_GRIPPER_OPEN_ANGLE = -90; // [实机必调] 仅数字识别时夹爪张开角度，单位：degree
constexpr float VISION_STORAGE_TRAY_ANGLE = 88.0f; // [实机必调] 数字/颜色识别时ID5储料盘避让角
constexpr uint32_t VISION_STORAGE_TRAY_SETTLE_MS = 500; // 储料盘转到视觉避让角后的停稳时间
constexpr float TEMP_COLOR_CENTER_TOLERANCE_PX = 3.0f; // [实机必调] 第二次暂存区颜色圆心误差
constexpr uint8_t TEMP_COLOR_STABLE_FRAME_COUNT = 4; // 连续居中帧数
constexpr uint32_t TEMP_COLOR_ALIGN_TIMEOUT_MS = 15000; // 单个颜色校准超时
bool finalArmInitialized = false;
RecordedPutPose firstCoarsePoses[4];
bool firstCoarsePosesValid = false;
RecordedPutPose firstTemporaryPoses[4];
bool firstTemporaryPosesValid = false;
bool digitAreaPreparedOnArrival = false;
bool storageTrayAtVisionAngle = false;
uint32_t storageTrayVisionReadyAt = 0;

struct TemporaryColorFrame
{
    int8_t dx = 0;
    int8_t dy = 0;
    uint8_t color = 0;
};

struct ArmPlanePoint
{
    float x = 0.0f;
    float y = 0.0f;
};

bool poseToArmPlanePoint(const RecordedPutPose &pose, ArmPlanePoint &point)
{
    const float radius =
        Config::PUT_ARM_ZERO_LENGTH + pose.horizontalPosition;
    if (radius <= 0.0f)
    {
        return false;
    }
    const float angle =
        pose.baseAngle * static_cast<float>(M_PI) / 1800.0f;
    point.x = radius * std::cos(angle);
    point.y = radius * std::sin(angle);
    return true;
}

bool armPlanePointToPose(const ArmPlanePoint &point,
                         uint8_t digit,
                         float verticalPosition,
                         float referenceBase,
                         RecordedPutPose &pose)
{
    const float radius = std::hypot(point.x, point.y);
    const float calculatedHorizontal =
        radius - Config::PUT_ARM_ZERO_LENGTH;
    // 第二轮耦合坐标超过M6正向限位时，取计算值与上限中的较小值继续作业。
    const float limitedHorizontal =
        std::min(calculatedHorizontal, Config::HORIZONTAL_MAX_POSITION);
    const float canonicalBase =
        std::atan2(point.y, point.x) * 1800.0f /
        static_cast<float>(M_PI);

    float selectedBase = canonicalBase;
    float bestDistance = 100000.0f;
    bool baseInRange = false;
    for (int8_t turn = -1; turn <= 1; ++turn)
    {
        const float candidate = canonicalBase + 3600.0f * turn;
        if (candidate < Config::BASE_MIN_ANGLE ||
            candidate > Config::BASE_MAX_ANGLE)
        {
            continue;
        }
        const float distance = std::fabs(candidate - referenceBase);
        if (distance < bestDistance)
        {
            selectedBase = candidate;
            bestDistance = distance;
            baseInRange = true;
        }
    }

    if (!baseInRange ||
        limitedHorizontal < Config::HORIZONTAL_MIN_POSITION)
    {
        Serial.print("DIGIT COUPLING ERROR: digit/M5/M6 out of range=");
        Serial.print(digit);
        Serial.print('/');
        Serial.print(selectedBase);
        Serial.print('/');
        Serial.println(calculatedHorizontal);
        return false;
    }

    if (limitedHorizontal != calculatedHorizontal)
    {
        Serial.print("DIGIT COUPLING M6 LIMIT: digit/calculated/used(0.1mm)=");
        Serial.print(digit);
        Serial.print('/');
        Serial.print(calculatedHorizontal);
        Serial.print('/');
        Serial.println(limitedHorizontal);
    }

    pose.baseAngle = selectedBase;
    pose.horizontalPosition = limitedHorizontal;
    pose.verticalPosition = verticalPosition;
    pose.digit = digit;
    return true;
}

bool coupleSecondRoundPoses(const RecordedPutPose firstRound[4],
                            const RecordedPutPose &secondDigit2,
                            RecordedPutPose secondRound[4])
{
    ArmPlanePoint firstDigit2Point;
    ArmPlanePoint secondDigit2Point;
    if (!poseToArmPlanePoint(firstRound[2], firstDigit2Point) ||
        !poseToArmPlanePoint(secondDigit2, secondDigit2Point))
    {
        Serial.println("DIGIT COUPLING ERROR: invalid digit 2 pose");
        return false;
    }

    const float offsetX = secondDigit2Point.x - firstDigit2Point.x;
    const float offsetY = secondDigit2Point.y - firstDigit2Point.y;
    const float baseOffset =
        secondDigit2.baseAngle - firstRound[2].baseAngle;
    secondRound[2] = secondDigit2;

    for (uint8_t digit = 1; digit <= 3; ++digit)
    {
        if (digit == 2)
        {
            continue;
        }
        ArmPlanePoint firstPoint;
        if (!poseToArmPlanePoint(firstRound[digit], firstPoint))
        {
            Serial.print("DIGIT COUPLING ERROR: invalid first-round digit ");
            Serial.println(digit);
            return false;
        }
        const ArmPlanePoint coupledPoint = {
            firstPoint.x + offsetX,
            firstPoint.y + offsetY};
        if (!armPlanePointToPose(coupledPoint,
                                 digit,
                                 secondDigit2.verticalPosition,
                                 firstRound[digit].baseAngle + baseOffset,
                                 secondRound[digit]))
        {
            return false;
        }
    }

    Serial.print("SECOND COARSE COUPLING: offset X/Y(0.1mm)=");
    Serial.print(offsetX);
    Serial.print('/');
    Serial.println(offsetY);
    for (uint8_t digit = 1; digit <= 3; ++digit)
    {
        Serial.print("COUPLED DIGIT pose digit/M5/M6=");
        Serial.print(digit);
        Serial.print('/');
        Serial.print(secondRound[digit].baseAngle);
        Serial.print('/');
        Serial.println(secondRound[digit].horizontalPosition);
    }
    return true;
}

class TemporaryColorReceiver
{
public:
    void initialize()
    {
        putCameraSerial.begin(Config::SERIAL_BAUDRATE);
        discard();
    }

    void discard()
    {
        while (putCameraSerial.available())
        {
            putCameraSerial.read();
        }
        count = 0;
    }

    bool poll(TemporaryColorFrame &frame)
    {
        while (putCameraSerial.available())
        {
            const uint8_t value = static_cast<uint8_t>(putCameraSerial.read());
            if (count == 0)
            {
                if (value != 0xAA)
                {
                    continue;
                }
                packet[count++] = value;
                continue;
            }

            packet[count++] = value;
            if (count < sizeof(packet))
            {
                continue;
            }

            count = 0;
            if (packet[0] != 0xAA || packet[4] != 0xBB)
            {
                continue;
            }
            frame.dx = static_cast<int8_t>(packet[1]);
            frame.dy = static_cast<int8_t>(packet[2]);
            frame.color = packet[3];
            return true;
        }
        return false;
    }

private:
    uint8_t packet[5] = {0};
    uint8_t count = 0;
};

TemporaryColorReceiver temporaryColorReceiver;

void setCameraFillLight(bool enabled)
{
    const uint8_t command[3] = {
        FILL_LIGHT_COMMAND_HEADER,
        enabled ? FILL_LIGHT_COMMAND_ON : FILL_LIGHT_COMMAND_OFF,
        FILL_LIGHT_COMMAND_TAIL};
    // 相机没有命令应答，重复发送可避免切换UART对象后的首帧丢失。
    for (uint8_t retry = 0; retry < 3; ++retry)
    {
        putCameraSerial.write(command, sizeof(command));
        putCameraSerial.flush();
        delay(20);
    }
    Serial.println(enabled ? "CAMERA FILL LIGHT: ON" : "CAMERA FILL LIGHT: OFF");
    if (enabled)
    {
        delay(FILL_LIGHT_ON_SETTLE_MS);
        digitReceiver.discard();
    }
}

void commandStorageTrayToVisionAngle()
{
    // 粗加工区/暂存区切换视觉避让角前，也必须先高位收回M6。
    prepareArmForSafeRotation();
    Serial.print("VISION STORAGE TRAY: angle(deg)=");
    Serial.println(VISION_STORAGE_TRAY_ANGLE);
    storageServo.setRawAngle(VISION_STORAGE_TRAY_ANGLE);
    storageTrayAtVisionAngle = false;
    storageTrayVisionReadyAt = millis() + VISION_STORAGE_TRAY_SETTLE_MS;
}

void waitStorageTrayAtVisionAngle()
{
    while (static_cast<int32_t>(millis() - storageTrayVisionReadyAt) < 0)
    {
        serviceLockedChassis();
        delay(1);
    }
    storageTrayAtVisionAngle = true;
}

void moveStorageTrayToVisionAngle()
{
    commandStorageTrayToVisionAngle();
    waitStorageTrayAtVisionAngle();
}

void openGripperForDigitScan()
{
    Serial.print("DIGIT SCAN GRIPPER ANGLE=");
    Serial.println(DIGIT_SCAN_GRIPPER_OPEN_ANGLE);
    gripper.setAngle(DIGIT_SCAN_GRIPPER_OPEN_ANGLE,
                     phaseDelayMs(Config::GRIPPER_CLOSE_TIME_MS),
                     Config::GRIPPER_MAX_POWER_MW);
    delay(phaseDelayMs(Config::GRIPPER_SETTLE_MS));
}

void applyTemporaryColorCorrection(const TemporaryColorFrame &frame,
                                   float materialHeightTenthMm)
{
    const float currentHeightMm = verticalCommandPosition / 10.0f;
    const float cameraHeight = Config::PUT_CAMERA_INSTALL_HEIGHT_MM - currentHeightMm;
    const float targetTopHeight = Config::PUT_TARGET_TOP_HEIGHT_MM +
                                  materialHeightTenthMm / 10.0f;
    const float deltaHeight = cameraHeight - targetTopHeight;
    const float scale =
        (((Config::PUT_CAMERA_INITIAL_SCALE_MM - Config::PUT_CAMERA_ZERO_SCALE_MM) /
              Config::PUT_CAMERA_INSTALL_HEIGHT_MM * deltaHeight) +
         Config::PUT_CAMERA_ZERO_SCALE_MM) /
        Config::PUT_CAMERA_IMAGE_HEIGHT_PX;
    const float tangentialCorrection =
        frame.dx * scale * 10.0f * Config::PUT_VISION_X_GAIN *
        Config::PUT_BASE_DIRECTION_SIGN;
    const float radialCorrection =
        frame.dy * scale * 10.0f * Config::PUT_VISION_Y_GAIN *
        Config::PUT_HORIZONTAL_DIRECTION_SIGN;
    const float horizontalTarget = std::clamp(
        horizontalCommandPosition + radialCorrection,
        Config::HORIZONTAL_MIN_POSITION,
        Config::HORIZONTAL_MAX_POSITION);
    const float armRadius = std::max(
        Config::PUT_ARM_ZERO_LENGTH + horizontalTarget, 1.0f);
    const float angleCorrection =
        atan2f(tangentialCorrection, armRadius) *
        (1800.0f / static_cast<float>(M_PI));

    Serial.print("TEMP COLOR CORRECTION: scale/M5delta/M6target=");
    Serial.print(scale, 4);
    Serial.print('/');
    Serial.print(angleCorrection, 3);
    Serial.print('/');
    Serial.println(horizontalTarget, 3);
    baseAxis.moveTo(baseAxis.position() + angleCorrection);
    moveHorizontal(horizontalTarget);
    delay(phaseDelayMs(Config::PUT_VISION_SETTLE_MS));
}

bool alignTemporaryMaterialColor(RecordedPutPose &pose,
                                 uint8_t expectedColor,
                                 float scanPosition)
{
    if (expectedColor < 1 || expectedColor > 6)
    {
        Serial.println("TEMP COLOR ERROR: invalid expected color");
        return false;
    }

    Serial.print("TEMP COLOR PREPOSITION: color/M5/M6/ID7=");
    Serial.print(expectedColor);
    Serial.print('/');
    Serial.print(pose.baseAngle);
    Serial.print('/');
    Serial.print(pose.horizontalPosition);
    Serial.print('/');
    Serial.println(scanPosition);

    // 转向暂存区记录坐标前先高位收M6，避免M5旋转路径与储料盘干涉。
    prepareArmForSafeRotation();
    baseAxis.moveTo(pose.baseAngle);
    if (fabsf(pose.horizontalPosition - horizontalCommandPosition) > 0.5f)
    {
        moveHorizontal(pose.horizontalPosition);
    }
    moveVertical(scanPosition,
                 Config::M7_DIGIT_SCAN_VELOCITY,
                 Config::M7_DIGIT_SCAN_ACCELERATION);

    // 第二批进入暂存区时已完成一次串口初始化；每件物料只清除途中积累的旧帧。
    temporaryColorReceiver.discard();
    uint8_t stableFrames = 0;
    const uint32_t startedAt = millis();
    while (millis() - startedAt < TEMP_COLOR_ALIGN_TIMEOUT_MS)
    {
        serviceLockedChassis();
        TemporaryColorFrame frame;
        if (!temporaryColorReceiver.poll(frame))
        {
            delay(1);
            continue;
        }
        if (frame.color != expectedColor)
        {
            continue;
        }

        const float distance = sqrtf(
            static_cast<float>(frame.dx * frame.dx + frame.dy * frame.dy));
        Serial.print("TEMP COLOR: id/dx/dy/r=");
        Serial.print(frame.color);
        Serial.print('/');
        Serial.print(frame.dx);
        Serial.print('/');
        Serial.print(frame.dy);
        Serial.print('/');
        Serial.println(distance);

        if (distance <= TEMP_COLOR_CENTER_TOLERANCE_PX)
        {
            ++stableFrames;
            if (stableFrames >= TEMP_COLOR_STABLE_FRAME_COUNT)
            {
                pose.baseAngle = baseAxis.position();
                pose.horizontalPosition = horizontalCommandPosition;
                pose.verticalPosition = scanPosition;
                Serial.print("TEMP COLOR POSE UPDATED: M5/M6=");
                Serial.print(pose.baseAngle);
                Serial.print('/');
                Serial.println(pose.horizontalPosition);
                return true;
            }
            continue;
        }

        stableFrames = 0;
        const float materialHeightTenthMm =
            std::max(Config::PUT_LOWEST_POSITION - scanPosition, 0.0f);
        applyTemporaryColorCorrection(frame, materialHeightTenthMm);
        temporaryColorReceiver.discard();
    }

    Serial.print("TEMP COLOR ERROR: alignment timeout, color=");
    Serial.println(expectedColor);
    return false;
}

void takeMaterialFromRecordedDigit(const RecordedPutPose &pose)
{
    Serial.print("PICK BACK FROM DIGIT ");
    Serial.println(pose.digit);

    // 不再绕行M5零位；只执行旋转前必需的安全动作：M7升至高位、M6收回。
    // 随后从当前位置直接转向已记录的地面物料坐标。
    prepareArmForSafeRotation();

    // 严格使用PickStorageTest的取料张开角度和夹取时序。
    openPickGripper();
    baseAxis.moveTo(pose.baseAngle);
    // 从地面夹取：先到数字识别高度，再调整M6，最后下降到夹取高度。
    moveVertical(Config::PUT_LOWEST_POSITION,
                 Config::M7_DIGIT_SCAN_VELOCITY,
                 Config::M7_DIGIT_SCAN_ACCELERATION);
    moveHorizontal(pose.horizontalPosition);
    moveVertical(Config::PUT_PLACE_POSITION,
                 Config::M7_GROUND_PICK_VELOCITY,
                 Config::M7_GROUND_PICK_ACCELERATION);
    delay(Config::GROUND_PICK_M7_TRAVEL_WAIT_MS);
    delay(Config::GROUND_PICK_BEFORE_GRIPPER_CLOSE_MS);
    Serial.println("GROUND PICK: close gripper");
    gripper.setAngle(Config::GRIPPER_CLOSE_ANGLE,
                     phaseDelayMs(Config::GRIPPER_CLOSE_TIME_MS),
                     Config::GRIPPER_MAX_POWER_MW);
    delay(phaseDelayMs(Config::GROUND_PICK_AFTER_GRIPPER_CLOSE_MS));
    // 夹起后不执行机械臂完整复位。调用方紧接着执行储料盘回存，
    // 由placeMaterialBackIntoStorage()先升高M7并收回M6，再转动ID5和M5。
}

void takeMaterialFromStorageContinuous(uint8_t slotNumber)
{
    slotNumber = constrain(slotNumber, 1, Config::STORAGE_SLOT_COUNT);
    Serial.print("CONTINUOUS TAKE FROM STORAGE SLOT ");
    Serial.println(slotNumber);
    // 粗加工区和暂存区取料统一时序：M7高位 -> M6收回 -> ID5转盘 -> M5转向。
    prepareArmForSafeRotation();
    openPickGripper();
    storageServo.setRawAngle(storageSlotAngle(slotNumber));
    storageServo.wait();
    baseAxis.moveTo(storagePlaceM5Angle(slotNumber));
    moveVertical(Config::PICK_VERTICAL_GRAB_POSITION,
                 Config::M7_STORAGE_PICK_VELOCITY,
                 Config::M7_STORAGE_PICK_ACCELERATION);
    moveHorizontal(storagePlaceM6Position(slotNumber));
    delay(Config::PICK_GRAB_SETTLE_MS);
    closeGripper();
    moveVertical(Config::VERTICAL_HIGH_POSITION,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);
}

void placeMaterialBackIntoStorage(uint8_t slotNumber)
{
    slotNumber = constrain(slotNumber, 1, Config::STORAGE_SLOT_COUNT);
    Serial.print("RETURN MATERIAL TO STORAGE SLOT ");
    Serial.println(slotNumber);

    // 从粗加工区/暂存区回存前，先高位收M6，再转动储料盘和M5。
    prepareArmForSafeRotation();
    storageServo.setRawAngle(storageSlotAngle(slotNumber));
    storageServo.wait();
    baseAxis.moveTo(storagePlaceM5Angle(slotNumber));
    if (fabsf(storagePlaceM6Position(slotNumber) -
              horizontalCommandPosition) > 0.5f)
    {
        moveHorizontal(storagePlaceM6Position(slotNumber));
    }
    moveVertical(Config::PICK_VERTICAL_GRAB_POSITION,
                 Config::M7_STORAGE_PLACE_VELOCITY,
                 Config::M7_STORAGE_PLACE_ACCELERATION);
    delay(Config::PICK_GRAB_SETTLE_MS);
    openPickGripper();
    // 中间动作只抬升M7；三件全部完成后统一复位。
    moveVertical(Config::VERTICAL_HIGH_POSITION,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);
}

void placeMaterialAtDepth(const RecordedPutPose &pose, float placeDepth)
{
    Serial.print("PLACE MATERIAL AT RECORDED DIGIT/DEPTH=");
    Serial.print(pose.digit);
    Serial.print('/');
    Serial.println(placeDepth);
    // 从储料盘转向粗加工区/暂存区数字位前，先高位收回M6，再转M5。
    prepareArmForSafeRotation();
    baseAxis.moveTo(pose.baseAngle);
    if (fabsf(pose.horizontalPosition - horizontalCommandPosition) > 0.5f)
    {
        moveHorizontal(pose.horizontalPosition);
    }
    moveVertical(placeDepth,
                 Config::M7_GROUND_PLACE_VELOCITY,
                 Config::M7_GROUND_PLACE_ACCELERATION);
    delay(Config::PUT_PLACE_EXTRA_WAIT_MS);
    delay(Config::PUT_PLACE_SETTLE_MS);
    gripper.openMax();
    delay(phaseDelayMs(Config::PUT_RELEASE_SETTLE_MS));
    // 中间动作只抬升M7，下一件从当前M5/M6坐标直接转移。
    moveVertical(Config::VERTICAL_HIGH_POSITION,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);
}
} // namespace

bool FinalTask_InitArm()
{
    if (finalArmInitialized)
    {
        return true;
    }

    if (!initializeHardware())
    {
        return false;
    }
    digitReceiver.initialize();
    finalArmInitialized = true;
    return true;
}

bool FinalTask_ResetStorageTray()
{
    // 原料区与数字区复用同一硬件串口；每次出口复位前重新绑定通信对象。
    servoProtocol.init(&servoSerial, Config::SERIAL_BAUDRATE);
    storageServo.init();
    if (!storageServo.isOnline)
    {
        Serial.println("STORAGE RESET ERROR: tray servo is offline; continue route");
        return false;
    }
    storageServo.setSpeed(Config::STORAGE_SERVO_SPEED);

    Serial.print("STORAGE RESET: angle(deg)=");
    Serial.println(Config::STORAGE_INITIAL_ANGLE);
    storageServo.setRawAngle(Config::STORAGE_INITIAL_ANGLE);
    digitAreaPreparedOnArrival = false;
    storageTrayAtVisionAngle = false;
    // 固定等待期间持续服务异步机械臂复位，使M5/M6/M7与ID5储料盘并行运动。
    // 不使用库的wait()，避免最长约60秒的不可控阻塞。
    const uint32_t resetStartedAt = millis();
    while (millis() - resetStartedAt < Config::STORAGE_RESET_SETTLE_MS)
    {
        FinalTask_ServiceArmReset();
        delay(1);
    }
    return true;
}

bool FinalTask_PrepareDigitAreaOnArrival()
{
    if (!finalArmInitialized)
    {
        return false;
    }
    // 原料区与数字区共享通信资源；必须先在安全高位完成重新绑定，
    // 再立即发送ID5视觉避让角，后续航向校准期间舵盘可并行转动。
    if (!initializeHardware())
    {
        Serial.println("DIGIT ARRIVAL ERROR: hardware preparation failed");
        return false;
    }
    commandStorageTrayToVisionAngle();
    digitAreaPreparedOnArrival = true;
    return true;
}

void FinalTask_CloseGripperForStart()
{
    if (!finalArmInitialized)
    {
        return;
    }
    Serial.println("START PREPARE: close gripper before M5 rotation");
    gripper.setAngle(Config::GRIPPER_CLOSE_ANGLE,
                     phaseDelayMs(Config::GRIPPER_CLOSE_TIME_MS),
                     Config::GRIPPER_MAX_POWER_MW);
    delay(phaseDelayMs(Config::GRIPPER_TO_MOVE_DELAY_MS));
}

void FinalTask_OpenGripperForStart()
{
    if (!finalArmInitialized)
    {
        return;
    }
    Serial.println("START: PB9 pressed, open gripper before departure");
    gripper.openMax();
    delay(phaseDelayMs(Config::GRIPPER_SETTLE_MS));
}

void FinalTask_SetArmStowed(bool stowed)
{
    if (!finalArmInitialized)
    {
        return;
    }

    if (fabsf(verticalCommandPosition - Config::VERTICAL_HIGH_POSITION) > 0.5f)
    {
        moveVertical(Config::VERTICAL_HIGH_POSITION,
                     Config::M7_LIFT_VELOCITY,
                     Config::M7_LIFT_ACCELERATION);
    }
    if (fabsf(horizontalCommandPosition - Config::HORIZONTAL_RESET_POSITION) > 0.5f)
    {
        moveHorizontal(Config::HORIZONTAL_RESET_POSITION);
    }
    baseAxis.moveTo(stowed ? FINAL_ARM_STOW_ANGLE : 0.0f);
}

void FinalTask_BeginAreaArmReset(bool rawArea)
{
    if (!finalArmInitialized)
    {
        return;
    }

    if (rawArea)
    {
        areaResetBackend = AreaResetBackend::Raw;
        FinalTask_BeginRawAreaArmReset();
        return;
    }

    areaResetBackend = AreaResetBackend::Digit;
    Serial.println("DIGIT EXIT: start M5/M6/M7 reset while chassis departs");
    if (fabsf(verticalCommandPosition - Config::VERTICAL_HIGH_POSITION) > 0.5f)
    {
        startVerticalMove(Config::VERTICAL_HIGH_POSITION,
                          Config::M7_LIFT_VELOCITY,
                          Config::M7_LIFT_ACCELERATION);
    }
    if (fabsf(horizontalCommandPosition - Config::HORIZONTAL_RESET_POSITION) > 0.5f)
    {
        startHorizontalMove(Config::HORIZONTAL_RESET_POSITION);
    }
    if (fabsf(baseAxis.position()) > 0.5f)
    {
        baseAxis.startMoveTo(0.0f);
    }
    digitAreaResetReadyAt = millis() + std::max(
        phaseDelayMs(Config::VERTICAL_SETTLE_MS),
        phaseDelayMs(Config::HORIZONTAL_SETTLE_MS));
    digitAreaResetActive = true;
}

void FinalTask_ServiceArmReset()
{
    if (areaResetBackend == AreaResetBackend::Raw)
    {
        FinalTask_ServiceRawAreaArmReset();
        if (FinalTask_RawAreaArmResetComplete())
        {
            areaResetBackend = AreaResetBackend::None;
        }
        return;
    }
    if (areaResetBackend != AreaResetBackend::Digit || !digitAreaResetActive)
    {
        return;
    }

    baseAxis.service();
    if (!baseAxis.isRunning() &&
        static_cast<int32_t>(millis() - digitAreaResetReadyAt) >= 0)
    {
        digitAreaResetActive = false;
        areaResetBackend = AreaResetBackend::None;
        Serial.println("DIGIT EXIT: asynchronous arm reset complete");
    }
}

bool FinalTask_ArmResetComplete()
{
    if (areaResetBackend == AreaResetBackend::Raw)
    {
        return FinalTask_RawAreaArmResetComplete();
    }
    return areaResetBackend == AreaResetBackend::None &&
           !digitAreaResetActive;
}

void FinalTask_MoveBaseTo(float angleTenthDegree)
{
    baseAxis.moveTo(angleTenthDegree);
}

void FinalTask_MoveHorizontalTo(float positionTenthMm)
{
    moveHorizontal(positionTenthMm);
}

void FinalTask_MoveVerticalTo(float positionTenthMm)
{
    moveVertical(positionTenthMm,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);
}

void FinalTask_OpenGripperForGroundPick()
{
    openPickGripper();
}

void FinalTask_CloseGripperForHolding()
{
    closeGripper();
}

void FinalTask_OpenGripperFully()
{
    gripper.openMax();
    delay(phaseDelayMs(Config::GRIPPER_SETTLE_MS));
}

void FinalTask_MoveStorageToSlot(uint8_t slotNumber)
{
    prepareArmForSafeRotation();
    storageServo.setRawAngle(storageSlotAngle(slotNumber));
    storageServo.wait();
    storageTrayAtVisionAngle = false;
}

bool FinalTask_ProcessDigitArea(const uint8_t sourceSlots[3],
                                const uint8_t targetDigits[3],
                                bool pickBack,
                                bool reuseTemporaryCoordinates,
                                float placeDepthReduction,
                                const uint8_t targetColors[3])
{
    if (!finalArmInitialized || sourceSlots == nullptr || targetDigits == nullptr)
    {
        return false;
    }

    if (digitAreaPreparedOnArrival)
    {
        // ID5命令已在到达功能区时发送；这里只等待尚未完成的停稳时间。
        waitStorageTrayAtVisionAngle();
        digitAreaPreparedOnArrival = false;
    }
    else
    {
        // PutTest等独立入口没有整车到达事件，保留完整初始化和避让动作。
        if (!initializeHardware())
        {
            Serial.println("DIGIT AREA ERROR: PutTest hardware reinitialization failed");
            return false;
        }
        moveStorageTrayToVisionAngle();
    }
    RecordedPutPose poses[4];
    const bool secondCoarseRound = pickBack && firstCoarsePosesValid;
    if (reuseTemporaryCoordinates)
    {
        if (!firstTemporaryPosesValid || targetColors == nullptr)
        {
            Serial.println("TEMP ERROR: first coordinates or target colors are not available");
            return false;
        }
        for (uint8_t digit = 1; digit <= 3; ++digit)
        {
            poses[digit] = firstTemporaryPoses[digit];
        }
        // 颜色相机协议整批只初始化一次，三件物料之间仅清空旧帧。
        temporaryColorReceiver.initialize();
        Serial.println("TEMP BATCH 2: use first coordinates as color-alignment prepositions");
    }
    else
    {
        digitReceiver.initialize();
        // 只在识别数字期间张到-90度，减少夹爪对相机视野的遮挡。
        openGripperForDigitScan();
        setCameraFillLight(true);
        bool digitsLocated = false;
        if (secondCoarseRound)
        {
            Serial.println("COARSE BATCH 2: locate digit 2 only, then couple first-round poses");
            digitsLocated = locateDigit2Pose(poses[2]) &&
                            coupleSecondRoundPoses(firstCoarsePoses,
                                                   poses[2], poses);
        }
        else
        {
            const float sideM6Extension = pickBack
                                              ? Config::COARSE_SIDE_M6_EXTENSION
                                              : Config::TEMPORARY_SIDE_M6_EXTENSION;
            Serial.print(pickBack
                             ? "COARSE DIGIT SEARCH: M6 side extension(0.1mm)="
                             : "TEMP DIGIT SEARCH: M6 side extension(0.1mm)=");
            Serial.println(sideM6Extension);
            digitsLocated = locateAllDigitPoses(poses, sideM6Extension);
        }
        // 补光灯只服务于数字识别；无论成功或超时，识别结束后都必须关闭。
        setCameraFillLight(false);
        if (!digitsLocated)
        {
            resetArmToHighZero();
            return false;
        }

        // 不取回物料表示第一次暂存区作业，保存三个数字的机械臂坐标。
        if (!pickBack)
        {
            for (uint8_t digit = 1; digit <= 3; ++digit)
            {
                firstTemporaryPoses[digit] = poses[digit];
            }
            firstTemporaryPosesValid = true;
            Serial.println("TEMP BATCH 1: digit coordinates saved");
        }
    }

    resetArmToHighZero();
    const float placeDepth = std::max(
        Config::PUT_PLACE_POSITION - placeDepthReduction, 0.0f);
    const float temporaryColorScanPosition = std::max(
        Config::PUT_LOWEST_POSITION - placeDepthReduction, 0.0f);
    for (uint8_t i = 0; i < 3; ++i)
    {
        const uint8_t slot = constrain(sourceSlots[i], 1, 3);
        const uint8_t digit = constrain(targetDigits[i], 1, 3);
        if (reuseTemporaryCoordinates)
        {
            // 第二次暂存区：先看第一层同色物料，更新该位置后再去储料盘取第二层物料。
            if (!storageTrayAtVisionAngle)
            {
                moveStorageTrayToVisionAngle();
            }
            if (!alignTemporaryMaterialColor(poses[digit],
                                             targetColors[i],
                                             temporaryColorScanPosition))
            {
                resetArmToHighZero();
                return false;
            }
            // 后续取盘函数会统一执行M7升高、M6收回，无需在这里重复检查M7高位。
        }
        takeMaterialFromStorageContinuous(slot);
        storageTrayAtVisionAngle = false;
        placeMaterialAtDepth(poses[digit], placeDepth);
    }

    if (pickBack)
    {
        // 粗加工区必须按放置顺序取回：第1件先取、第2件次之、第3件最后取。
        // 每件从地面夹起后不回M5零位；placeMaterialBackIntoStorage()
        // 会先升高M7并收回M6，确认安全姿态后才转动ID5储料盘和M5。
        for (uint8_t i = 0; i < Config::STORAGE_SLOT_COUNT; ++i)
        {
            const uint8_t slot = constrain(sourceSlots[i], 1, 3);
            const uint8_t digit = constrain(targetDigits[i], 1, 3);
            Serial.print("COARSE PICK BACK ORDER: item/digit/slot=");
            Serial.print(i + 1);
            Serial.print('/');
            Serial.print(digit);
            Serial.print('/');
            Serial.println(slot);
            takeMaterialFromRecordedDigit(poses[digit]);
            placeMaterialBackIntoStorage(slot);
        }
    }

    if (pickBack && !secondCoarseRound)
    {
        for (uint8_t digit = 1; digit <= 3; ++digit)
        {
            firstCoarsePoses[digit] = poses[digit];
        }
        firstCoarsePosesValid = true;
        Serial.println("COARSE BATCH 1: digit coordinates saved for batch 2 coupling");
    }

    // 第3件完成后保持储料盘末位；RobotApp确认离开功能区时，
    // 再启动M5/M6/M7与ID5储料盘的并行复位。
    Serial.println("DIGIT AREA: storage tray reset deferred until area exit");
    Serial.println(pickBack ? "DIGIT AREA COMPLETE: materials returned"
                            : "DIGIT AREA COMPLETE: materials retained on field");
    return true;
}
