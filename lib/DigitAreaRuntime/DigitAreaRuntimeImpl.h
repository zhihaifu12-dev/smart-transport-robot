// PutTest reuses the verified PickStorageTest hardware configuration and motion helpers.
#include "VisionConfig.h"
#define PICK_STORAGE_TEST
#define setup storageTestSetup
#define loop storageTestLoop
#include "../StorageRuntime/StorageRuntimeImpl.h"
#undef setup
#undef loop

namespace Config
{
// -------------------- PutTest实机调试参数 --------------------
// 视觉调整幅度：数值越大，单次M5旋转和M6伸缩越多。
constexpr float PUT_VISION_X_GAIN = 3.0f; // [实机必调] M5视觉补偿增益
constexpr float PUT_VISION_Y_GAIN = 6.0f; // [实机必调] M6视觉补偿增益

// 以数字2中心坐标为基准搜索数字3；数字1由数字2、3的中心坐标预测。
constexpr float PUT_DIGIT3_M5_OFFSET = -360.0f; // [实机必调] -450=-45度
// [旧流程保留] 数字1改为由数字2、3坐标预测，当前不再使用此视觉预定位参数。
constexpr float PUT_DIGIT1_M5_OFFSET = 400.0f;
// [实机必调] 数字2识别完成后，搜索数字3时M6向外增加的长度；单位：0.1 mm。
// PutTest、粗加工区、暂存区分别调节，互不影响；360表示向外伸长36 mm。
constexpr float PUT_SIDE_M6_EXTENSION = 360.0f; // PutTest专用
constexpr float COARSE_SIDE_M6_EXTENSION = 360.0f; // 粗加工区专用
constexpr float TEMPORARY_SIDE_M6_EXTENSION = 360.0f; // 暂存区专用

constexpr uint8_t PUT_CAMERA_RX_PIN = PE7;
constexpr uint8_t PUT_CAMERA_TX_PIN = PE8;
constexpr float PUT_LOWEST_POSITION = 1300.0f; // [实机必调] ID7数字扫描位置，单位0.1 mm
constexpr float PUT_PLACE_POSITION = PUT_LOWEST_POSITION + 100.0f; // [实机必调] ID7地面取放高度，当前1450
constexpr float PUT_HORIZONTAL_INITIAL_POSITION = 300.0f; // [实机必调] 数字2识别前ID6预伸，100=10 mm
constexpr uint32_t PUT_DIGIT2_M6_PRE_EXTEND_WAIT_MS = 100; // [实机必调] 预伸命令后的额外停稳时间
constexpr float PUT_CENTER_TOLERANCE_PX = 2.0f; // [实机必调] 数字中心允许误差，单位px
constexpr uint8_t PUT_STABLE_FRAME_COUNT = 4; // 连续满足中心误差的帧数
constexpr uint32_t PUT_DIGIT_TIMEOUT_MS = 30000;
constexpr uint32_t PUT_VISION_SETTLE_MS = 120;
constexpr uint32_t PUT_PLACE_EXTRA_WAIT_MS = 500; // [实机必调] ID7长距离下降后的额外等待
constexpr uint32_t PUT_PLACE_SETTLE_MS = 300; // [实机必调] 到达地面取放高度后、夹爪动作前稳定0.2s
constexpr uint32_t PUT_RELEASE_SETTLE_MS = 200;

// -------------------- 粗加工区/暂存区从地面夹取：夹爪独立等待参数 --------------------
// [实机必调] 单位：ms。默认值复制当前时序，修改后不影响地面放料或储料盘取放。
constexpr uint32_t GROUND_PICK_M7_TRAVEL_WAIT_MS = 100; // M7下降后的实际运动等待
constexpr uint32_t GROUND_PICK_BEFORE_GRIPPER_CLOSE_MS = 200; // 到达夹取高度后、闭爪前稳定
constexpr uint32_t GROUND_PICK_AFTER_GRIPPER_CLOSE_MS = 100; // 闭爪后、M7抬升前等待

// -------------------- PutTest / 最终数字区的M7速度与加速度 --------------------
constexpr uint16_t M7_DIGIT_SCAN_VELOCITY = 5000; // [实机必调] 下降到数字/颜色识别高度
constexpr uint16_t M7_DIGIT_SCAN_ACCELERATION = 150;
constexpr uint16_t M7_GROUND_PLACE_VELOCITY = 5000; // [实机必调] 下降到地面放料高度
// 夹持物料下降到地面放料时降低加速度；比当前地面夹取加速度少200。
constexpr uint16_t M7_GROUND_PLACE_ACCELERATION = 120;
constexpr uint16_t M7_GROUND_PICK_VELOCITY = 5000; // [实机必调] 下降到地面夹取高度
constexpr uint16_t M7_GROUND_PICK_ACCELERATION = 150;
constexpr uint16_t M7_SELF_TEST_VELOCITY = 10000; // [实机必调] PutTest自检下降
constexpr uint16_t M7_SELF_TEST_ACCELERATION = 100;

// -------------------- PB9启动后的M5/M6/M7自检幅度 --------------------
constexpr float PUT_SELF_TEST_M5_ANGLE = 50.0f; // [实机必调] M5转动量，50=5度
constexpr float PUT_SELF_TEST_M6_POSITION = 40.0f; // [实机必调] ID6伸出量，30=3 mm
constexpr float PUT_SELF_TEST_M7_POSITION = 100.0f; // [实机必调] ID7下降量，100=10 mm

constexpr int8_t PUT_BASE_DIRECTION_SIGN = 1; // X越调越偏时改为-1
constexpr int8_t PUT_HORIZONTAL_DIRECTION_SIGN = -1; // Y越调越偏时改为1

constexpr float PUT_CAMERA_INITIAL_SCALE_MM = 262.0f;
constexpr float PUT_CAMERA_ZERO_SCALE_MM = 20.0f;
constexpr float PUT_CAMERA_INSTALL_HEIGHT_MM = 282.0f;
constexpr float PUT_TARGET_TOP_HEIGHT_MM = 147.0f; // [实机必调] 恢复原补偿，ID7=1500时约0.03 mm/px
constexpr float PUT_CAMERA_IMAGE_HEIGHT_PX = 240.0f;
constexpr float PUT_ARM_ZERO_LENGTH = 1206.96826f; // M5轴到相机/夹爪中心，单位0.1 mm
} // namespace Config

namespace
{
HardwareSerial putCameraSerial(Config::PUT_CAMERA_RX_PIN,
                               Config::PUT_CAMERA_TX_PIN);

struct DigitFrame
{
    int8_t dx = 0;
    int8_t dy = 0;
    uint8_t digit = 0;
};

class DigitReceiver
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

    bool poll(DigitFrame &frame)
    {
        while (putCameraSerial.available())
        {
            const uint8_t value = static_cast<uint8_t>(putCameraSerial.read());
            if (count == 0)
            {
                if (value != 0xAC)
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
            if (packet[0] != 0xAC || packet[4] != 0xBC)
            {
                continue;
            }
            frame.dx = static_cast<int8_t>(packet[1]);
            frame.dy = static_cast<int8_t>(packet[2]);
            frame.digit = packet[3];
            return true;
        }
        return false;
    }

private:
    uint8_t packet[5] = {0};
    uint8_t count = 0;
};

DigitReceiver digitReceiver;
bool m7DigitHeightCommandSent = false;

struct RecordedPutPose
{
    float baseAngle = 0.0f;
    float horizontalPosition = 0.0f;
    float verticalPosition = Config::PUT_LOWEST_POSITION;
    uint8_t digit = 0;
};

float putVisionScaleMmPerPixel()
{
    const float currentHeightMm = verticalCommandPosition / 10.0f;
    const float cameraHeight =
        Config::PUT_CAMERA_INSTALL_HEIGHT_MM - currentHeightMm;
    const float deltaHeight = cameraHeight - Config::PUT_TARGET_TOP_HEIGHT_MM;
    return (((Config::PUT_CAMERA_INITIAL_SCALE_MM -
              Config::PUT_CAMERA_ZERO_SCALE_MM) /
                 Config::PUT_CAMERA_INSTALL_HEIGHT_MM * deltaHeight) +
            Config::PUT_CAMERA_ZERO_SCALE_MM) /
           Config::PUT_CAMERA_IMAGE_HEIGHT_PX;
}

void applyDigitCorrection(const DigitFrame &frame)
{
    const float scale = putVisionScaleMmPerPixel();
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
        std::atan2(tangentialCorrection, armRadius) *
        (1800.0f / static_cast<float>(M_PI));

    Serial.print("VISION CORRECTION: scale(mm/px)=");
    Serial.print(scale, 4);
    Serial.print(" M5 delta(0.1deg)=");
    Serial.print(angleCorrection, 3);
    Serial.print(" M6 target(0.1mm)=");
    Serial.println(horizontalTarget, 3);

    baseAxis.moveTo(baseAxis.position() + angleCorrection);
    moveHorizontal(horizontalTarget);
    delay(phaseDelayMs(Config::PUT_VISION_SETTLE_MS));
}

// 数字搜索期间若连续1秒未识别到目标数字，M6每次相对移动5 mm：
// 第一次向外伸出，下一次向内收回，之后持续交替。每次运动后重新计时，
// 并清除运动期间积压的相机帧，避免使用运动前的旧坐标。
void serviceDigitSearchM6(uint32_t &lastTargetDigitAt, bool &extendNext)
{
    const uint32_t now = millis();
    if (now - lastTargetDigitAt < VisionConfig::DIGIT_SEARCH_M6_INTERVAL_MS)
    {
        return;
    }

    const float direction = extendNext ? 1.0f : -1.0f;
    const float requestedTarget = horizontalCommandPosition +
                                  direction * VisionConfig::DIGIT_SEARCH_M6_STEP_TENTH_MM;
    const float limitedTarget = std::clamp(
        requestedTarget,
        Config::HORIZONTAL_MIN_POSITION,
        Config::HORIZONTAL_MAX_POSITION);

    Serial.print("DIGIT SEARCH M6: ");
    Serial.print(extendNext ? "extend" : "retract");
    Serial.print(" requested/used(0.1mm)=");
    Serial.print(requestedTarget);
    Serial.print('/');
    Serial.println(limitedTarget);

    if (fabsf(limitedTarget - horizontalCommandPosition) > 0.5f)
    {
        moveHorizontal(limitedTarget);
    }
    else
    {
        Serial.println("DIGIT SEARCH M6: software limit reached, direction reversed");
    }

    extendNext = !extendNext;
    lastTargetDigitAt = millis();
    digitReceiver.discard();
}

bool alignAndRecordDigit(RecordedPutPose &pose)
{
    moveVertical(Config::PUT_LOWEST_POSITION,
                 Config::M7_DIGIT_SCAN_VELOCITY,
                 Config::M7_DIGIT_SCAN_ACCELERATION);
    Serial.print("PUT SCAN: move ID6 to initial position(0.1mm)=");
    Serial.println(Config::PUT_HORIZONTAL_INITIAL_POSITION);
    moveHorizontal(Config::PUT_HORIZONTAL_INITIAL_POSITION);
    digitReceiver.discard();
    uint8_t stableFrames = 0;
    uint8_t stableDigit = 0;
    uint8_t lastValidDigit = 0;
    const uint32_t startedAt = millis();
    uint32_t lastTargetDigitAt = startedAt;
    bool extendM6Next = true;

    while (millis() - startedAt < Config::PUT_DIGIT_TIMEOUT_MS)
    {
        serviceLockedChassis();
        serviceDigitSearchM6(lastTargetDigitAt, extendM6Next);
        DigitFrame frame;
        if (!digitReceiver.poll(frame))
        {
            delay(1);
            continue;
        }
        if (frame.digit < 1 || frame.digit > Config::STORAGE_SLOT_COUNT)
        {
            Serial.print("DIGIT PACKET: no target, value=");
            Serial.println(frame.digit);
            stableFrames = 0;
            continue;
        }
        lastTargetDigitAt = millis();
        lastValidDigit = frame.digit;

        Serial.print("DIGIT=");
        Serial.print(frame.digit);
        Serial.print(" dx/dy=");
        Serial.print(frame.dx);
        Serial.print('/');
        Serial.println(frame.dy);

        const float distance = std::sqrt(
            static_cast<float>(frame.dx * frame.dx + frame.dy * frame.dy));
        if (distance <= Config::PUT_CENTER_TOLERANCE_PX)
        {
            stableFrames = (stableDigit == frame.digit) ? stableFrames + 1 : 1;
            stableDigit = frame.digit;
            if (stableFrames >= Config::PUT_STABLE_FRAME_COUNT)
            {
                pose.baseAngle = baseAxis.position();
                pose.horizontalPosition = horizontalCommandPosition;
                pose.verticalPosition = Config::PUT_LOWEST_POSITION;
                pose.digit = frame.digit;
                Serial.print("PUT POSE RECORDED: digit/M5/M6/ID7=");
                Serial.print(pose.digit);
                Serial.print('/');
                Serial.print(pose.baseAngle);
                Serial.print('/');
                Serial.print(pose.horizontalPosition);
                Serial.print('/');
                Serial.println(pose.verticalPosition);
                return true;
            }
            continue;
        }

        stableFrames = 0;
        stableDigit = 0;
        applyDigitCorrection(frame);
        digitReceiver.discard();
    }

    if (lastValidDigit >= 1 && lastValidDigit <= Config::STORAGE_SLOT_COUNT)
    {
        pose.baseAngle = baseAxis.position();
        pose.horizontalPosition = horizontalCommandPosition;
        pose.verticalPosition = Config::PUT_LOWEST_POSITION;
        pose.digit = lastValidDigit;
        Serial.print("PUT ALIGN TIMEOUT: use current pose, digit/M5/M6/ID7=");
        Serial.print(pose.digit);
        Serial.print('/');
        Serial.print(pose.baseAngle);
        Serial.print('/');
        Serial.print(pose.horizontalPosition);
        Serial.print('/');
        Serial.println(pose.verticalPosition);
        return true;
    }

    Serial.println("PUT ERROR: timeout without any valid digit 1/2/3");
    return false;
}

void recordCurrentPutPose(uint8_t digit, RecordedPutPose &pose,
                          const char *reason)
{
    pose.baseAngle = baseAxis.position();
    pose.horizontalPosition = horizontalCommandPosition;
    pose.verticalPosition = Config::PUT_LOWEST_POSITION;
    pose.digit = digit;
    Serial.print(reason);
    Serial.print(": digit/M5/M6/ID7=");
    Serial.print(pose.digit);
    Serial.print('/');
    Serial.print(pose.baseAngle);
    Serial.print('/');
    Serial.print(pose.horizontalPosition);
    Serial.print('/');
    Serial.println(pose.verticalPosition);
}

bool predictDigit1Pose(const RecordedPutPose &digit2,
                       const RecordedPutPose &digit3,
                       RecordedPutPose &digit1)
{
    // M5/M6是以M5旋转轴为原点的极坐标。先转换到地面平面坐标，
    // 使用“数字2是数字1和数字3的中点”计算P1=2*P2-P3，再转回M5/M6。
    constexpr float TENTH_DEG_TO_RAD =
        static_cast<float>(M_PI) / 1800.0f;
    constexpr float RAD_TO_TENTH_DEG =
        1800.0f / static_cast<float>(M_PI);

    const float digit2Radius =
        Config::PUT_ARM_ZERO_LENGTH + digit2.horizontalPosition;
    const float digit3Radius =
        Config::PUT_ARM_ZERO_LENGTH + digit3.horizontalPosition;
    if (digit2Radius <= 0.0f || digit3Radius <= 0.0f)
    {
        Serial.println("DIGIT 1 PREDICTION ERROR: invalid digit 2/3 radius");
        return false;
    }

    const float digit2Angle = digit2.baseAngle * TENTH_DEG_TO_RAD;
    const float digit3Angle = digit3.baseAngle * TENTH_DEG_TO_RAD;
    const float digit1X =
        2.0f * digit2Radius * std::cos(digit2Angle) -
        digit3Radius * std::cos(digit3Angle);
    const float digit1Y =
        2.0f * digit2Radius * std::sin(digit2Angle) -
        digit3Radius * std::sin(digit3Angle);
    const float digit1Radius = std::hypot(digit1X, digit1Y);
    const float geometricHorizontal =
        digit1Radius - Config::PUT_ARM_ZERO_LENGTH;

    // atan2给出等价角度后，选择最接近镜像预测角2*M5(2)-M5(3)的安全圈次。
    const float canonicalBase =
        std::atan2(digit1Y, digit1X) * RAD_TO_TENTH_DEG;
    const float expectedBase =
        2.0f * digit2.baseAngle - digit3.baseAngle;
    float geometricBase = canonicalBase;
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
        const float distance = std::fabs(candidate - expectedBase);
        if (distance < bestDistance)
        {
            geometricBase = candidate;
            bestDistance = distance;
            baseInRange = true;
        }
    }

    // 几何预测完成后分别叠加M5角度和M6长度补偿，再检查最终目标限位。
    const float compensatedBase =
        geometricBase + VisionConfig::DIGIT1_PREDICT_M5_COMPENSATION_TENTH_DEG;
    const float calculatedHorizontal =
        geometricHorizontal + VisionConfig::DIGIT1_PREDICT_M6_COMPENSATION_TENTH_MM;
    // 预测伸长量超过M6正向限位时，取计算值与上限中的较小值继续放置。
    // 负向仍严格遵守-5 mm软件限位，禁止用截断掩盖异常预测。
    const float limitedHorizontal =
        std::min(calculatedHorizontal, Config::HORIZONTAL_MAX_POSITION);

    if (!baseInRange ||
        compensatedBase < Config::BASE_MIN_ANGLE ||
        compensatedBase > Config::BASE_MAX_ANGLE ||
        limitedHorizontal < Config::HORIZONTAL_MIN_POSITION)
    {
        Serial.print("DIGIT 1 PREDICTION ERROR: M5/M6 out of range=");
        Serial.print(compensatedBase);
        Serial.print('/');
        Serial.println(calculatedHorizontal);
        return false;
    }

    if (limitedHorizontal != calculatedHorizontal)
    {
        Serial.print("DIGIT 1 M6 LIMIT: calculated/used(0.1mm)=");
        Serial.print(calculatedHorizontal);
        Serial.print('/');
        Serial.println(limitedHorizontal);
    }

    digit1.baseAngle = compensatedBase;
    digit1.horizontalPosition = limitedHorizontal;
    digit1.verticalPosition = digit2.verticalPosition;
    digit1.digit = 1;

    Serial.print("DIGIT 1 GEOMETRIC M5/M6=");
    Serial.print(geometricBase);
    Serial.print('/');
    Serial.println(geometricHorizontal);
    Serial.print("DIGIT 1 COMPENSATION M5/M6=");
    Serial.print(VisionConfig::DIGIT1_PREDICT_M5_COMPENSATION_TENTH_DEG);
    Serial.print('/');
    Serial.println(VisionConfig::DIGIT1_PREDICT_M6_COMPENSATION_TENTH_MM);
    Serial.print("DIGIT 1 PREDICTED: digit/M5/M6/ID7=");
    Serial.print(digit1.digit);
    Serial.print('/');
    Serial.print(digit1.baseAngle);
    Serial.print('/');
    Serial.print(digit1.horizontalPosition);
    Serial.print('/');
    Serial.println(digit1.verticalPosition);
    return true;
}

bool alignExpectedDigit(uint8_t expectedDigit, RecordedPutPose &pose)
{
    digitReceiver.discard();
    uint8_t stableFrames = 0;
    bool expectedDigitSeen = false;
    const uint32_t startedAt = millis();
    uint32_t lastTargetDigitAt = startedAt;
    bool extendM6Next = true;

    Serial.print("SEARCH DIGIT ");
    Serial.println(expectedDigit);
    while (millis() - startedAt < Config::PUT_DIGIT_TIMEOUT_MS)
    {
        serviceLockedChassis();
        serviceDigitSearchM6(lastTargetDigitAt, extendM6Next);
        DigitFrame frame;
        if (!digitReceiver.poll(frame))
        {
            delay(1);
            continue;
        }

        if (frame.digit != expectedDigit)
        {
            stableFrames = 0;
            if (frame.digit >= 1 && frame.digit <= Config::STORAGE_SLOT_COUNT)
            {
                Serial.print("IGNORE DIGIT ");
                Serial.print(frame.digit);
                Serial.print(", EXPECT ");
                Serial.println(expectedDigit);
            }
            continue;
        }

        lastTargetDigitAt = millis();
        expectedDigitSeen = true;
        Serial.print("DIGIT=");
        Serial.print(frame.digit);
        Serial.print(" dx/dy=");
        Serial.print(frame.dx);
        Serial.print('/');
        Serial.println(frame.dy);

        const float distance = std::sqrt(
            static_cast<float>(frame.dx * frame.dx + frame.dy * frame.dy));
        if (distance <= Config::PUT_CENTER_TOLERANCE_PX)
        {
            ++stableFrames;
            if (stableFrames >= Config::PUT_STABLE_FRAME_COUNT)
            {
                recordCurrentPutPose(expectedDigit, pose, "PUT POSE RECORDED");
                return true;
            }
            continue;
        }

        stableFrames = 0;
        applyDigitCorrection(frame);
        digitReceiver.discard();
    }

    if (expectedDigitSeen)
    {
        recordCurrentPutPose(expectedDigit, pose,
                             "PUT ALIGN TIMEOUT: use current pose");
        return true;
    }

    Serial.print("PUT ERROR: timeout without expected digit ");
    Serial.println(expectedDigit);
    return false;
}

bool locateDigit2Pose(RecordedPutPose &digit2Pose)
{
    // 数字2开始识别前，M6先伸出一小段距离，再将M7降到数字扫描高度。
    Serial.print("DIGIT 2 SEARCH: extend ID6 first, target(0.1mm)=");
    Serial.println(Config::PUT_HORIZONTAL_INITIAL_POSITION);
    moveHorizontal(Config::PUT_HORIZONTAL_INITIAL_POSITION);
    delay(phaseDelayMs(Config::PUT_DIGIT2_M6_PRE_EXTEND_WAIT_MS));
    Serial.println("DIGIT 2 SEARCH: ID6 10mm pre-extension complete");
    moveVertical(Config::PUT_LOWEST_POSITION,
                 Config::M7_DIGIT_SCAN_VELOCITY,
                 Config::M7_DIGIT_SCAN_ACCELERATION);

    if (!alignExpectedDigit(2, digit2Pose))
    {
        return false;
    }

    // 数字2记录完成后恢复安全高位旋转前的M6/M5位置，M7保持识别高度。
    moveHorizontal(Config::HORIZONTAL_RESET_POSITION);
    baseAxis.moveTo(0.0f);
    return true;
}

bool locateAllDigitPoses(
    RecordedPutPose poses[4],
    float sideM6Extension = Config::PUT_SIDE_M6_EXTENSION)
{
    if (!locateDigit2Pose(poses[2]))
    {
        return false;
    }

    const float digit2Base = poses[2].baseAngle;
    const float digit2Horizontal = poses[2].horizontalPosition;
    const float sideHorizontalTarget = std::clamp(
        digit2Horizontal + sideM6Extension,
        Config::HORIZONTAL_MIN_POSITION,
        Config::HORIZONTAL_MAX_POSITION);

    Serial.print("DIGIT 3 PREPOSITION: M6 extension(0.1mm)=");
    Serial.println(sideM6Extension);
    baseAxis.moveTo(digit2Base + Config::PUT_DIGIT3_M5_OFFSET);
    moveHorizontal(sideHorizontalTarget);
    if (!alignExpectedDigit(3, poses[3]))
    {
        return false;
    }
    moveHorizontal(Config::HORIZONTAL_RESET_POSITION);
    baseAxis.moveTo(0.0f);

    // 数字1不再进行相机识别、短时确认或微调；放置和夹回直接使用预测姿态。
    if (!predictDigit1Pose(poses[2], poses[3], poses[1]))
    {
        return false;
    }

    Serial.println("ALL DIGIT POSES READY: digit 2/3 vision, digit 1 predicted");
    return true;
}

// M5或ID5储料盘转动前的安全前置动作。
// 顺序固定为：M7升至高位 -> M6收回零位；完成后调用方才能执行旋转。
void prepareArmForSafeRotation()
{
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
    Serial.println("ROTATION SAFE: ID7 high, M6 retracted");
}

void resetArmToHighZero()
{
    prepareArmForSafeRotation();
    if (fabsf(baseAxis.position()) > 0.5f)
    {
        baseAxis.moveTo(0.0f);
    }
}

void runArmSelfTest()
{
    Serial.println("=== PUT SELF TEST: M5 -> M6 -> M7 ===");
    resetArmToHighZero();

    Serial.print("SELF TEST M5 target(0.1deg)=");
    Serial.println(Config::PUT_SELF_TEST_M5_ANGLE);
    baseAxis.moveTo(Config::PUT_SELF_TEST_M5_ANGLE);
    baseAxis.moveTo(0.0f);

    Serial.print("SELF TEST M6 target(0.1mm)=");
    Serial.println(Config::PUT_SELF_TEST_M6_POSITION);
    moveHorizontal(Config::PUT_SELF_TEST_M6_POSITION);
    moveHorizontal(Config::HORIZONTAL_RESET_POSITION);

    Serial.print("SELF TEST M7 target(0.1mm)=");
    Serial.println(Config::PUT_SELF_TEST_M7_POSITION);
    moveVertical(Config::PUT_SELF_TEST_M7_POSITION,
                 Config::M7_SELF_TEST_VELOCITY,
                 Config::M7_SELF_TEST_ACCELERATION);
    moveVertical(Config::VERTICAL_HIGH_POSITION,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);

    Serial.println("=== PUT SELF TEST COMPLETE: M5/M6/M7 reset ===");
}

void takeMaterialFromStorage(uint8_t slotNumber)
{
    Serial.print("TAKE MATERIAL FROM STORAGE SLOT ");
    Serial.println(slotNumber);

    // 严格沿用PickStorageTest的取料张爪参数。PutTest放完上一件物料后会
    // 使用openMax()，因此每次取下一件前必须重新张开到PICK_GRIPPER_OPEN_ANGLE。
    // 粗加工区/暂存区取盘中物料：先高位收M6，再转ID5储料盘和M5。
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

    // 与PickStorage相同地抬高并复位M6/M5，但保持夹爪闭合以便搬运。
    moveVertical(Config::VERTICAL_HIGH_POSITION,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);
    if (fabsf(horizontalCommandPosition - Config::HORIZONTAL_RESET_POSITION) > 0.5f)
    {
        moveHorizontal(Config::HORIZONTAL_RESET_POSITION);
    }
    if (fabsf(baseAxis.position()) > 0.5f)
    {
        baseAxis.moveTo(0.0f);
    }
}

void placeMaterialAtRecordedPose(const RecordedPutPose &pose)
{
    Serial.print("PLACE MATERIAL AT RECORDED DIGIT ");
    Serial.println(pose.digit);
    // 从储料盘转向粗加工区/暂存区数字位前，先高位收回M6，再转M5。
    prepareArmForSafeRotation();
    baseAxis.moveTo(pose.baseAngle);
    if (fabsf(pose.horizontalPosition - Config::HORIZONTAL_RESET_POSITION) > 0.5f)
    {
        moveHorizontal(pose.horizontalPosition);
    }
    // 放料必须回到最低高度，并在机械振动停止后才松开夹爪。
    moveVertical(Config::PUT_PLACE_POSITION,
                 Config::M7_GROUND_PLACE_VELOCITY,
                 Config::M7_GROUND_PLACE_ACCELERATION);
    Serial.print("PLACE: extra ID7 travel wait(ms)=");
    Serial.println(Config::PUT_PLACE_EXTRA_WAIT_MS);
    delay(Config::PUT_PLACE_EXTRA_WAIT_MS);
    Serial.print("PLACE: lowest position reached, settle(ms)=");
    Serial.println(Config::PUT_PLACE_SETTLE_MS);
    delay(Config::PUT_PLACE_SETTLE_MS);
    Serial.println("PLACE: open gripper after settling");
    gripper.openMax();
    delay(phaseDelayMs(Config::PUT_RELEASE_SETTLE_MS));
    resetArmToHighZero();
}

void runPutTestOnce()
{
    runArmSelfTest();
    RecordedPutPose poses[4];
    if (!locateAllDigitPoses(poses))
    {
        resetArmToHighZero();
        return;
    }

    resetArmToHighZero();
    for (uint8_t digit = 1; digit <= Config::STORAGE_SLOT_COUNT; ++digit)
    {
        Serial.print("PUT MATERIAL: storage slot -> digit ");
        Serial.println(digit);
        takeMaterialFromStorage(digit);
        placeMaterialAtRecordedPose(poses[digit]);
    }
    Serial.println("PUT TEST COMPLETE: materials placed at digits 1/2/3");
}
} // namespace

#ifndef PUTTEST_SETUP_NAME
#define PUTTEST_SETUP_NAME setup
#endif
#ifndef PUTTEST_LOOP_NAME
#define PUTTEST_LOOP_NAME loop
#endif

extern "C" void PUTTEST_SETUP_NAME()
{
    storageTestSetup();
    digitReceiver.initialize();
    Serial.println("=== PUT TEST READY ===");
    Serial.println("PB9: record digits 2/3, predict digit 1, then place materials");
}

extern "C" void PUTTEST_LOOP_NAME()
{
    serviceLockedChassis();
    if (startPressed())
    {
        runPutTestOnce();
    }
    delay(1);
}

extern "C" void M7DigitHeightTest_Setup()
{
    Serial.begin(Config::SERIAL_BAUDRATE);
    delay(Config::POWER_STABILIZE_MS);

    pinMode(Config::START_BUTTON_PIN, INPUT_PULLUP);
    rawButton = stableButton = digitalRead(Config::START_BUTTON_PIN);
    buttonChangedAt = millis();
    lockChassis();

    // 测试入口只初始化M7。上电前必须先确认M7位于机械高位，当前点将被设为0。
    ttlProtocol.init(&ttlSerial, Config::SERIAL_BAUDRATE);
    verticalStepper.init();
    verticalStepper.set(Config::M7_DIGIT_SCAN_VELOCITY,
                        phaseAcceleration(Config::M7_DIGIT_SCAN_ACCELERATION),
                        Config::VERTICAL_POSITIVE_DIRECTION,
                        LEAD_SCREW_TENTH_MM_PER_REV,
                        MICROSTEPS);
    ttlProtocol.Emm_V5_En_Control(Config::VERTICAL_ID, true, false);
    delay(100);
    ttlProtocol.Emm_V5_Reset_CurPos_To_Zero(Config::VERTICAL_ID);
    delay(100);
    verticalCommandPosition = Config::VERTICAL_HIGH_POSITION;
    m7DigitHeightCommandSent = false;

    Serial.println();
    Serial.println("=== M7 DIGIT HEIGHT TEST READY ===");
    Serial.print("TARGET M7(0.1mm)=");
    Serial.println(Config::PUT_LOWEST_POSITION);
    Serial.println("Confirm M7 is at the high zero position, then press PB9 once.");
}

extern "C" void M7DigitHeightTest_Loop()
{
    serviceLockedChassis();
    if (!m7DigitHeightCommandSent && startPressed())
    {
        Serial.println("M7 DIGIT HEIGHT TEST: move to digit recognition height");
        moveVertical(Config::PUT_LOWEST_POSITION,
                     Config::M7_DIGIT_SCAN_VELOCITY,
                     Config::M7_DIGIT_SCAN_ACCELERATION);
        m7DigitHeightCommandSent = true;
        Serial.println("M7 DIGIT HEIGHT TEST: target command sent; hold position");
    }
    delay(1);
}
