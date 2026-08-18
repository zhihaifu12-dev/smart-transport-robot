#include <Arduino.h>

#include <algorithm>
#include <cmath>

#include <AccelStepper.h>
#include <FashionStar_SmartGripper.h>
#include <FashionStar_UartServo.h>
#include "ArmConfig.h"
#include "TTL_STEPPER.h"

namespace Config
{
// -------------------- 常用抓取调试参数 --------------------
constexpr float HORIZONTAL_INITIAL_POSITION = 30.0f; // [实机必调] ID6初始伸出量，80表示8 mm
constexpr float VERTICAL_PICK_POSITION = 650.0f;    // [实机必调] ID7最终抓取深度，单位0.1 mm；统一减少20 mm
constexpr float HIGH_POSITION_Y_OFFSET_MM = 10.0f;   // [实机必调] 高位相机Y方向机械补偿，单位mm
constexpr float VISION_X_GAIN = 1.0f;                // [实机必调] M5底座视觉补偿增益
constexpr float VISION_Y_GAIN = 2.0f;                // [实机必调] ID6悬臂视觉补偿增益；原1.0，现增加50%修正量
constexpr int8_t BASE_DIRECTION_SIGN = 1;            // [实机必调] X误差越调越大时改为-1
constexpr int8_t HORIZONTAL_DIRECTION_SIGN = -1;     // [实机必调] Y误差越调越大时改为1

constexpr int GRIPPER_OPEN_ANGLE = -30;
constexpr int GRIPPER_CLOSE_ANGLE = 45;
constexpr int GRIPPER_OPEN_MAX_ANGLE = -80;
constexpr int STORAGE_RELEASE_ANGLE =
    ArmConfig::StorageReturn::GRIPPER_RELEASE_ANGLE_DEG;
constexpr uint16_t GRIPPER_MAX_POWER_MW = 400;
constexpr uint16_t GRIPPER_CLOSE_TIME_MS = 200; // [实机必调] ID4手爪闭合命令时间
// -------------------- 原料区从地面夹取：夹爪独立等待参数 --------------------
// [实机必调] 单位：ms。三项只影响原料区地面夹取，不影响储料盘取放。
constexpr uint32_t GROUND_PICK_GRIPPER_OPEN_SETTLE_MS = 200; // 张爪后、机械臂继续动作前
constexpr uint32_t GROUND_PICK_BEFORE_GRIPPER_CLOSE_MS = 40; // 到达夹取姿态后、闭爪前
constexpr uint32_t GROUND_PICK_AFTER_GRIPPER_CLOSE_MS = 200; // 闭爪后、M7抬升前
constexpr uint32_t ARM_BEFORE_GRIPPER_SETTLE_MS =
    ArmConfig::StorageReturn::PLACE_ARRIVAL_SETTLE_MS;
constexpr uint32_t POST_PICK_HIGH_HOLD_MS = 200; // [实机必调] 抓取并升到高位后的短暂稳定时间
constexpr uint8_t STORAGE_SLOT_COUNT = ArmConfig::StorageReturn::SLOT_COUNT;
static constexpr const float (&STORAGE_SERVO_ANGLES)[STORAGE_SLOT_COUNT] =
    ArmConfig::StorageReturn::ID5_SLOT_ANGLES_DEG;
constexpr uint8_t PICK_COLOR_SEQUENCE[STORAGE_SLOT_COUNT] = {2, 3, 4}; // 抓取顺序：黄、蓝、绿
constexpr float STORAGE_INITIAL_ANGLE = -91.0f; // [实机必调] 上电、测试清理及离开功能区时的储料盘复位角度
constexpr uint16_t STORAGE_SERVO_SPEED = 400;
constexpr uint32_t STORAGE_SETTLE_MS = 200;

// -------------------- 1/2/3号储料盘独立放料坐标 --------------------
static constexpr const float (&STORAGE_PLACE_M5_ANGLES)[STORAGE_SLOT_COUNT] =
    ArmConfig::StorageReturn::M5_SLOT_ANGLES_TENTH_DEG;
static constexpr const float (&STORAGE_PLACE_M6_POSITIONS)[STORAGE_SLOT_COUNT] =
    ArmConfig::StorageReturn::M6_SLOT_POSITIONS_TENTH_MM;
constexpr float HORIZONTAL_RESET_POSITION = 0.0f; // 整个任务结束后的M6/ID6复位位置
constexpr float VERTICAL_PLACE_POSITION =
    ArmConfig::StorageReturn::M7_PLACE_POSITION_TENTH_MM;

// -------------------- 两阶段总时长控制 --------------------
// 1.0=当前时长，0.8约缩短到80%，1.2约延长到120%。建议实机范围0.5~2.0。
constexpr float PICKUP_TIME_COEFFICIENT = 0.7f; // [实机必调] 识别、对准、下降和夹紧阶段
constexpr float PLACE_RETURN_TIME_COEFFICIENT = 0.7f; // [实机必调] 抬升、放料和返回阶段
constexpr float MIN_TIME_COEFFICIENT = 0.25f;
constexpr float MAX_TIME_COEFFICIENT = 10.0f;

// 以下参数参考已成功运行的visual_pick_to_tray_debug运动配置。
// -------------------- M7各动作独立速度/加速度 --------------------
// 速度越大运行越快；加速度越大启停越急。TTL加速度为uint8_t，最大值255。
constexpr uint16_t M7_REFERENCE_VELOCITY = 100000; // [实机必调] 下降到固定坐标/半高识别位置
constexpr uint16_t M7_REFERENCE_ACCELERATION = 900;
constexpr uint16_t M7_PICK_VELOCITY = 100000; // [实机必调] 从识别高度下降到原料抓取高度
constexpr uint16_t M7_PICK_ACCELERATION = 300;
constexpr uint16_t M7_STORAGE_PLACE_VELOCITY =
    ArmConfig::StorageReturn::M7_PLACE_VELOCITY;
constexpr uint8_t M7_STORAGE_PLACE_ACCELERATION =
    ArmConfig::StorageReturn::M7_PLACE_ACCELERATION;
constexpr uint16_t M7_LIFT_VELOCITY = 100000; // [实机必调] 抓取/放料后抬升及M7复位
constexpr uint16_t M7_LIFT_ACCELERATION = 900;
constexpr uint8_t VERTICAL_POSITIVE_DIRECTION = 0;
constexpr uint32_t VERTICAL_SETTLE_MS = 300; // ID7动作等待，兼顾到位可靠性与10秒抓取目标
constexpr uint32_t VERTICAL_ARRIVAL_TIMEOUT_MS = 100; // ID7到位状态查询时间；无应答时采用固定等待结果
constexpr uint32_t PICK_HEIGHT_STABLE_MS = 100; // 确认到达抓取高度后的机械稳定时间
constexpr uint32_t PICKUP_FRAME_WAIT_MS = 200; // 抓取高度等待最终视觉帧的最长时间
constexpr uint16_t HORIZONTAL_VELOCITY = 6000; // [实机必调] ID6运行速度；由80提高50%
constexpr uint8_t HORIZONTAL_ACCELERATION = 300; // TTL协议为uint8_t，255是有效最大值
constexpr uint8_t HORIZONTAL_POSITIVE_DIRECTION = 0;
constexpr uint32_t HORIZONTAL_SETTLE_MS = 500; // ID6动作等待，单次视觉校准后使用

constexpr float VERTICAL_HIGH_POSITION = 0.0f;
constexpr float PICK_REFERENCE_POSITION = 225.0f; // [实机必调] 初始识别并固定夹取坐标的ID7高度，原425减20mm
constexpr float HORIZONTAL_MIN_POSITION =
    ArmConfig::M6_MIN_POSITION_TENTH_MM;
constexpr float HORIZONTAL_MAX_POSITION = 1500.0f;
constexpr uint32_t VISION_SETTLE_MS = 80; // [实机必调] 半高视觉修正后的等待时间
constexpr uint32_t MATERIAL_SEARCH_TIMEOUT_MS = 120000; // 按键后等待首次识别的最长时间
constexpr float REFERENCE_CENTER_RADIUS_PX = 15.0f; // [实机必调] 首件圆心距相机中心不超过此值才开始稳定计时
constexpr float REFERENCE_STABILITY_DELTA_PX = 4.0f; // [实机必调] 首件圆心在稳定计时期间允许的最大漂移
constexpr float REFERENCE_DISTANT_RADIUS_PX = 17.0f; // [实机必调] 首件超过此距离时改为间隔跟随，避免连续追踪干涉物料
constexpr uint32_t REFERENCE_DISTANT_REFRESH_MS = 700; // [实机必调] 远距离物料修正刷新间隔
constexpr float PICK_COLOR_CENTER_RADIUS_PX = 28.0f; // [实机必调] 目标颜色圆心距相机中心不超过20 px才允许夹取
constexpr uint32_t REFERENCE_STABLE_TIME_MS = 90; // [实机必调] 首件圆心连续稳定后固定坐标
constexpr uint32_t REFERENCE_FRAME_GAP_MS = 150; // 超过此时间未收到有效视觉帧则重新稳定计时
constexpr uint32_t DETECTION_TO_GRIP_LIMIT_MS = 10000; // 首次发现到夹持完成的目标上限为10秒
constexpr uint32_t GRIPPER_SETTLE_MS = 700;

constexpr float CAMERA_INITIAL_SCALE_MM = 262.0f;
constexpr float CAMERA_ZERO_SCALE_MM = 20.0f;
constexpr float CAMERA_INSTALL_HEIGHT_MM = 282.0f;
constexpr float MATERIAL_TOP_HEIGHT_MM = 147.0f;
constexpr float CAMERA_IMAGE_HEIGHT_PX = 240.0f;
constexpr float ARM_ZERO_LENGTH = 1206.96826f; // M5转轴到手爪中心的距离，单位0.1 mm

constexpr float BASE_MIN_ANGLE = -1800.0f;
constexpr float BASE_MAX_ANGLE = 3450.0f;
constexpr float BASE_MAX_SPEED = 100000.0f;   // [实机必调] M5最大脉冲速度；由1500提高50%
constexpr float BASE_ACCELERATION = 70000.0f; // [实机必调] M5脉冲加速度；由750提高50%
constexpr float BASE_MOTOR_STEPS = 200.0f;
constexpr float BASE_MICROSTEPS = 16.0f;
constexpr float BASE_GEAR_RATIO = 5.0f;
constexpr float BASE_PULSES_PER_TENTH_DEGREE =
    BASE_MOTOR_STEPS * BASE_MICROSTEPS * BASE_GEAR_RATIO / 3600.0f;

constexpr uint8_t START_BUTTON_PIN = PB9;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
constexpr uint32_t START_DELAY_MS = 200; // [实机必调] 按下PB9后开始抓取前的等待时间
constexpr uint32_t POWER_STABILIZE_MS = 1000;

constexpr uint32_t SERIAL_BAUDRATE = 115200;
constexpr uint8_t TTL_RX_PIN = PA3;
constexpr uint8_t TTL_TX_PIN = PA2;
constexpr uint8_t SERVO_RX_PIN = PC7;
constexpr uint8_t SERVO_TX_PIN = PC6;
constexpr uint8_t CAMERA_RX_PIN = PE7;
constexpr uint8_t CAMERA_TX_PIN = PE8;
constexpr uint8_t VERTICAL_ID = 7;
constexpr uint8_t HORIZONTAL_ID = 6;
constexpr uint8_t GRIPPER_ID = 4;
constexpr uint8_t STORAGE_SERVO_ID = 5;

// 底盘电机保持零位目标并持续使能，使小车在机械臂测试期间锁止。
constexpr uint8_t CHASSIS_ENABLE_PIN = PE13;
constexpr uint8_t CHASSIS_DIR_PINS[4] = {PD6, PE9, PD14, PC3_C};
constexpr uint8_t CHASSIS_STEP_PINS[4] = {PD4, PE11, PD15, PA1};

// M5采用STEP/DIR控制，仅用于补偿视觉X方向误差。
constexpr uint8_t BASE_ENABLE_PIN = PE10;
constexpr uint8_t BASE_DIR_PIN = PE15;
constexpr uint8_t BASE_STEP_PIN = PB11;
} // 命名空间Config结束

namespace
{
constexpr float LEAD_SCREW_TENTH_MM_PER_REV = 120.0f;
constexpr float RACK_TENTH_MM_PER_REV =
    36.0f * static_cast<float>(M_PI) * 10.0f;
constexpr uint16_t MICROSTEPS = 16;
constexpr uint16_t MOTOR_STEPS_PER_REV = 200;

float activeTimeCoefficient = 1.0f;
uint32_t placeReturnStartedAt = 0;

void setTimeCoefficient(float coefficient)
{
    activeTimeCoefficient = std::clamp(
        coefficient,
        Config::MIN_TIME_COEFFICIENT,
        Config::MAX_TIME_COEFFICIENT);
}

uint32_t phaseDelayMs(uint32_t baseMs)
{
    return std::max<uint32_t>(1, lroundf(baseMs * activeTimeCoefficient));
}

uint16_t phaseServoTimeMs(uint16_t baseMs)
{
    return static_cast<uint16_t>(std::min<uint32_t>(phaseDelayMs(baseMs), 60000));
}

uint16_t phaseVelocity(uint16_t baseVelocity)
{
    return static_cast<uint16_t>(std::clamp(
        lroundf(baseVelocity / activeTimeCoefficient), 1L, 60000L));
}

uint8_t phaseAcceleration(uint16_t baseAcceleration)
{
    const float scaled = baseAcceleration /
                         (activeTimeCoefficient * activeTimeCoefficient);
    return static_cast<uint8_t>(std::clamp(lroundf(scaled), 1L, 255L));
}

HardwareSerial ttlSerial(Config::TTL_RX_PIN, Config::TTL_TX_PIN);
HardwareSerial servoSerial(Config::SERVO_RX_PIN, Config::SERVO_TX_PIN);
HardwareSerial cameraSerial(Config::CAMERA_RX_PIN, Config::CAMERA_TX_PIN);

TTL_Protocol ttlProtocol(&ttlSerial, Config::SERIAL_BAUDRATE);
TTL_Stepper verticalStepper(Config::VERTICAL_ID, &ttlProtocol);
TTL_Stepper horizontalStepper(Config::HORIZONTAL_ID, &ttlProtocol);

FSUS_Protocol servoProtocol(&servoSerial, Config::SERIAL_BAUDRATE);
FSUS_Servo storageServo(Config::STORAGE_SERVO_ID, &servoProtocol);
FSUS_Servo gripperServo(Config::GRIPPER_ID, &servoProtocol);
FSGP_Gripper gripper(&gripperServo,
                     Config::GRIPPER_OPEN_ANGLE,
                     Config::GRIPPER_CLOSE_ANGLE,
                     Config::GRIPPER_OPEN_MAX_ANGLE);

class BaseAxis
{
public:
    BaseAxis()
        : stepper(AccelStepper::DRIVER,
                  Config::BASE_STEP_PIN,
                  Config::BASE_DIR_PIN)
    {
    }

    void initialize()
    {
        stepper.setEnablePin(Config::BASE_ENABLE_PIN);
        stepper.setPinsInverted(false, false, true);
        stepper.setMinPulseWidth(3);
        stepper.setMaxSpeed(Config::BASE_MAX_SPEED);
        stepper.setAcceleration(Config::BASE_ACCELERATION);
        stepper.setCurrentPosition(0);
        stepper.enableOutputs();
        currentAngle = 0.0f;
    }

    void moveTo(float targetAngle)
    {
        startMoveTo(targetAngle);
        while (service())
        {
        }
    }

    void startMoveTo(float targetAngle)
    {
        stepper.setMaxSpeed(std::min(
            Config::BASE_MAX_SPEED / activeTimeCoefficient, 60000.0f));
        stepper.setAcceleration(std::min(
            Config::BASE_ACCELERATION /
                (activeTimeCoefficient * activeTimeCoefficient),
            60000.0f));
        targetAngle = std::clamp(targetAngle,
                                 Config::BASE_MIN_ANGLE,
                                 Config::BASE_MAX_ANGLE);
        const long targetPulses = lroundf(
            targetAngle * Config::BASE_PULSES_PER_TENTH_DEGREE);
        Serial.print("MOVE M5 target(0.1deg)=");
        Serial.println(targetAngle);
        stepper.moveTo(targetPulses);
    }

    bool service()
    {
        if (stepper.distanceToGo() == 0)
        {
            currentAngle = stepper.currentPosition() /
                           Config::BASE_PULSES_PER_TENTH_DEGREE;
            return false;
        }
        stepper.run();
        return true;
    }

    bool isRunning()
    {
        return stepper.distanceToGo() != 0;
    }

    float position() const
    {
        return currentAngle;
    }

private:
    AccelStepper stepper;
    float currentAngle = 0.0f;
};

BaseAxis baseAxis;

AccelStepper chassis1(AccelStepper::DRIVER,
                      Config::CHASSIS_STEP_PINS[0], Config::CHASSIS_DIR_PINS[0]);
AccelStepper chassis2(AccelStepper::DRIVER,
                      Config::CHASSIS_STEP_PINS[1], Config::CHASSIS_DIR_PINS[1]);
AccelStepper chassis3(AccelStepper::DRIVER,
                      Config::CHASSIS_STEP_PINS[2], Config::CHASSIS_DIR_PINS[2]);
AccelStepper chassis4(AccelStepper::DRIVER,
                      Config::CHASSIS_STEP_PINS[3], Config::CHASSIS_DIR_PINS[3]);

float horizontalCommandPosition = 0.0f;
float verticalCommandPosition = 0.0f;
bool pickupReferenceValid = false;
float pickupReferenceBaseAngle = 0.0f;
float pickupReferenceHorizontalPosition = 0.0f;
// 最终整车批处理时只在进入/离开功能区完整复位；独立ArmTest默认仍逐件复位。
bool preserveArmPoseBetweenMaterials = false;

struct VisionFrame
{
    int dx = 0;
    int dy = 0;
    uint8_t color = 0;
};

class CameraReceiver
{
public:
    void initialize()
    {
        cameraSerial.begin(Config::SERIAL_BAUDRATE);
        const uint8_t resetCommand[4] = {0xAA, 0xFF, 0x00, 0xBB};
        cameraSerial.write(resetCommand, sizeof(resetCommand));
        cameraSerial.flush();
        discard();
        delay(100);
    }

    void stop()
    {
        const uint8_t resetCommand[4] = {0xAA, 0xFF, 0x00, 0xBB};
        cameraSerial.write(resetCommand, sizeof(resetCommand));
        cameraSerial.flush();
        discard();
    }

    void discard()
    {
        while (cameraSerial.available())
        {
            cameraSerial.read();
        }
        count = 0;
    }

    bool poll(VisionFrame &frame)
    {
        while (cameraSerial.available())
        {
            if (consume(static_cast<uint8_t>(cameraSerial.read()), frame))
            {
                return true;
            }
        }
        return false;
    }

private:
    bool consume(uint8_t value, VisionFrame &frame)
    {
        if (value == 0xAA)
        {
            count = 0;
        }
        if (count < sizeof(packet))
        {
            packet[count++] = value;
        }
        if (count != sizeof(packet))
        {
            return false;
        }

        count = 0;
        if (packet[0] != 0xAA || packet[4] != 0xBB)
        {
            return false;
        }
        frame.dx = static_cast<int8_t>(packet[1]);
        frame.dy = static_cast<int8_t>(packet[2]);
        frame.color = packet[3];
        return true;
    }

    uint8_t packet[5] = {0};
    uint8_t count = 0;
};

CameraReceiver camera;

enum class TestState : uint8_t
{
    Waiting,
    Running,
    Passed,
    Failed
};

TestState testState = TestState::Waiting;
bool rawButton = HIGH;
bool stableButton = HIGH;
uint32_t buttonChangedAt = 0;

void initLockedMotor(AccelStepper &motor)
{
    motor.setMinPulseWidth(3);
    motor.setCurrentPosition(0);
    motor.moveTo(0);
    motor.enableOutputs();
}

void lockVehicleAndBase()
{
    pinMode(Config::CHASSIS_ENABLE_PIN, OUTPUT);
    digitalWrite(Config::CHASSIS_ENABLE_PIN, LOW);
    initLockedMotor(chassis1);
    initLockedMotor(chassis2);
    initLockedMotor(chassis3);
    initLockedMotor(chassis4);

    pinMode(Config::BASE_STEP_PIN, OUTPUT);
    pinMode(Config::BASE_DIR_PIN, OUTPUT);
    pinMode(Config::BASE_ENABLE_PIN, OUTPUT);
    digitalWrite(Config::BASE_STEP_PIN, LOW);
    digitalWrite(Config::BASE_DIR_PIN, LOW);
    digitalWrite(Config::BASE_ENABLE_PIN, LOW);
}

void serviceLockedChassis()
{
    chassis1.run();
    chassis2.run();
    chassis3.run();
    chassis4.run();
}

bool startPressed()
{
    const bool sample = digitalRead(Config::START_BUTTON_PIN);
    if (sample != rawButton)
    {
        rawButton = sample;
        buttonChangedAt = millis();
    }
    if (millis() - buttonChangedAt < Config::BUTTON_DEBOUNCE_MS ||
        stableButton == rawButton)
    {
        return false;
    }
    stableButton = rawButton;
    return stableButton == LOW;
}

void clearTtlReceiveBuffer()
{
    while (ttlSerial.available())
    {
        ttlSerial.read();
    }
}

void moveHorizontal(float target)
{
    target = std::clamp(target,
                        Config::HORIZONTAL_MIN_POSITION,
                        Config::HORIZONTAL_MAX_POSITION);
    const bool negativeTarget = target < 0.0f;
    const uint8_t direction = negativeTarget
                                  ? static_cast<uint8_t>(Config::HORIZONTAL_POSITIVE_DIRECTION ^ 1U)
                                  : Config::HORIZONTAL_POSITIVE_DIRECTION;
    const uint32_t pulses = lroundf(
        fabsf(target) * MICROSTEPS * MOTOR_STEPS_PER_REV /
        RACK_TENTH_MM_PER_REV);

    Serial.print("MOVE ID6 target(0.1mm)/direction=");
    Serial.print(target);
    Serial.print('/');
    Serial.println(direction);
    clearTtlReceiveBuffer();
    ttlProtocol.Emm_V5_Pos_Control(Config::HORIZONTAL_ID,
                                   direction,
                                   phaseVelocity(Config::HORIZONTAL_VELOCITY),
                                   phaseAcceleration(Config::HORIZONTAL_ACCELERATION),
                                   pulses, true, false);
    ttlSerial.flush();
    delay(phaseDelayMs(Config::HORIZONTAL_SETTLE_MS));
    clearTtlReceiveBuffer();
    horizontalCommandPosition = target;
}

void startHorizontalMove(float target)
{
    target = std::clamp(target,
                        Config::HORIZONTAL_MIN_POSITION,
                        Config::HORIZONTAL_MAX_POSITION);
    const bool negativeTarget = target < 0.0f;
    const uint8_t direction = negativeTarget
                                  ? static_cast<uint8_t>(Config::HORIZONTAL_POSITIVE_DIRECTION ^ 1U)
                                  : Config::HORIZONTAL_POSITIVE_DIRECTION;
    const uint32_t pulses = lroundf(
        fabsf(target) * MICROSTEPS * MOTOR_STEPS_PER_REV /
        RACK_TENTH_MM_PER_REV);
    clearTtlReceiveBuffer();
    ttlProtocol.Emm_V5_Pos_Control(Config::HORIZONTAL_ID,
                                   direction,
                                   phaseVelocity(Config::HORIZONTAL_VELOCITY),
                                   phaseAcceleration(Config::HORIZONTAL_ACCELERATION),
                                   pulses, true, false);
    ttlSerial.flush();
    horizontalCommandPosition = target;
}

void startVerticalMove(float target, uint16_t velocity, uint16_t acceleration)
{
    target = std::max(target, 0.0f);
    const uint32_t pulses = lroundf(
        target * MICROSTEPS * MOTOR_STEPS_PER_REV /
        LEAD_SCREW_TENTH_MM_PER_REV);
    clearTtlReceiveBuffer();
    ttlProtocol.Emm_V5_Pos_Control(Config::VERTICAL_ID,
                                   Config::VERTICAL_POSITIVE_DIRECTION,
                                   phaseVelocity(velocity),
                                   phaseAcceleration(acceleration),
                                   pulses, true, false);
    ttlSerial.flush();
    verticalCommandPosition = target;
}

void moveVertical(float target, uint16_t velocity, uint16_t acceleration)
{
    target = std::max(target, 0.0f);
    const uint32_t pulses = lroundf(
        target * MICROSTEPS * MOTOR_STEPS_PER_REV /
        LEAD_SCREW_TENTH_MM_PER_REV);

    Serial.print("MOVE ID7 target(0.1mm)=");
    Serial.print(target);
    Serial.print(" velocity/acceleration=");
    Serial.print(velocity);
    Serial.print('/');
    Serial.println(acceleration);
    clearTtlReceiveBuffer();
    ttlProtocol.Emm_V5_Pos_Control(Config::VERTICAL_ID,
                                   Config::VERTICAL_POSITIVE_DIRECTION,
                                   phaseVelocity(velocity),
                                   phaseAcceleration(acceleration),
                                   pulses, true, false);
    ttlSerial.flush();
    delay(phaseDelayMs(Config::VERTICAL_SETTLE_MS));
    clearTtlReceiveBuffer();
    verticalCommandPosition = target;
}

bool waitVerticalArrival()
{
    clearTtlReceiveBuffer();
    verticalStepper.onPos_state = false;
    verticalStepper.recDate_Clear();
    const uint32_t startedAt = millis();

    while (millis() - startedAt < Config::VERTICAL_ARRIVAL_TIMEOUT_MS)
    {
        verticalStepper.state_update();
        if (verticalStepper.onPos_state)
        {
            Serial.println("ID7: target height arrival confirmed");
            delay(phaseDelayMs(Config::PICK_HEIGHT_STABLE_MS));
            return true;
        }
        delay(2);
    }

    Serial.println("ID7 WARNING: no arrival reply, continue after fixed move wait");
    verticalStepper.recDate_Clear();
    delay(phaseDelayMs(Config::PICK_HEIGHT_STABLE_MS));
    return true;
}

void moveHorizontalBy(float deltaTenthMm)
{
    moveHorizontal(horizontalCommandPosition + deltaTenthMm);
}

float updateVisionScale(float currentHeightMm)
{
    const float cameraHeight =
        Config::CAMERA_INSTALL_HEIGHT_MM - currentHeightMm;
    const float deltaHeight = cameraHeight - Config::MATERIAL_TOP_HEIGHT_MM;
    return (((Config::CAMERA_INITIAL_SCALE_MM - Config::CAMERA_ZERO_SCALE_MM) /
                 Config::CAMERA_INSTALL_HEIGHT_MM * deltaHeight) +
            Config::CAMERA_ZERO_SCALE_MM) /
           Config::CAMERA_IMAGE_HEIGHT_PX;
}

void applyVisionCorrection(float moveXmm, float moveYmm)
{
    // 不限制单次视觉修正量；只在最终绝对目标处保留机械行程保护。
    const float tangentialCorrection =
        moveXmm * 10.0f * Config::VISION_X_GAIN *
        Config::BASE_DIRECTION_SIGN;
    const float radialCorrection =
        moveYmm * 10.0f * Config::VISION_Y_GAIN *
        Config::HORIZONTAL_DIRECTION_SIGN;
    const float horizontalTarget = std::clamp(
        horizontalCommandPosition + radialCorrection,
        Config::HORIZONTAL_MIN_POSITION,
        Config::HORIZONTAL_MAX_POSITION);
    const float armRadius = std::max(
        Config::ARM_ZERO_LENGTH + horizontalTarget, 1.0f);
    float angleCorrection =
        std::atan2(tangentialCorrection, armRadius) *
        (1800.0f / static_cast<float>(M_PI));

    baseAxis.moveTo(baseAxis.position() + angleCorrection);
    moveHorizontal(horizontalTarget);
    delay(phaseDelayMs(Config::VISION_SETTLE_MS));
}

void closeGripperAndWait()
{
    delay(Config::GROUND_PICK_BEFORE_GRIPPER_CLOSE_MS);
    Serial.println("PICKUP: command ID4 gripper close");
    gripper.setAngle(Config::GRIPPER_CLOSE_ANGLE,
                     phaseServoTimeMs(Config::GRIPPER_CLOSE_TIME_MS),
                     Config::GRIPPER_MAX_POWER_MW);
    delay(phaseDelayMs(Config::GROUND_PICK_AFTER_GRIPPER_CLOSE_MS));
    Serial.println("PICKUP: ID4 close delay complete, lift immediately");
}

void openGripperForStorageRelease()
{
    delay(Config::ARM_BEFORE_GRIPPER_SETTLE_MS);
    Serial.print("STORAGE RELEASE: open gripper to angle=");
    Serial.println(Config::STORAGE_RELEASE_ANGLE);
    gripper.setAngle(Config::STORAGE_RELEASE_ANGLE,
                     phaseServoTimeMs(Config::GRIPPER_CLOSE_TIME_MS),
                     Config::GRIPPER_MAX_POWER_MW);
    delay(phaseDelayMs(Config::GRIPPER_SETTLE_MS));
}

bool initializePickupHardware()
{
    Serial.println("INIT: wait for actuator power");
    delay(Config::POWER_STABILIZE_MS);

    // 将上电时的机械朝向定义为M5软件零点，启动时不执行固定角度旋转。
    baseAxis.initialize();

    ttlProtocol.init(&ttlSerial, Config::SERIAL_BAUDRATE);
    verticalStepper.init();
    horizontalStepper.init();
    verticalStepper.set(Config::M7_LIFT_VELOCITY,
                        phaseAcceleration(Config::M7_LIFT_ACCELERATION),
                        Config::VERTICAL_POSITIVE_DIRECTION,
                        LEAD_SCREW_TENTH_MM_PER_REV, MICROSTEPS);
    horizontalStepper.set(Config::HORIZONTAL_VELOCITY,
                          Config::HORIZONTAL_ACCELERATION,
                          Config::HORIZONTAL_POSITIVE_DIRECTION,
                          RACK_TENTH_MM_PER_REV, MICROSTEPS);

    ttlProtocol.Emm_V5_En_Control(Config::HORIZONTAL_ID, true, false);
    delay(100);
    ttlProtocol.Emm_V5_En_Control(Config::VERTICAL_ID, true, false);
    delay(100);
    ttlProtocol.Emm_V5_Reset_CurPos_To_Zero(Config::HORIZONTAL_ID);
    delay(50);
    ttlProtocol.Emm_V5_Reset_CurPos_To_Zero(Config::VERTICAL_ID);
    delay(100);
    horizontalCommandPosition = 0.0f;
    verticalCommandPosition = 0.0f;

    servoProtocol.init(&servoSerial, Config::SERIAL_BAUDRATE);
    storageServo.init();
    // 只检测夹爪舵机是否在线，不调用gripper.init()，避免进入功能区时自动张爪。
    gripperServo.init();
    if (!gripperServo.isOnline)
    {
        Serial.println("INIT ERROR: ID4 gripper servo is offline");
        return false;
    }
    storageServo.setSpeed(Config::STORAGE_SERVO_SPEED);
    gripper.setMaxPower(Config::GRIPPER_MAX_POWER_MW);
    Serial.print("INIT: storage servo ID5 angle(deg)=");
    Serial.println(Config::STORAGE_INITIAL_ANGLE);
    storageServo.setRawAngle(Config::STORAGE_INITIAL_ANGLE);
    delay(Config::STORAGE_SETTLE_MS);
    camera.initialize();
    Serial.println("INIT: complete; M5 visual compensation enabled");
    return true;
}

// 整车正式流程已经在启停区完成公共机械臂初始化，进入原料区时只绑定
// RawPickupRuntime自己的通信和软件状态。这里禁止重复使能、重新定义M6/M7
// 硬件零点或转动ID5储料盘，否则会破坏行驶期间一直保持的坐标基准。
bool preparePickupHardwareForFinalTask()
{
    Serial.println("RAW PREPARE: bind controllers without hardware re-zero");

    // RawPickupRuntime与数字区当前仍各自持有一个M5驱动对象。每次进入原料区
    // 都将Raw对象的软件坐标同步为当前工作零位。initialize()只配置STEP/DIR
    // 并设置软件当前位置，不调用moveTo()，因此不会产生M5旋转脉冲。
    // RobotApp在调用本函数前已经确认离区复位完成。
    baseAxis.initialize();

    ttlProtocol.init(&ttlSerial, Config::SERIAL_BAUDRATE);
    verticalStepper.init();
    horizontalStepper.init();
    verticalStepper.set(Config::M7_LIFT_VELOCITY,
                        phaseAcceleration(Config::M7_LIFT_ACCELERATION),
                        Config::VERTICAL_POSITIVE_DIRECTION,
                        LEAD_SCREW_TENTH_MM_PER_REV, MICROSTEPS);
    horizontalStepper.set(Config::HORIZONTAL_VELOCITY,
                          Config::HORIZONTAL_ACCELERATION,
                          Config::HORIZONTAL_POSITIVE_DIRECTION,
                          RACK_TENTH_MM_PER_REV, MICROSTEPS);

    // 到达原料区前RobotApp必须等待异步复位完成，因此仅同步软件坐标，
    // 不再向ID6/ID7发送使能或当前位置清零命令。
    horizontalCommandPosition = Config::HORIZONTAL_RESET_POSITION;
    verticalCommandPosition = Config::VERTICAL_HIGH_POSITION;

    // 数字区与原料区共用物理串口，但使用不同协议对象；切换任务时需重新绑定。
    // init()只读取舵机状态，不发送角度运动命令。
    servoProtocol.init(&servoSerial, Config::SERIAL_BAUDRATE);
    storageServo.init();
    gripperServo.init();
    if (!gripperServo.isOnline)
    {
        Serial.println("RAW PREPARE ERROR: ID4 gripper servo is offline");
        return false;
    }
    storageServo.setSpeed(Config::STORAGE_SERVO_SPEED);
    gripper.setMaxPower(Config::GRIPPER_MAX_POWER_MW);

    camera.initialize();
    Serial.println("RAW PREPARE: complete; keep current ID5 angle and hardware zeros");
    return true;
}

float storageSlotAngle(uint8_t slotNumber)
{
    slotNumber = constrain(slotNumber, 1, Config::STORAGE_SLOT_COUNT);
    return Config::STORAGE_SERVO_ANGLES[slotNumber - 1];
}

float storagePlaceM5Angle(uint8_t slotNumber)
{
    slotNumber = constrain(slotNumber, 1, Config::STORAGE_SLOT_COUNT);
    return Config::STORAGE_PLACE_M5_ANGLES[slotNumber - 1];
}

float storagePlaceM6Position(uint8_t slotNumber)
{
    slotNumber = constrain(slotNumber, 1, Config::STORAGE_SLOT_COUNT);
    return Config::STORAGE_PLACE_M6_POSITIONS[slotNumber - 1];
}

// 储料盘或M5转动前的统一防干涉时序：
// 1. M7先升到高位；2. M6完全收回；3. 调用方才允许转动ID5储料盘和M5底座。
void prepareArmForStorageRotation()
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
    Serial.println("STORAGE ROTATION SAFE: ID7 high, M6 retracted");
}

void placeMaterialOnStorageSlot(uint8_t slotNumber)
{
    slotNumber = constrain(slotNumber, 1, Config::STORAGE_SLOT_COUNT);
    Serial.print("PLACE: StorageTest flow, slot ");
    Serial.println(slotNumber);
    Serial.print("PLACE COORDINATE M5(0.1deg)/M6(0.1mm)=");
    Serial.print(storagePlaceM5Angle(slotNumber));
    Serial.print('/');
    Serial.println(storagePlaceM6Position(slotNumber));

    // 先在高位收回M6，再转动储料盘和M5，避免运动路径发生干涉。
    // 无论是否连续处理物料，每次转盘和转M5前都必须先在高位收回M6。
    prepareArmForStorageRotation();

    storageServo.setSpeed(Config::STORAGE_SERVO_SPEED);
    storageServo.setRawAngle(storageSlotAngle(slotNumber));
    storageServo.wait();
    baseAxis.moveTo(storagePlaceM5Angle(slotNumber));
    // StorageTest放料到达深度后直接松爪，不增加取料停稳等待。
    if (fabsf(storagePlaceM6Position(slotNumber) -
              horizontalCommandPosition) > 0.5f)
    {
        moveHorizontal(storagePlaceM6Position(slotNumber));
    }
    moveVertical(Config::VERTICAL_PLACE_POSITION,
                 Config::M7_STORAGE_PLACE_VELOCITY,
                 Config::M7_STORAGE_PLACE_ACCELERATION);
    Serial.println("PLACE: release material");
    openGripperForStorageRelease();

    // 连续批处理只抬升M7，M5/M6保持当前坐标；离开功能区前再完整复位。
    moveVertical(Config::VERTICAL_HIGH_POSITION,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);
    if (!preserveArmPoseBetweenMaterials &&
        fabsf(horizontalCommandPosition - Config::HORIZONTAL_RESET_POSITION) > 0.5f)
    {
        moveHorizontal(Config::HORIZONTAL_RESET_POSITION);
    }
    if (!preserveArmPoseBetweenMaterials && fabsf(baseAxis.position()) > 0.5f)
    {
        baseAxis.moveTo(0.0f);
    }
    Serial.println(preserveArmPoseBetweenMaterials
                       ? "PLACE: ID7 high; keep M5/M6 for next material"
                       : "PLACE: arm reset complete, ID7 high, M6=0, M5=0");

    if (slotNumber == Config::STORAGE_SLOT_COUNT)
    {
        // 第3件完成后保持储料盘末位；真正离开功能区时再与机械臂同时复位。
        Serial.println("PLACE: all slots complete, storage tray reset deferred until area exit");
    }
}

const char *colorName(uint8_t color)
{
    switch (color)
    {
    case 2:
        return "Yellow";
    case 3:
        return "Blue";
    case 4:
        return "Green";
    default:
        return "Unknown";
    }
}

bool pickRecognizedMaterial(uint8_t expectedColor,
                            uint8_t storageSlotNumber,
                            bool keepPreparedGripperAngle = false)
{
    setTimeCoefficient(Config::PICKUP_TIME_COEFFICIENT);
    const bool usePickupReference = pickupReferenceValid;
    const uint32_t pickupPhaseStartedAt = millis();
    uint8_t state = 0x00;
    uint32_t detectedAt = 0;
    bool referenceStabilityStarted = false;
    int referenceAnchorDx = 0;
    int referenceAnchorDy = 0;
    uint32_t referenceStabilityStartedAt = 0;
    uint32_t referenceLastFrameAt = 0;
    bool distantCorrectionStarted = false;
    uint32_t lastDistantCorrectionAt = 0;
    const uint32_t searchStartedAt = millis();

    camera.discard();
    while (millis() - searchStartedAt < Config::MATERIAL_SEARCH_TIMEOUT_MS)
    {
        switch (state)
        {
        case 0x00:
        {
            Serial.println("STATE 0x00: prepare pickup pose");
            if (keepPreparedGripperAngle)
            {
                // 第二轮首件已在进入原料区时张到-90度，此处保持该角度。
                Serial.println("SECOND RAW FIRST PICK: keep ID4 at -90 deg");
            }
            else
            {
                // 放入储料盘时夹爪只张到-50度；其他抓取前仍完全张到-80度。
                gripper.openMax();
                delay(phaseDelayMs(Config::GROUND_PICK_GRIPPER_OPEN_SETTLE_MS));
            }
            if (usePickupReference)
            {
                // 上一件放料结束时ID7已在高位，M5/M6直接从盘位坐标转到首件固定坐标。
                Serial.println("NEXT MATERIAL: move directly to fixed half-height pose");
                Serial.print("FIXED PICK COORDINATE: M5(0.1deg)=");
                Serial.print(pickupReferenceBaseAngle);
                Serial.print(", ID6(0.1mm)=");
                Serial.println(pickupReferenceHorizontalPosition);
                baseAxis.moveTo(pickupReferenceBaseAngle);
                moveHorizontal(pickupReferenceHorizontalPosition);
            }
            else
            {
                Serial.println("FIRST MATERIAL: lower to half height before initial M6 position");
            }
            const float halfPickPosition = Config::PICK_REFERENCE_POSITION;
            Serial.println("ZERO HEIGHT: no vision correction");
            Serial.print("ZERO HEIGHT: lower directly to half pickup depth=");
            Serial.println(halfPickPosition);
            moveVertical(halfPickPosition,
                         Config::M7_REFERENCE_VELOCITY,
                         Config::M7_REFERENCE_ACCELERATION);
            if (!waitVerticalArrival())
            {
                return false;
            }
            if (!usePickupReference)
            {
                moveHorizontal(Config::HORIZONTAL_INITIAL_POSITION);
            }
            camera.discard();
            state = 0x18;
            break;
        }

        case 0x18:
        {
            VisionFrame frame;
            if (!camera.poll(frame))
            {
                if (!usePickupReference && referenceStabilityStarted &&
                    millis() - referenceLastFrameAt > Config::REFERENCE_FRAME_GAP_MS)
                {
                    referenceStabilityStarted = false;
                    Serial.println("HALF HEIGHT: frame gap, restart stability timer");
                }
                break;
            }
            if (frame.color < 1 || frame.color > 6 ||
                (usePickupReference && frame.color != expectedColor))
            {
                break;
            }
            if (detectedAt == 0)
            {
                detectedAt = millis();
            }
            referenceLastFrameAt = millis();

            const float scale = updateVisionScale(
                verticalCommandPosition / 10.0f);
            const float moveXmm = frame.dx * scale;
            const float moveYmm = frame.dy * scale;
            const float centerDistancePx = sqrtf(
                static_cast<float>(frame.dx * frame.dx + frame.dy * frame.dy));
            if (!usePickupReference &&
                centerDistancePx > Config::REFERENCE_DISTANT_RADIUS_PX)
            {
                referenceStabilityStarted = false;
                const uint32_t now = millis();
                if (!distantCorrectionStarted ||
                    now - lastDistantCorrectionAt >=
                        Config::REFERENCE_DISTANT_REFRESH_MS)
                {
                    Serial.print("HALF HEIGHT: timed distant correction, radius(px)=");
                    Serial.println(centerDistancePx);
                    applyVisionCorrection(moveXmm, moveYmm);
                    camera.discard();
                    distantCorrectionStarted = true;
                    lastDistantCorrectionAt = millis();
                }
                break;
            }
            Serial.print("HALF HEIGHT: color arrived: ");
            Serial.print(colorName(frame.color));
            Serial.print("; vision X/Y(mm)=");
            Serial.print(moveXmm);
            Serial.print('/');
            Serial.println(moveYmm);
            if (!usePickupReference)
            {
                if (centerDistancePx > Config::REFERENCE_CENTER_RADIUS_PX)
                {
                    referenceStabilityStarted = false;
                    Serial.print("HALF HEIGHT: center distance(px)=");
                    Serial.print(centerDistancePx);
                    Serial.println("; correct M5/ID6 and restart stability timer");
                    applyVisionCorrection(moveXmm, moveYmm);
                    camera.discard();
                    break;
                }

                if (!referenceStabilityStarted)
                {
                    referenceStabilityStarted = true;
                    referenceAnchorDx = frame.dx;
                    referenceAnchorDy = frame.dy;
                    referenceStabilityStartedAt = millis();
                    Serial.println("HALF HEIGHT: centered; start stability timer");
                    break;
                }

                const float stabilityDeltaPx = sqrtf(
                    static_cast<float>((frame.dx - referenceAnchorDx) *
                                           (frame.dx - referenceAnchorDx) +
                                       (frame.dy - referenceAnchorDy) *
                                           (frame.dy - referenceAnchorDy)));
                if (stabilityDeltaPx > Config::REFERENCE_STABILITY_DELTA_PX)
                {
                    referenceAnchorDx = frame.dx;
                    referenceAnchorDy = frame.dy;
                    referenceStabilityStartedAt = millis();
                    Serial.print("HALF HEIGHT: center changed(px)=");
                    Serial.print(stabilityDeltaPx);
                    Serial.println("; restart stability timer");
                    break;
                }

                if (millis() - referenceStabilityStartedAt <
                    Config::REFERENCE_STABLE_TIME_MS)
                {
                    break;
                }

                pickupReferenceBaseAngle = baseAxis.position();
                pickupReferenceHorizontalPosition = horizontalCommandPosition;
                pickupReferenceValid = true;
                Serial.print("PICKUP REFERENCE SAVED AT HALF HEIGHT: M5(0.1deg)=");
                Serial.print(pickupReferenceBaseAngle);
                Serial.print(", ID6(0.1mm)=");
                Serial.println(pickupReferenceHorizontalPosition);
                Serial.print("REFERENCE FIXED AT HALF HEIGHT: wait required color ");
                Serial.println(colorName(expectedColor));
                camera.discard();
                state = 0x24;
                break;
            }
            else
            {
                if (centerDistancePx > Config::PICK_COLOR_CENTER_RADIUS_PX)
                {
                    break;
                }
                Serial.println("HALF HEIGHT: color arrived; keep first material coordinate");
            }

            if (!pickupReferenceValid)
            {
                Serial.println("PICK ERROR: reference coordinate is not fixed; block final descent");
                return false;
            }
            Serial.println("HALF HEIGHT: lower to final pickup depth");
            moveVertical(Config::VERTICAL_PICK_POSITION,
                         Config::M7_PICK_VELOCITY,
                         Config::M7_PICK_ACCELERATION);
            if (!waitVerticalArrival())
            {
                return false;
            }
            Serial.println("TARGET READY: close ID4 gripper without a second low-position wait");
            closeGripperAndWait();
            Serial.print("DETECTION TO GRIP(ms)=");
            Serial.println(millis() - detectedAt);
            state = 0x33;
            break;
        }

        case 0x22:
        {
            VisionFrame frame;
            if (!camera.poll(frame))
            {
                if (!usePickupReference && referenceStabilityStarted &&
                    millis() - referenceLastFrameAt > Config::REFERENCE_FRAME_GAP_MS)
                {
                    referenceStabilityStarted = false;
                    Serial.println("REFERENCE: frame gap, restart stability timer");
                }
                break;
            }

            if (frame.color < 1 || frame.color > 6 ||
                (usePickupReference && frame.color != expectedColor))
            {
                break;
            }

            referenceLastFrameAt = millis();

            const float scale = updateVisionScale(
                verticalCommandPosition / 10.0f);
            const float moveXmm = frame.dx * scale;
            const float moveYmm = frame.dy * scale;
            const float centerDistancePx = sqrtf(
                static_cast<float>(frame.dx * frame.dx + frame.dy * frame.dy));

            Serial.print("PICKUP DEPTH vision X/Y(mm)=");
            Serial.print(moveXmm);
            Serial.print('/');
            Serial.println(moveYmm);
            if (!usePickupReference)
            {
                if (centerDistancePx > Config::REFERENCE_DISTANT_RADIUS_PX)
                {
                    referenceStabilityStarted = false;
                    const uint32_t now = millis();
                    if (!distantCorrectionStarted ||
                        now - lastDistantCorrectionAt >=
                            Config::REFERENCE_DISTANT_REFRESH_MS)
                    {
                        Serial.print("REFERENCE: timed distant correction, radius(px)=");
                        Serial.println(centerDistancePx);
                        applyVisionCorrection(moveXmm, moveYmm);
                        camera.discard();
                        distantCorrectionStarted = true;
                        lastDistantCorrectionAt = millis();
                    }
                    break;
                }

                if (centerDistancePx > Config::REFERENCE_CENTER_RADIUS_PX)
                {
                    referenceStabilityStarted = false;
                    Serial.print("REFERENCE: center distance(px)=");
                    Serial.print(centerDistancePx);
                    Serial.println("; correct M5/ID6 and restart stability timer");
                    applyVisionCorrection(moveXmm, moveYmm);
                    camera.discard();
                    break;
                }

                if (!referenceStabilityStarted)
                {
                    referenceStabilityStarted = true;
                    referenceAnchorDx = frame.dx;
                    referenceAnchorDy = frame.dy;
                    referenceStabilityStartedAt = millis();
                    Serial.println("REFERENCE: centered; start stability timer");
                    break;
                }

                const float stabilityDeltaPx = sqrtf(
                    static_cast<float>((frame.dx - referenceAnchorDx) *
                                           (frame.dx - referenceAnchorDx) +
                                       (frame.dy - referenceAnchorDy) *
                                           (frame.dy - referenceAnchorDy)));
                if (stabilityDeltaPx > Config::REFERENCE_STABILITY_DELTA_PX)
                {
                    referenceAnchorDx = frame.dx;
                    referenceAnchorDy = frame.dy;
                    referenceStabilityStartedAt = millis();
                    Serial.print("REFERENCE: center changed(px)=");
                    Serial.print(stabilityDeltaPx);
                    Serial.println("; restart stability timer");
                    break;
                }

                if (millis() - referenceStabilityStartedAt <
                    Config::REFERENCE_STABLE_TIME_MS)
                {
                    break;
                }

                pickupReferenceBaseAngle = baseAxis.position();
                pickupReferenceHorizontalPosition = horizontalCommandPosition;
                pickupReferenceValid = true;
                Serial.print("PICKUP REFERENCE SAVED: M5(0.1deg)=");
                Serial.print(pickupReferenceBaseAngle);
                Serial.print(", ID6(0.1mm)=");
                Serial.println(pickupReferenceHorizontalPosition);
                Serial.print("REFERENCE FIXED: wait required color ");
                Serial.println(colorName(expectedColor));
                camera.discard();
                state = 0x24;
                break;
            }
            else
            {
                if (centerDistancePx > Config::PICK_COLOR_CENTER_RADIUS_PX)
                {
                    break;
                }
                Serial.println("PICKUP DEPTH: target color arrived; use first material coordinate");
            }
            Serial.println("STATE 0x22: close ID4 gripper immediately");
            closeGripperAndWait();
            Serial.print("DETECTION TO GRIP(ms)=");
            Serial.println(millis() - detectedAt);
            if (millis() - detectedAt > Config::DETECTION_TO_GRIP_LIMIT_MS)
            {
                Serial.println("WARNING: detection-to-grip exceeded 10 seconds");
            }
            state = 0x33;
            break;
        }

        case 0x24:
        {
            VisionFrame frame;
            if (!camera.poll(frame) || frame.color != expectedColor)
            {
                break;
            }
            const float centerDistancePx = sqrtf(
                static_cast<float>(frame.dx * frame.dx + frame.dy * frame.dy));
            if (centerDistancePx > Config::PICK_COLOR_CENTER_RADIUS_PX)
            {
                break;
            }
            Serial.print("HALF HEIGHT: required color arrived: ");
            Serial.println(colorName(expectedColor));
            Serial.println("TARGET READY: lower to final pickup depth and grip");
            moveVertical(Config::VERTICAL_PICK_POSITION,
                         Config::M7_PICK_VELOCITY,
                         Config::M7_PICK_ACCELERATION);
            if (!waitVerticalArrival())
            {
                return false;
            }
            closeGripperAndWait();
            Serial.print("DETECTION TO GRIP(ms)=");
            Serial.println(millis() - detectedAt);
            state = 0x33;
            break;
        }

        case 0x33:
            Serial.println("STATE 0x33: lift material using pickup time coefficient");
            camera.discard();
            moveVertical(Config::VERTICAL_HIGH_POSITION,
                         Config::M7_LIFT_VELOCITY,
                         Config::M7_LIFT_ACCELERATION);
            Serial.print("PICKUP PHASE(ms, including lift)=");
            Serial.println(millis() - pickupPhaseStartedAt);

            setTimeCoefficient(Config::PLACE_RETURN_TIME_COEFFICIENT);
            placeReturnStartedAt = millis();
            Serial.print("STATE 0x33: place material on storage slot ");
            Serial.println(storageSlotNumber);
            Serial.println("STATE 0x33: short hold at high position");
            delay(phaseDelayMs(Config::POST_PICK_HIGH_HOLD_MS));
            placeMaterialOnStorageSlot(storageSlotNumber);
            return true;

        default:
            return false;
        }
        serviceLockedChassis();
        delay(1);
    }

    Serial.print("PICK ERROR: timeout waiting for expected color ");
    Serial.print(colorName(expectedColor));
    Serial.print('(');
    Serial.print(expectedColor);
    Serial.println("); lift to high position");
    moveVertical(Config::VERTICAL_HIGH_POSITION,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);
    return false;
}

void prepareNextMaterialPickup()
{
    setTimeCoefficient(Config::PLACE_RETURN_TIME_COEFFICIENT);
    Serial.println("NEXT MATERIAL: skip zero-position transfer");
    camera.discard();
    if (placeReturnStartedAt != 0)
    {
        Serial.print("PLACE+RETURN PHASE(ms)=");
        Serial.println(millis() - placeReturnStartedAt);
        placeReturnStartedAt = 0;
    }
}

void resetForNextPickup()
{
    setTimeCoefficient(Config::PLACE_RETURN_TIME_COEFFICIENT);
    Serial.println("RESET: return arm to next-pick pose");
    camera.stop();
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
    if (fabsf(baseAxis.position()) > 0.5f)
    {
        baseAxis.moveTo(0.0f);
    }

    storageServo.setRawAngle(Config::STORAGE_INITIAL_ANGLE);
    delay(phaseDelayMs(Config::STORAGE_SETTLE_MS));
    gripper.openMax();
    delay(phaseDelayMs(Config::GRIPPER_SETTLE_MS));

    digitalWrite(Config::CHASSIS_ENABLE_PIN, LOW);
    digitalWrite(Config::BASE_ENABLE_PIN, LOW);
    if (placeReturnStartedAt != 0)
    {
        Serial.print("PLACE+RETURN PHASE(ms)=");
        Serial.println(millis() - placeReturnStartedAt);
        placeReturnStartedAt = 0;
    }
    Serial.println("RESET: complete, release and press PB9 for next pickup");
}

void runOnce()
{
    testState = TestState::Running;
    Serial.println("START: fixed color order Yellow/Blue/Green with M5 compensation");
    delay(Config::START_DELAY_MS);

    if (!initializePickupHardware())
    {
        testState = TestState::Failed;
        Serial.println("RESULT: FAIL - hardware initialization");
        return;
    }

    pickupReferenceValid = false;
    pickupReferenceBaseAngle = 0.0f;
    pickupReferenceHorizontalPosition = 0.0f;

    bool pickupPassed = true;
    for (uint8_t slot = 1; slot <= Config::STORAGE_SLOT_COUNT; ++slot)
    {
        if (slot > 1)
        {
            prepareNextMaterialPickup();
        }
        const uint8_t expectedColor = Config::PICK_COLOR_SEQUENCE[slot - 1];
        Serial.print("BATCH PICK: ");
        Serial.print(colorName(expectedColor));
        Serial.print('(');
        Serial.print(expectedColor);
        Serial.print(") -> storage slot ");
        Serial.println(slot);
        if (!pickRecognizedMaterial(expectedColor, slot))
        {
            pickupPassed = false;
            break;
        }
    }

    if (pickupPassed)
    {
        testState = TestState::Passed;
        Serial.println("RESULT: PASS - 3 materials placed on storage slots 1/2/3");
    }
    else
    {
        testState = TestState::Failed;
        Serial.println("RESULT: FAIL - camera or pickup state machine");
    }

    resetForNextPickup();
    testState = TestState::Waiting;
}
} // 匿名命名空间结束

#ifndef ARMTEST_SETUP_NAME
#define ARMTEST_SETUP_NAME setup
#endif
#ifndef ARMTEST_LOOP_NAME
#define ARMTEST_LOOP_NAME loop
#endif

void ARMTEST_SETUP_NAME()
{
    Serial.begin(Config::SERIAL_BAUDRATE);
    pinMode(Config::START_BUTTON_PIN, INPUT_PULLUP);
    rawButton = stableButton = digitalRead(Config::START_BUTTON_PIN);
    buttonChangedAt = millis();
    lockVehicleAndBase();

    delay(200);
    Serial.println();
    Serial.println("=== ARMTEST: VISUAL PICK DEBUG ===");
    Serial.println("Before PB9: ID6 near, ID7 high, M5 at pickup heading.");
    Serial.println("PB9 picks 3 recognized materials into storage slots 1/2/3.");
    Serial.println("M5 only performs visual micro-adjustment; chassis motors remain locked.");
}

void ARMTEST_LOOP_NAME()
{
    serviceLockedChassis();
    if (testState == TestState::Waiting && startPressed())
    {
        runOnce();
    }
    delay(1);
}
