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
// -------------------- 直接复制自ArmTest的放料参数 --------------------
constexpr int GRIPPER_OPEN_ANGLE = -30;
constexpr int GRIPPER_CLOSE_ANGLE = 45;
constexpr int GRIPPER_OPEN_MAX_ANGLE = -80;
constexpr int STORAGE_RELEASE_ANGLE =
    ArmConfig::StorageReturn::GRIPPER_RELEASE_ANGLE_DEG;
constexpr uint16_t GRIPPER_MAX_POWER_MW = 400;
constexpr uint16_t GRIPPER_CLOSE_TIME_MS = 300;
constexpr uint32_t GRIPPER_TO_MOVE_DELAY_MS = 100;
constexpr uint32_t GRIPPER_SETTLE_MS = 200;

#ifdef PICK_STORAGE_TEST
constexpr uint32_t PICK_GRAB_SETTLE_MS =
    ArmConfig::StorageReturn::PLACE_ARRIVAL_SETTLE_MS;
// [实机必调] PickStorageTest专用参数，不影响StorageTest。
constexpr int PICK_GRIPPER_OPEN_ANGLE = -50;         // 取料前及复位后的夹爪张开角度，单位度
static_assert(PICK_GRIPPER_OPEN_ANGLE >= -180 &&
                  PICK_GRIPPER_OPEN_ANGLE <= 180,
              "PICK_GRIPPER_OPEN_ANGLE must use degrees (-180..180)");
constexpr float PICK_VERTICAL_GRAB_POSITION =
    ArmConfig::StorageReturn::M7_PLACE_POSITION_TENTH_MM;
#endif

constexpr uint8_t STORAGE_SLOT_COUNT = ArmConfig::StorageReturn::SLOT_COUNT;
static constexpr const float (&STORAGE_SERVO_ANGLES)[STORAGE_SLOT_COUNT] =
    ArmConfig::StorageReturn::ID5_SLOT_ANGLES_DEG;
constexpr float STORAGE_INITIAL_ANGLE = -91.0f; // [实机必调] 上电及每次测试结束后的储料盘复位角度
constexpr uint16_t STORAGE_SERVO_SPEED = 400;
constexpr uint32_t STORAGE_SETTLE_MS = 200;
constexpr uint32_t STORAGE_RESET_SETTLE_MS = 500; // 功能区出口复位等待，禁止wait()长期阻塞

// 只需修改下面6个参数，即可分别调节1/2/3号盘的M5、M6放料坐标。
static constexpr const float (&STORAGE_PLACE_M5_ANGLES)[STORAGE_SLOT_COUNT] =
    ArmConfig::StorageReturn::M5_SLOT_ANGLES_TENTH_DEG;
static constexpr const float (&STORAGE_PLACE_M6_POSITIONS)[STORAGE_SLOT_COUNT] =
    ArmConfig::StorageReturn::M6_SLOT_POSITIONS_TENTH_MM;

constexpr float VERTICAL_HIGH_POSITION = 0.0f;
// StorageTest、原料区放盘和粗加工区回存使用同一个M7入盘高度。
constexpr float VERTICAL_PLACE_POSITION =
    ArmConfig::StorageReturn::M7_PLACE_POSITION_TENTH_MM;
constexpr float HORIZONTAL_RESET_POSITION = 0.0f;
constexpr float HORIZONTAL_MIN_POSITION =
    ArmConfig::M6_MIN_POSITION_TENTH_MM;
constexpr float HORIZONTAL_MAX_POSITION = 1500.0f;
// 到达储料盘放料位置后、松爪前的额外稳定时间；当前统一为100 ms。
constexpr uint32_t PLACE_ARRIVAL_SETTLE_MS =
    ArmConfig::StorageReturn::PLACE_ARRIVAL_SETTLE_MS;

constexpr float TEST_TIME_COEFFICIENT = 0.7f;
// -------------------- M7各动作独立速度/加速度 --------------------
// 速度越大运行越快；加速度越大启停越急。TTL加速度为uint8_t，最大值255。
constexpr uint16_t M7_STORAGE_PLACE_VELOCITY =
    ArmConfig::StorageReturn::M7_PLACE_VELOCITY;
constexpr uint8_t M7_STORAGE_PLACE_ACCELERATION =
    ArmConfig::StorageReturn::M7_PLACE_ACCELERATION;
constexpr uint16_t M7_STORAGE_PICK_VELOCITY = 20000; // [实机必调] 下降到储料盘夹取高度
constexpr uint16_t M7_STORAGE_PICK_ACCELERATION = 250;
constexpr uint16_t M7_LIFT_VELOCITY = 10000; // [实机必调] 抬升到高位及M7复位
constexpr uint16_t M7_LIFT_ACCELERATION = 200;
constexpr uint8_t VERTICAL_POSITIVE_DIRECTION = 0;
constexpr uint32_t VERTICAL_SETTLE_MS = 500;
constexpr uint16_t HORIZONTAL_VELOCITY = 2000;
constexpr uint8_t HORIZONTAL_ACCELERATION = 100;
constexpr uint8_t HORIZONTAL_POSITIVE_DIRECTION = 0;
constexpr uint32_t HORIZONTAL_SETTLE_MS = 500;

constexpr float BASE_MIN_ANGLE = -1800.0f;
constexpr float BASE_MAX_ANGLE = 3450.0f;
constexpr float BASE_MAX_SPEED = 120000.0f;
constexpr float BASE_ACCELERATION = 100000.0f;
constexpr float BASE_MOTOR_STEPS = 200.0f;
constexpr float BASE_MICROSTEPS = 16.0f;
constexpr float BASE_GEAR_RATIO = 5.0f;
constexpr float BASE_PULSES_PER_TENTH_DEGREE =
    BASE_MOTOR_STEPS * BASE_MICROSTEPS * BASE_GEAR_RATIO / 3600.0f;

constexpr uint8_t START_BUTTON_PIN = PB9;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
constexpr uint32_t POWER_STABILIZE_MS = 1000;
constexpr uint32_t SERIAL_BAUDRATE = 115200;

constexpr uint8_t TTL_RX_PIN = PA3;
constexpr uint8_t TTL_TX_PIN = PA2;
constexpr uint8_t SERVO_RX_PIN = PC7;
constexpr uint8_t SERVO_TX_PIN = PC6;
constexpr uint8_t VERTICAL_ID = 7;
constexpr uint8_t HORIZONTAL_ID = 6;
constexpr uint8_t GRIPPER_ID = 4;
constexpr uint8_t STORAGE_SERVO_ID = 5;

constexpr uint8_t CHASSIS_ENABLE_PIN = PE13;
constexpr uint8_t CHASSIS_DIR_PINS[4] = {PD6, PE9, PD14, PC3_C};
constexpr uint8_t CHASSIS_STEP_PINS[4] = {PD4, PE11, PD15, PA1};
constexpr uint8_t BASE_ENABLE_PIN = PE10;
constexpr uint8_t BASE_DIR_PIN = PE15;
constexpr uint8_t BASE_STEP_PIN = PB11;
} // namespace Config

namespace
{
constexpr float LEAD_SCREW_TENTH_MM_PER_REV = 120.0f;
constexpr float RACK_TENTH_MM_PER_REV =
    36.0f * static_cast<float>(M_PI) * 10.0f;
constexpr uint16_t MICROSTEPS = 16;
constexpr uint16_t MOTOR_STEPS_PER_REV = 200;

HardwareSerial ttlSerial(Config::TTL_RX_PIN, Config::TTL_TX_PIN);
HardwareSerial servoSerial(Config::SERVO_RX_PIN, Config::SERVO_TX_PIN);
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
uint8_t nextStorageSlot = 1;
bool rawButton = HIGH;
bool stableButton = HIGH;
uint32_t buttonChangedAt = 0;

uint32_t phaseDelayMs(uint32_t baseMs)
{
    return std::max<uint32_t>(
        1, lroundf(baseMs * Config::TEST_TIME_COEFFICIENT));
}

uint16_t phaseVelocity(uint16_t baseVelocity)
{
    return static_cast<uint16_t>(std::clamp(
        lroundf(baseVelocity / Config::TEST_TIME_COEFFICIENT), 1L, 60000L));
}

uint8_t phaseAcceleration(uint16_t baseAcceleration)
{
    const float scaled = baseAcceleration /
                         (Config::TEST_TIME_COEFFICIENT *
                          Config::TEST_TIME_COEFFICIENT);
    return static_cast<uint8_t>(std::clamp(lroundf(scaled), 1L, 255L));
}

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
        targetAngle = std::clamp(targetAngle,
                                 Config::BASE_MIN_ANGLE,
                                 Config::BASE_MAX_ANGLE);
        stepper.setMaxSpeed(std::min(
            Config::BASE_MAX_SPEED / Config::TEST_TIME_COEFFICIENT,
            60000.0f));
        stepper.setAcceleration(std::min(
            Config::BASE_ACCELERATION /
                (Config::TEST_TIME_COEFFICIENT * Config::TEST_TIME_COEFFICIENT),
            60000.0f));
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

void initLockedMotor(AccelStepper &motor)
{
    motor.setMinPulseWidth(3);
    motor.setCurrentPosition(0);
    motor.moveTo(0);
    motor.enableOutputs();
}

void lockChassis()
{
    pinMode(Config::CHASSIS_ENABLE_PIN, OUTPUT);
    digitalWrite(Config::CHASSIS_ENABLE_PIN, LOW);
    initLockedMotor(chassis1);
    initLockedMotor(chassis2);
    initLockedMotor(chassis3);
    initLockedMotor(chassis4);
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
    Serial.print("MOVE M6/ID6 target(0.1mm)/direction=");
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

void closeGripper()
{
    Serial.println("GRIPPER: close after PB9 press");
    gripper.setAngle(Config::GRIPPER_CLOSE_ANGLE,
                     phaseDelayMs(Config::GRIPPER_CLOSE_TIME_MS),
                     Config::GRIPPER_MAX_POWER_MW);
    delay(phaseDelayMs(Config::GRIPPER_TO_MOVE_DELAY_MS));
}

#ifdef PICK_STORAGE_TEST
void openPickGripper()
{
    Serial.print("GRIPPER: open to angle=");
    Serial.println(Config::PICK_GRIPPER_OPEN_ANGLE);
    gripper.setAngle(Config::PICK_GRIPPER_OPEN_ANGLE,
                     phaseDelayMs(Config::GRIPPER_CLOSE_TIME_MS),
                     Config::GRIPPER_MAX_POWER_MW);
    delay(phaseDelayMs(Config::GRIPPER_SETTLE_MS));
}
#endif

bool initializeHardware()
{
    delay(Config::POWER_STABILIZE_MS);
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
        Serial.println("INIT ERROR: gripper servo ID4 offline");
        return false;
    }
    storageServo.setSpeed(Config::STORAGE_SERVO_SPEED);
    gripper.setMaxPower(Config::GRIPPER_MAX_POWER_MW);
    storageServo.setRawAngle(Config::STORAGE_INITIAL_ANGLE);
    delay(Config::STORAGE_SETTLE_MS);
#ifdef PICK_STORAGE_TEST
#ifndef FINAL_TASK_EMBEDDED
    openPickGripper();
#endif
#else
    gripper.openMax();
    delay(phaseDelayMs(Config::GRIPPER_SETTLE_MS));
#endif
    return true;
}

void placeMaterialOnSlot(uint8_t slotNumber)
{
    Serial.print("=== STORAGE TEST SLOT ");
    Serial.print(slotNumber);
    Serial.println(" START ===");
    Serial.print("M5(0.1deg)/M6(0.1mm)=");
    Serial.print(storagePlaceM5Angle(slotNumber));
    Serial.print('/');
    Serial.println(storagePlaceM6Position(slotNumber));

    closeGripper();

    // 转动ID5储料盘和M5前，固定先执行：M7高位 -> M6完全收回。
    moveVertical(Config::VERTICAL_HIGH_POSITION,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);

    if (fabsf(horizontalCommandPosition - Config::HORIZONTAL_RESET_POSITION) > 0.5f)
    {
        moveHorizontal(Config::HORIZONTAL_RESET_POSITION);
    }
    storageServo.setRawAngle(storageSlotAngle(slotNumber));
    storageServo.wait();
    baseAxis.moveTo(storagePlaceM5Angle(slotNumber));
    if (fabsf(storagePlaceM6Position(slotNumber) -
              Config::HORIZONTAL_RESET_POSITION) > 0.5f)
    {
        moveHorizontal(storagePlaceM6Position(slotNumber));
    }
    moveVertical(Config::VERTICAL_PLACE_POSITION,
                 Config::M7_STORAGE_PLACE_VELOCITY,
                 Config::M7_STORAGE_PLACE_ACCELERATION);
    delay(Config::PLACE_ARRIVAL_SETTLE_MS);
    Serial.print("STORAGE RELEASE: open gripper to angle=");
    Serial.println(Config::STORAGE_RELEASE_ANGLE);
    gripper.setAngle(Config::STORAGE_RELEASE_ANGLE,
                     phaseDelayMs(Config::GRIPPER_CLOSE_TIME_MS),
                     Config::GRIPPER_MAX_POWER_MW);
    delay(phaseDelayMs(Config::GRIPPER_SETTLE_MS));

    // 每次放料后都复位机械臂；只有完成3号盘后才复位储料盘。
    moveVertical(Config::VERTICAL_HIGH_POSITION,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);
    moveVertical(Config::VERTICAL_HIGH_POSITION,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);
    if (fabsf(horizontalCommandPosition -
              Config::HORIZONTAL_RESET_POSITION) > 0.5f)
    {
        moveHorizontal(Config::HORIZONTAL_RESET_POSITION);
    }
    if (fabsf(baseAxis.position()) > 0.5f)
    {
        baseAxis.moveTo(0.0f);
    }
    Serial.println("ARM RESET COMPLETE: ID7 high, M6=0, M5=0");

    if (slotNumber == Config::STORAGE_SLOT_COUNT)
    {
        Serial.println("ALL 3 SLOTS COMPLETE: reset storage tray to -91 deg");
        storageServo.setRawAngle(Config::STORAGE_INITIAL_ANGLE);
        storageServo.wait();
        Serial.println("FULL RESET COMPLETE: next PB9 starts from slot 1");
    }
    else
    {
        Serial.print("=== SLOT ");
        Serial.print(slotNumber);
        Serial.println(" COMPLETE; keep tray position, reload material and press PB9 ===");
    }
}

#ifdef PICK_STORAGE_TEST
void pickMaterialFromSlot(uint8_t slotNumber)
{
    Serial.print("=== PICK STORAGE TEST SLOT ");
    Serial.print(slotNumber);
    Serial.println(" START ===");
    Serial.print("M5(0.1deg)/M6(0.1mm)=");
    Serial.print(storagePlaceM5Angle(slotNumber));
    Serial.print('/');
    Serial.println(storagePlaceM6Position(slotNumber));

    // 取料同样先确保M7高位、M6收回，再转动ID5储料盘和M5。
    moveVertical(Config::VERTICAL_HIGH_POSITION,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);
    if (fabsf(horizontalCommandPosition - Config::HORIZONTAL_RESET_POSITION) > 0.5f)
    {
        moveHorizontal(Config::HORIZONTAL_RESET_POSITION);
    }
    storageServo.setRawAngle(storageSlotAngle(slotNumber));
    storageServo.wait();
    baseAxis.moveTo(storagePlaceM5Angle(slotNumber));
    moveVertical(Config::PICK_VERTICAL_GRAB_POSITION,
                 Config::M7_STORAGE_PICK_VELOCITY,
                 Config::M7_STORAGE_PICK_ACCELERATION);
    moveHorizontal(storagePlaceM6Position(slotNumber));
    delay(Config::PICK_GRAB_SETTLE_MS);
    closeGripper();

    // 夹住物料后先抬高，再复位M6和M5，最后在复位位置松爪。
    moveVertical(Config::VERTICAL_HIGH_POSITION,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);
    moveVertical(Config::VERTICAL_HIGH_POSITION,
                 Config::M7_LIFT_VELOCITY,
                 Config::M7_LIFT_ACCELERATION);
    if (fabsf(horizontalCommandPosition -
              Config::HORIZONTAL_RESET_POSITION) > 0.5f)
    {
        moveHorizontal(Config::HORIZONTAL_RESET_POSITION);
    }
    if (fabsf(baseAxis.position()) > 0.5f)
    {
        baseAxis.moveTo(0.0f);
    }
    Serial.println("ARM RESET COMPLETE: open gripper");
    openPickGripper();

    Serial.print("=== SLOT ");
    Serial.print(slotNumber);
    Serial.println(" PICK COMPLETE; arm reset, tray position retained ===");
}
#endif
} // namespace

void setup()
{
    Serial.begin(Config::SERIAL_BAUDRATE);
    pinMode(Config::START_BUTTON_PIN, INPUT_PULLUP);
    rawButton = stableButton = digitalRead(Config::START_BUTTON_PIN);
    buttonChangedAt = millis();
    lockChassis();

    Serial.println();
#ifdef PICK_STORAGE_TEST
    Serial.println("=== PICK STORAGE COORDINATE TEST ===");
#else
    Serial.println("=== STORAGE COORDINATE TEST ===");
#endif
    if (!initializeHardware())
    {
        Serial.println("RESULT: hardware initialization failed");
        while (true)
        {
            serviceLockedChassis();
            delay(1);
        }
    }
#ifdef PICK_STORAGE_TEST
    Serial.println("READY: put materials in slots 1/2/3, then press PB9");
    Serial.println("Each press picks one slot in order: 1 -> 2 -> 3 -> 1");
#else
    Serial.println("READY: load one material into the open gripper, then press PB9");
    Serial.println("Each press tests one slot in order: 1 -> 2 -> 3 -> 1");
#endif
}

void loop()
{
    serviceLockedChassis();
    if (startPressed())
    {
#ifdef PICK_STORAGE_TEST
        pickMaterialFromSlot(nextStorageSlot);
#else
        placeMaterialOnSlot(nextStorageSlot);
#endif
        nextStorageSlot = nextStorageSlot % Config::STORAGE_SLOT_COUNT + 1;
    }
    delay(1);
}
