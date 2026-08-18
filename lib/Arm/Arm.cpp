#include "Arm.h"

//#define ARM_TWO
#ifndef ARM_TWO
// 载物盘舵机角度   0出发位置   1R  2G  3B  不干涉位置，储物盘三个盘位正对机械臂的角度
// [0]行驶归位，[1..3]三个托盘，[4]齿条避让位。
// 中间托盘基准88度，相邻托盘中心夹角81.27度：88-81.27=6.73，88+81.27=169.27。
// [实机必调] 若左右托盘顺序相反，仅交换storage[1]和storage[3]。
// [0]初始/复位，[1]1号盘，[2]2号盘，[3]3号盘，[4]齿条避让位。
// 初始位、避让位与1号盘均为-91度；后续盘位依次增加81度。
float storage[5] = {-91.0f, -91.0f, -10.0f, 71.0f, -91.0f};
// 手爪舵机参数设置
int GRIPPER_OPEN_ANGLE = -30;     // [实机已验证] 手爪张开角度
int GRIPPER_CLOSE_ANGLE = 45;     // [实机已验证] 手爪闭合角度
int GRIPPER_OPEN_MAX_ANGLE = -60; // [实机已验证] 主力爪A最大张开角度
#else
// 载物盘舵机角度   0出发位置   1R  2G  3B  不干涉位置，储物盘三个盘位正对机械臂的角度
float storage[5] = {70, -108, -20, 68, -113};
// 手爪舵机参数设置
int GRIPPER_OPEN_ANGLE = -35;     // 爪子张开时的角度
int GRIPPER_CLOSE_ANGLE = -72;    // 爪子闭合时的角度
int GRIPPER_OPEN_MAX_ANGLE = -20; // 爪子张开最大大角度
#endif
// 水平自由度步进电机参数设置
/*机械臂伸出：ID6上电时位于近行程，近端定义为0，向外伸出为正*/
float STEPPER_GRIPPER[4] = {0, 931.404236, 300, 904.333496}; // 近端零点，R,G,B
int STEPPER_GRIPPER_ZHUANPAN_CENTER = 800;
/*实际上机时，机械臂抓夹中心距离转轴的距离0.1mm*/
float Arm_Zero_Length = 1206.96826;
// 竖直自由度步进电机参数设置：ID7上电时位于高位，高位定义为0，向下为正
int STEPPER_ZERO = 0;       // 丝杆高位零点
int STEPPER_ZHUANPAN = 550; // 下降到转盘抓取，单位0.1毫米；统一减少20 mm
int STEPPER_GROUND = 1300;  // 下降到地面，单位0.1毫米；统一减少20 mm
int STEPPER_STORAGE = 240;  // 下降放到载物台，单位0.1毫米；统一减少20 mm
int STEPPER_UP = 200;       // 不干涉高点
int MATERIAL_HEIGHT = 700;  // 物料高度单位0.1毫米
// 基座步进电机参数设置
int ARM_BASE_Stepper_HOME_ANGLE = 315;                     // 机械臂发车初始角度
float ARM_BASE_STEPPER_ANGLE[4] = {2150, 771.188843, 1215, 1660.72083}; // 机械臂底部舵机角度{storage,R,G,B}
// 是否准备好抓取
bool Ready_to_Grab;
// 夹爪命令状态
bool gripper_state = false;
//             串口步进
HardwareSerial Serial_Stepper(Stepper_RX, Stepper_TX);
//             串口舵机          RX          TX
HardwareSerial Serial_SERVO(SERVO_RX, SERVO_TX);

// 创建舵机的通信协议对象
FSUS_Protocol protocol(&Serial_SERVO, SERVO_BAUDRATE); // 协议V2版本新增
FSUS_Servo storageServo(STORAGE_SERVO_ID, &protocol);  // 载物盘舵机
FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &protocol);  // 手爪舵机
// 创建智能机械爪实例
FSGP_Gripper gripper(&gripperServo, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE, GRIPPER_OPEN_MAX_ANGLE);

// 创建步进电机的通信协议对象
TTL_Protocol Stepper_protocol(&Serial_Stepper, STEPPER_BAUDRATE);
TTL_Stepper armStepper(ARM_Stepper_ID, &Stepper_protocol);         // 机械臂竖直自由度步进，12导程，16细分步数
TTL_Stepper gripperStepper(Gripper_Stepper_ID, &Stepper_protocol); // 机械臂水平自由度电机
PulseBaseStepper armBaseStepper(ARM_BASE_STEP_PIN, ARM_BASE_DIR_PIN, ARM_BASE_EN_PIN);

// 摄像头cam.py使用 CAMERA_HEIGHT_MM / HEIGHT = 130 / 240 mm/px。
// Follow_by_Arm的输入单位是0.1 mm，因此每像素乘以1300/240。
constexpr float VISION_TENTH_MM_PER_PIXEL = 1300.0f / 240.0f; // [实机必调] 摄像头高度变化后需重新标定
constexpr int VISION_COARSE_TOLERANCE_PX = 8;                 // [实机必调] 允许开始下降的像素误差
constexpr int VISION_FINE_TOLERANCE_PX = 3;                   // [实机必调] 夹取前最终像素误差
constexpr int VISION_STABLE_FRAME_COUNT = 3;                  // 连续稳定帧数，防止单帧误识别
constexpr uint32_t VISION_FRAME_TIMEOUT_MS = 5000;            // [实机必调] 单个物料等待超时
constexpr int VISION_MAX_ALIGN_STEPS = 15;                    // 防止视觉对准无限循环
constexpr bool ENABLE_VERTICAL_HOME_ON_START = false;         // [临时排障] ID7丝杆限位/零点未标定，禁止启动归零

// ================= 底座、ID6、ID7手动标定区 =================
// 下列参数只供实机标定时修改。Arm_Base_Vertical_Init()中的测试动作默认已用#if 0关闭，
// 因此正常一键启动只初始化驱动器，不会在这里移动机械臂，也不会等待到位信号。
constexpr float VERTICAL_TEST_START_HEIGHT = 0.0f;             // ID7高位零点，单位0.1mm
constexpr float VERTICAL_TEST_TRAVEL = 30.0f;                  // [ID7实机必调] 从高位向下测试，30=3mm
constexpr uint16_t VERTICAL_TEST_VELOCITY = 100;               // [ID7实机必调] 测试速度
constexpr uint8_t VERTICAL_TEST_ACCELERATION = 5;              // [ID7实机必调] 加速度；降低可使升降启停更平滑
constexpr uint8_t VERTICAL_MOTOR_DIRECTION = 0;                // [ID7实机必调] 必须使正坐标对应向下，错误则改0/1
constexpr float HORIZONTAL_TEST_START_LENGTH = 0.0f;           // ID6近端零点，单位0.1mm
constexpr float HORIZONTAL_TEST_TRAVEL = 30.0f;                // [ID6实机必调] 从近端向外测试，30=3mm
constexpr uint16_t HORIZONTAL_TEST_VELOCITY = 2000;             // [ID6实机必调] 运行速度，RPM；原100
constexpr uint8_t HORIZONTAL_TEST_ACCELERATION = 30;           // [ID6实机必调] 加速度；降低可使伸缩启停更平滑
constexpr uint8_t HORIZONTAL_MOTOR_DIRECTION = 0;              // [ID6实机已调] 0=正坐标向外伸长；若更换接线后反向则改为1
constexpr float BASE_MAX_SPEED_PPS = 8000.0f;                  // [M5实机必调] 底座最高脉冲速度；由2000提高50%
constexpr float BASE_ACCELERATION_PPS2 = 2000.0f;              // [M5实机必调] 底座脉冲加速度；降低可减小旋转冲击
constexpr float BASE_MIN_ANGLE = -3450.0f;                     // [M5实机必调] 相对开机零点最小角，单位0.1度
constexpr float BASE_MAX_ANGLE = 3450.0f;                      // [M5实机必调] 相对开机零点最大角，单位0.1度
constexpr float BASE_TEST_HOME_ANGLE = 315.0f;                 // [底座实机必调] 复位角度，315=31.5度
constexpr float BASE_TEST_DELTA_ANGLE = 50.0f;                 // [底座实机必调] 测试偏转，50=5度
constexpr float STARTUP_BASE_TEST_ANGLE = 50.0f;               // [实机必调] 启动自检底座偏转，50=5度
constexpr float STARTUP_STORAGE_HOME_ANGLE = -91.0f;           // [实机已调] 储料盘初始/1号盘角度
constexpr float STARTUP_STORAGE_TEST_DELTA = 10.0f;            // [实机必调] 储料盘自检偏转角度
constexpr uint16_t STARTUP_SERVO_MOVE_MS = 500;                // [实机必调] 手爪/储料盘单次动作时间
constexpr uint16_t STARTUP_SERVO_SETTLE_MS = 650;              // [实机必调] 舵机动作后的等待时间
constexpr uint16_t STARTUP_VERTICAL_SETTLE_MS = 800;           // [实机必调] ID7单程3mm运动等待时间
constexpr uint16_t STARTUP_HORIZONTAL_SETTLE_MS = 800;         // [实机必调] ID6单程3mm运动等待时间
// =====================================================

static float horizontalAxisPosition = 0.0f; // ID6软件绝对位置，单位0.1mm
static float verticalAxisPosition = 0.0f;   // ID7软件绝对位置，单位0.1mm；高位为0，向下为正

void Arm_Move_Horizontal_To(float targetLength)
{
    targetLength = max(0.0f, targetLength);
    const float deltaLength = targetLength - horizontalAxisPosition;
    const uint32_t pulses = lroundf(
        fabsf(deltaLength) * 16.0f * 200.0f / CHILUN);
    if (pulses == 0)
    {
        horizontalAxisPosition = targetLength;
        return;
    }

    const uint8_t direction = deltaLength > 0.0f
                                  ? HORIZONTAL_MOTOR_DIRECTION
                                  : !HORIZONTAL_MOTOR_DIRECTION;
    Serial.print("ID6 direct move: from/to/dir/pulses=");
    Serial.print(horizontalAxisPosition);
    Serial.print('/');
    Serial.print(targetLength);
    Serial.print('/');
    Serial.print(direction);
    Serial.print('/');
    Serial.println(pulses);

    // 与已验证的启动自检保持一致：单次相对命令，不重发、不做到位查询。
    Stepper_protocol.Emm_V5_Pos_Control(Gripper_Stepper_ID,
                                        direction,
                                        HORIZONTAL_TEST_VELOCITY,
                                        HORIZONTAL_TEST_ACCELERATION,
                                        pulses,
                                        false,
                                        false);

    const float revolutions = pulses / (16.0f * 200.0f);
    const uint32_t estimatedMoveMs = lroundf(
        revolutions * 60000.0f / HORIZONTAL_TEST_VELOCITY);
    delay(estimatedMoveMs + 500); // [ID6实机必调] 额外500ms供加减速和串口响应
    while (Serial_Stepper.available())
    {
        Serial_Stepper.read();
    }
    horizontalAxisPosition = targetLength;
}

void Arm_Move_Vertical_To(float targetHeight)
{
    targetHeight = max(0.0f, targetHeight);
    const float deltaHeight = targetHeight - verticalAxisPosition;
    const uint32_t pulses = lroundf(
        fabsf(deltaHeight) * 16.0f * 200.0f / LUOGAN);
    if (pulses == 0)
    {
        verticalAxisPosition = targetHeight;
        return;
    }

    const uint8_t direction = deltaHeight > 0.0f
                                  ? VERTICAL_MOTOR_DIRECTION
                                  : !VERTICAL_MOTOR_DIRECTION;
    Serial.print("ID7 direct move: from/to/dir/pulses=");
    Serial.print(verticalAxisPosition);
    Serial.print('/');
    Serial.print(targetHeight);
    Serial.print('/');
    Serial.print(direction);
    Serial.print('/');
    Serial.println(pulses);

    Stepper_protocol.Emm_V5_Pos_Control(ARM_Stepper_ID,
                                        direction,
                                        VERTICAL_TEST_VELOCITY,
                                        VERTICAL_TEST_ACCELERATION,
                                        pulses,
                                        false,
                                        false);
    const float revolutions = pulses / (16.0f * 200.0f);
    const uint32_t estimatedMoveMs = lroundf(
        revolutions * 60000.0f / VERTICAL_TEST_VELOCITY);
    delay(estimatedMoveMs + 500); // [ID7实机必调] 加减速和串口响应余量
    while (Serial_Stepper.available())
    {
        Serial_Stepper.read();
    }
    verticalAxisPosition = targetHeight;
}

static void discardVisionFrames()
{
    while (Serial_Maix.available())
    {
        Serial_Maix.read();
    }
    vision_updated = false;
}

static bool waitExpectedMaterialFrame(int expectedColor, int &dx, int &dy)
{
    const uint32_t started = millis();
    while (millis() - started < VISION_FRAME_TIMEOUT_MS)
    {
        while (Serial_Maix.available())
        {
            MaixCam.Maix_ReadData(Serial_Maix.read());
            if (vision_updated)
            {
                vision_updated = false;
                if (MaixCam.Color == expectedColor)
                {
                    dx = MaixCam.Delta_X;
                    dy = MaixCam.Delta_Y;
                    return true;
                }
            }
        }
        delay(1);
    }
    return false;
}

static bool alignToMaterial(int expectedColor, int tolerancePixels)
{
    ARM_PID.PID_Init(ARM_PID_Kp, ARM_PID_Ki, ARM_PID_Kd,
                     ARM_PID_MItg, ARM_PID_MOut);
    ARM_BASE_PID.PID_Init(ARM_BASE_PID_Kp, ARM_BASE_PID_Ki,
                          ARM_BASE_PID_Kd, ARM_BASE_PID_MItg,
                          ARM_BASE_PID_MOut);

    int stableFrames = 0;
    for (int step = 0; step < VISION_MAX_ALIGN_STEPS; ++step)
    {
        int dx = 0;
        int dy = 0;
        if (!waitExpectedMaterialFrame(expectedColor, dx, dy))
        {
            return false;
        }

        Serial.print("Vision color/dx/dy: ");
        Serial.print(expectedColor);
        Serial.print('/');
        Serial.print(dx);
        Serial.print('/');
        Serial.println(dy);

        if (abs(dx) <= tolerancePixels && abs(dy) <= tolerancePixels)
        {
            if (++stableFrames >= VISION_STABLE_FRAME_COUNT)
            {
                return true;
            }
            continue;
        }

        stableFrames = 0;
        Follow_by_Arm(dx * VISION_TENTH_MM_PER_PIXEL,
                      dy * VISION_TENTH_MM_PER_PIXEL,
                      Gripper_STEPPER_Vel,
                      ARM_BASE_STEPPER_Vel,
                      Gripper_STEPPER_Acc,
                      ARM_BASE_STEPPER_Acc);
        discardVisionFrames(); // 丢弃机械臂移动期间积压的旧帧
    }
    return false;
}

PulseBaseStepper::PulseBaseStepper(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin)
    : stepper(AccelStepper::DRIVER, stepPin, dirPin), enablePin(enablePin)
{
}

void PulseBaseStepper::init()
{
    stepper.setEnablePin(enablePin);
    stepper.setPinsInverted(false, false, true); // EN 低电平有效
    stepper.setMinPulseWidth(3);
    stepper.setMaxSpeed(BASE_MAX_SPEED_PPS);
    stepper.setAcceleration(BASE_ACCELERATION_PPS2);
    stepper.setCurrentPosition(0);
    stepper.enableOutputs();
    currentPosition = 0.0f;
    onPos_state = true;
}

void PulseBaseStepper::moveToAngle(float angle)
{
    angle = std::clamp(angle, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
    const long targetPulses = lroundf(angle * ARM_BASE_PULSES_PER_TENTH_DEGREE);
    onPos_state = false;
    stepper.moveTo(targetPulses);
    stepper.runToPosition();
    update_CurrentPos();
    onPos_state = true;
}

void PulseBaseStepper::setAngle(float angle)
{
    moveToAngle(angle);
}

void PulseBaseStepper::setAngle(float angle, uint16_t, uint8_t)
{
    moveToAngle(angle);
}

void PulseBaseStepper::runToNewPosition(float angle)
{
    moveToAngle(angle);
}

void PulseBaseStepper::wait()
{
    stepper.runToPosition();
    update_CurrentPos();
    onPos_state = true;
}

void PulseBaseStepper::update_CurrentPos()
{
    currentPosition = stepper.currentPosition() / ARM_BASE_PULSES_PER_TENTH_DEGREE;
}

void PulseBaseStepper::update_state()
{
    stepper.run();
    update_CurrentPos();
    onPos_state = (stepper.distanceToGo() == 0);
}

/// @brief 收起机械臂、载物台
void Arm_Hardware_Init(bool initializeCamera)
{
    Serial.println("ARM INIT: begin");
    if (initializeCamera)
    {
        Serial.println("ARM INIT: camera");
        MaixCam.Maix_Init();
        delay(100);
    }
    else
    {
        Serial.println("ARM INIT: camera skipped");
    }

    Serial.println("ARM INIT: TTL steppers");
    Stepper_protocol.init(&Serial_Stepper, STEPPER_BAUDRATE);
    armStepper.init();
    gripperStepper.init();
    armStepper.set(2500, 255, 0, LUOGAN, 16);       // [实机必调] 丝杆轴速度/加速度/方向
    gripperStepper.set(HORIZONTAL_TEST_VELOCITY,
                       HORIZONTAL_TEST_ACCELERATION,
                       HORIZONTAL_MOTOR_DIRECTION,
                       CHILUN,
                       16);
    armBaseStepper.init();
    delay(100);

    Serial.println("ARM INIT: bus servos");
    protocol.init(&Serial_SERVO, SERVO_BAUDRATE);
    storageServo.init();
    gripper.init();
    gripper.setMaxPower(400);                       // 成功示例使用的夹爪最大功率，mW
    storageServo.setSpeed(500);                       // 成功示例的载物盘舵机速度

    ARM_PID.PID_Init(ARM_PID_Kp, ARM_PID_Ki, ARM_PID_Kd,
                     ARM_PID_MItg, ARM_PID_MOut);
    ARM_BASE_PID.PID_Init(ARM_BASE_PID_Kp, ARM_BASE_PID_Ki,
                          ARM_BASE_PID_Kd, ARM_BASE_PID_MItg,
                          ARM_BASE_PID_MOut);
    delay(100);
    Arm_Init();
    Serial.println("ARM INIT: complete");
}

void Arm_Base_Only_Init()
{
    Serial.println("ARM BASE ONLY: begin");
    armBaseStepper.init();
    delay(100);
    Serial.print("ARM BASE ONLY: target(tenth-degree)=");
    Serial.println(ARM_BASE_Stepper_HOME_ANGLE);
    armBaseStepper.runToNewPosition(ARM_BASE_Stepper_HOME_ANGLE);
    armBaseStepper.wait();
    Serial.println("ARM BASE ONLY: complete");
}

void Arm_Startup_Self_Test()
{
    Serial.println("ARM STARTUP SELF TEST: begin");

    // 本轮初始化5号底座STEP/DIR步进、TTL ID6悬臂、TTL ID7丝杆、
    // 手爪舵机ID4和储料盘舵机ID5；不初始化相机。
    armBaseStepper.init();
    Stepper_protocol.init(&Serial_Stepper, STEPPER_BAUDRATE);
    armStepper.init();
    armStepper.set(VERTICAL_TEST_VELOCITY,
                   VERTICAL_TEST_ACCELERATION,
                   VERTICAL_MOTOR_DIRECTION,
                   LUOGAN,
                   16);
    gripperStepper.init();
    gripperStepper.set(HORIZONTAL_TEST_VELOCITY,
                       HORIZONTAL_TEST_ACCELERATION,
                       HORIZONTAL_MOTOR_DIRECTION,
                       CHILUN,
                       16);
    protocol.init(&Serial_SERVO, SERVO_BAUDRATE);
    storageServo.init();
    gripper.init();
    gripper.setMaxPower(400);
    storageServo.setSpeed(500);
    delay(100);

    // 5号底座步进：以启动位置为0，旋转一个小角度后回到启动位置。
    Serial.println("ARM SELF TEST: base move");
    armBaseStepper.runToNewPosition(STARTUP_BASE_TEST_ANGLE);
    delay(300);
    Serial.println("ARM SELF TEST: base return");
    armBaseStepper.runToNewPosition(0.0f);
    delay(300);

    // ID7丝杆：初始高位，向下移动3mm后反向返回高位。
    // 直接发送相对位移命令，不调用TTL库中可能无限等待应答的runToNewPosition()/wait()。
    const uint32_t verticalTestPulses = lroundf(
        VERTICAL_TEST_TRAVEL * 16.0f * 200.0f / LUOGAN);
    while (Serial_Stepper.available())
    {
        Serial_Stepper.read();
    }

    Serial.println("ARM SELF TEST: ID7 move down");
    Stepper_protocol.Emm_V5_Pos_Control(ARM_Stepper_ID,
                                        VERTICAL_MOTOR_DIRECTION,
                                        VERTICAL_TEST_VELOCITY,
                                        VERTICAL_TEST_ACCELERATION,
                                        verticalTestPulses,
                                        false,
                                        false);
    delay(STARTUP_VERTICAL_SETTLE_MS);
    while (Serial_Stepper.available())
    {
        Serial_Stepper.read();
    }
    Serial.println("ARM SELF TEST: ID7 return high");
    Stepper_protocol.Emm_V5_Pos_Control(ARM_Stepper_ID,
                                        !VERTICAL_MOTOR_DIRECTION,
                                        VERTICAL_TEST_VELOCITY,
                                        VERTICAL_TEST_ACCELERATION,
                                        verticalTestPulses,
                                        false,
                                        false);
    delay(STARTUP_VERTICAL_SETTLE_MS);
    while (Serial_Stepper.available())
    {
        Serial_Stepper.read();
    }

    // ID6悬臂：初始近端，向外伸出3mm后反向收回近端。
    // 同样直接发送相对位移命令，不调用TTL库的runToNewPosition()/wait()。
    const uint32_t horizontalTestPulses = lroundf(
        HORIZONTAL_TEST_TRAVEL * 16.0f * 200.0f / CHILUN);
    Serial.println("ARM SELF TEST: ID6 extend");
    Stepper_protocol.Emm_V5_Pos_Control(Gripper_Stepper_ID,
                                        HORIZONTAL_MOTOR_DIRECTION,
                                        HORIZONTAL_TEST_VELOCITY,
                                        HORIZONTAL_TEST_ACCELERATION,
                                        horizontalTestPulses,
                                        false,
                                        false);
    delay(STARTUP_HORIZONTAL_SETTLE_MS);
    while (Serial_Stepper.available())
    {
        Serial_Stepper.read();
    }
    Serial.println("ARM SELF TEST: ID6 retract");
    Stepper_protocol.Emm_V5_Pos_Control(Gripper_Stepper_ID,
                                        !HORIZONTAL_MOTOR_DIRECTION,
                                        HORIZONTAL_TEST_VELOCITY,
                                        HORIZONTAL_TEST_ACCELERATION,
                                        horizontalTestPulses,
                                        false,
                                        false);
    delay(STARTUP_HORIZONTAL_SETTLE_MS);
    while (Serial_Stepper.available())
    {
        Serial_Stepper.read();
    }

    // 手爪：张开后闭合。使用固定等待，避免到位查询影响一键启动状态机。
    Serial.println("ARM SELF TEST: gripper open");
    gripper.setAngle(GRIPPER_OPEN_ANGLE, STARTUP_SERVO_MOVE_MS, 400);
    delay(STARTUP_SERVO_SETTLE_MS);
    Serial.println("ARM SELF TEST: gripper close");
    gripper.setAngle(GRIPPER_CLOSE_ANGLE, STARTUP_SERVO_MOVE_MS, 400);
    delay(STARTUP_SERVO_SETTLE_MS);

    // 储料盘：从复位角度偏转一个小角度，然后回到复位角度。
    Serial.println("ARM SELF TEST: storage move");
    storageServo.setRawAngle(STARTUP_STORAGE_HOME_ANGLE +
                                 STARTUP_STORAGE_TEST_DELTA,
                             STARTUP_SERVO_MOVE_MS);
    delay(STARTUP_SERVO_SETTLE_MS);
    Serial.println("ARM SELF TEST: storage return");
    storageServo.setRawAngle(STARTUP_STORAGE_HOME_ANGLE,
                             STARTUP_SERVO_MOVE_MS);
    delay(STARTUP_SERVO_SETTLE_MS);

    Serial.println("ARM STARTUP SELF TEST: complete");
}

void Arm_Base_Horizontal_Init()
{
    Serial.println("ARM BASE+HORIZONTAL: begin");

    // 只初始化ID6悬臂齿轮齿条轴，不初始化ID7丝杆升降轴。
    Stepper_protocol.init(&Serial_Stepper, STEPPER_BAUDRATE);
    gripperStepper.init();
    gripperStepper.set(HORIZONTAL_TEST_VELOCITY,
                       HORIZONTAL_TEST_ACCELERATION,
                       HORIZONTAL_MOTOR_DIRECTION,
                       CHILUN,
                       16);

    Serial.println("ARM BASE+HORIZONTAL: horizontal retract to 0");
    gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);
    gripperStepper.wait();
    Serial.println("ARM BASE+HORIZONTAL: horizontal complete");
    delay(100);

    armBaseStepper.init();
    Serial.print("ARM BASE+HORIZONTAL: base target(tenth-degree)=");
    Serial.println(ARM_BASE_Stepper_HOME_ANGLE);
    armBaseStepper.runToNewPosition(ARM_BASE_Stepper_HOME_ANGLE);
    armBaseStepper.wait();
    Serial.println("ARM BASE+HORIZONTAL: complete");
}

void Arm_Base_Vertical_Init()
{
    Serial.println("ARM BASE+ID6+ID7: begin");

    // ID6和ID7共用TTL串口。正常一键启动只初始化，不移动、不等待到位，
    // 避免限位器、零点或方向未标定时卡住状态机。
    Stepper_protocol.init(&Serial_Stepper, STEPPER_BAUDRATE);
    armStepper.init();
    armStepper.set(2500, 255, VERTICAL_MOTOR_DIRECTION, LUOGAN, 16);
    gripperStepper.init();
    gripperStepper.set(HORIZONTAL_TEST_VELOCITY,
                       HORIZONTAL_TEST_ACCELERATION,
                       HORIZONTAL_MOTOR_DIRECTION,
                       CHILUN,
                       16);

    // 初始化底座STEP/DIR脉冲电机（无总线ID），但不在这里移动或等待。
    armBaseStepper.init();

#if 0
    // ================= 手动标定动作：确认安全后将#if 0改成#if 1 =================
    // 当前实机初始状态：ID7在高位、ID6在近端，因此可将这两个位置写为驱动器零点。
    // 这两条命令只清零坐标，不会驱动电机运动；正常一键启动时不会执行。
    Stepper_protocol.Emm_V5_Reset_CurPos_To_Zero(ARM_Stepper_ID);
    delay(50);
    Stepper_protocol.Emm_V5_Reset_CurPos_To_Zero(Gripper_Stepper_ID);
    delay(50);

    // ID7：从标定起始高度移动一小段，再返回起始高度。
    // 若运动方向继续顶向限位器，请立即断电，并修改VERTICAL_MOTOR_DIRECTION。
    Serial.println("ARM BASE+VERTICAL: ID7 move test");
    armStepper.runToNewPosition(VERTICAL_TEST_START_HEIGHT + VERTICAL_TEST_TRAVEL,
                                VERTICAL_TEST_VELOCITY,
                                VERTICAL_TEST_ACCELERATION);
    armStepper.wait();
    delay(300);
    Serial.println("ARM BASE+VERTICAL: ID7 return start height");
    armStepper.runToNewPosition(VERTICAL_TEST_START_HEIGHT,
                                VERTICAL_TEST_VELOCITY,
                                VERTICAL_TEST_ACCELERATION);
    armStepper.wait();
    Serial.println("ARM BASE+VERTICAL: ID7 test complete");

    // ID6：从标定起始伸出量移动一小段，再返回起始位置。
    // 若运动方向继续顶向机械限位，请立即断电，并修改HORIZONTAL_MOTOR_DIRECTION。
    Serial.println("ARM BASE+ID6+ID7: ID6 move test");
    gripperStepper.runToNewPosition(HORIZONTAL_TEST_START_LENGTH +
                                        HORIZONTAL_TEST_TRAVEL,
                                    HORIZONTAL_TEST_VELOCITY,
                                    HORIZONTAL_TEST_ACCELERATION);
    gripperStepper.wait();
    delay(300);
    Serial.println("ARM BASE+ID6+ID7: ID6 return start length");
    gripperStepper.runToNewPosition(HORIZONTAL_TEST_START_LENGTH,
                                    HORIZONTAL_TEST_VELOCITY,
                                    HORIZONTAL_TEST_ACCELERATION);
    gripperStepper.wait();
    Serial.println("ARM BASE+ID6+ID7: ID6 test complete");

    // 底座STEP/DIR电机：先到标定复位角度，偏转一小段，再返回复位角度。
    Serial.print("ARM BASE+VERTICAL: base target(tenth-degree)=");
    Serial.println(BASE_TEST_HOME_ANGLE);
    armBaseStepper.runToNewPosition(BASE_TEST_HOME_ANGLE);
    armBaseStepper.wait();
    delay(300);
    Serial.println("ARM BASE+VERTICAL: base move test");
    armBaseStepper.runToNewPosition(BASE_TEST_HOME_ANGLE +
                                    BASE_TEST_DELTA_ANGLE);
    armBaseStepper.wait();
    delay(300);
    Serial.println("ARM BASE+VERTICAL: base return home");
    armBaseStepper.runToNewPosition(BASE_TEST_HOME_ANGLE);
    armBaseStepper.wait();
#endif

    Serial.println("ARM BASE+ID6+ID7: initialized, calibration motion skipped");
}

void Storage_SetSlot(int slot)
{
    slot = constrain(slot, 0, 4);
    storageServo.setRawAngle(storage[slot]);
}

/// @brief 用MaixCam识别指定颜色，对准后从原料转盘抓取并放入当前载物盘位。
/// @return true表示抓取流程完成；false表示识别或对准超时。
bool Grab_Recognized_Material_To_Storage(int expectedColor,
                                         bool rotateToPickupHeading,
                                         float pickupHeight)
{
    if (expectedColor < Red || expectedColor > LightBlue)
    {
        return false;
    }

    if (pickupHeight < 0.0f)
    {
        pickupHeight = STEPPER_ZHUANPAN;
    }

    Serial.print("Start vision pickup, color=");
    Serial.println(expectedColor);
    Storage_SetSlot(taskStorageSlot(dataIndex));
    Arm_Move_Vertical_To(STEPPER_ZERO);
    if (rotateToPickupHeading)
    {
        armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[PLACEMENT_2]); // [实机必调] 摄像头朝向原料转盘的初始角度
        armBaseStepper.wait();
    }
    gripper.openMax();
    Arm_Move_Horizontal_To(STEPPER_GRIPPER_ZHUANPAN_CENTER); // [实机必调] 转盘中心初始伸出量
    storageServo.wait();

    discardVisionFrames();
    MaixCam.Maix_Follow(expectedColor);
    if (!alignToMaterial(expectedColor, VISION_COARSE_TOLERANCE_PX))
    {
        goto pickup_failed;
    }

    // 粗对准后降至抓取高度，再用较小容差做一次精对准。
    Arm_Move_Vertical_To(pickupHeight); // ArmTest由ARM_TEST_PICKUP_HEIGHT单独设置
    discardVisionFrames();
    if (!alignToMaterial(expectedColor, VISION_FINE_TOLERANCE_PX))
    {
        goto pickup_failed;
    }

    gripper.close();
    gripper.wait();
    delay(100);
    Arm_Move_Vertical_To(STEPPER_ZERO);
    Grab_ZhuanPan_to_Storage(expectedColor);
    Serial.println("Vision pickup complete");
    return true;

pickup_failed:
    Serial.println("Vision pickup failed");
    gripper.openMax();
    Arm_Move_Vertical_To(STEPPER_ZERO);
    Arm_Move_Horizontal_To(STEPPER_GRIPPER[0]);
    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);
    armBaseStepper.wait();
    return false;
}

void Arm_Init()
{
    if (ENABLE_VERTICAL_HOME_ON_START)
    {
        Serial.println("ARM HOME: vertical axis");
        armStepper.runToNewPosition(STEPPER_ZERO);
        armStepper.wait();
        delay(100);
    }
    else
    {
        // 待确认限位开关逻辑、驱动器回零方向和实际零点后，
        // 将ENABLE_VERTICAL_HOME_ON_START改为true恢复下面的归零命令。
        // armStepper.runToNewPosition(STEPPER_ZERO);
        // armStepper.wait();
        Serial.println("ARM HOME: vertical axis skipped");
    }

    Serial.println("ARM HOME: horizontal axis");
    gripperStepper.runToNewPosition(0);
    gripperStepper.wait();
    delay(100);

    // 大电流执行器严格串行归位，避免手爪、载物盘和基座同时起动。
    Serial.println("ARM HOME: gripper open");
    gripper.openMax();                                             // 初始保持最大张开，识别并到达抓取高度后才闭合
    gripper.wait();
    delay(150);
    Serial.println("ARM HOME: storage tray");
    Storage_SetSlot(0);                                            // 载物盘归位
    storageServo.wait();
    delay(150);
    Serial.println("ARM HOME: base axis");
    armBaseStepper.runToNewPosition(ARM_BASE_Stepper_HOME_ANGLE); // 机械臂归位
    armBaseStepper.wait();
    Serial.println("ARM HOME: complete");
};

/// @brief 从转盘抓取color物料，并放置在载物台上，最终机械臂朝载物台
/// @param color
void Grab_ZhuanPan_to_Storage(int color)
{
    // armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]); // 机械臂朝外
    (void)color;
    Storage_SetSlot(taskStorageSlot(dataIndex));                 // 按抓取顺序使用三个车载盘位
    Arm_Move_Vertical_To(STEPPER_ZERO);            // 机械臂上升至最高位置
    Arm_Move_Horizontal_To(STEPPER_GRIPPER[0]);    // 机械臂收回减少转动惯量

    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]); // 机械臂朝向载物盘
    armBaseStepper.wait();                              // 等待机械臂旋转到位
    storageServo.wait();                                // 等待载物台旋转到位
    Arm_Move_Vertical_To(STEPPER_STORAGE);              // 机械臂下降到载物台
    gripper.open();                                     // 夹爪打开，内置到位等待
    Arm_Move_Vertical_To(STEPPER_ZERO);                 // 机械臂上升到最高位置
}

/// @brief 从载物台抓取color物料，并放置在地/堆垛物料上表面，最终机械臂朝向载物台
/// @param n ,选择是否是地面还是物料上表面
void Grab_Storage_to_Place(int stackLevel, bool temporaryArea)
{
    const int storageSlot = taskStorageSlot(dataIndex);
    const int targetPosition = temporaryArea
                                   ? taskTemporaryPosition(dataIndex)
                                   : taskCoarsePosition(dataIndex);
    Storage_SetSlot(storageSlot);                // 载物盘旋转
    gripper.open();                                        // 爪子打开
    armBaseStepper.wait();                                 // 机械臂旋转到位
    storageServo.wait();                                   // 载物台旋转到位
    armStepper.runToNewPosition(STEPPER_STORAGE);          // 机械臂下降到载物台
    armStepper.wait();                                     // 等待下降到位
    gripper.close();                                       // 夹爪关闭
    gripper.wait();                                        // 夹爪夹紧到位
    armStepper.runToNewPosition(STEPPER_ZERO);             // 机械臂上升
    armStepper.wait();                                     // 上升到位
    if (storageSlot == 2)
    {
        Storage_SetSlot(4);                // 载物台旋转到齿条避让位
    }
    else
    {
        if (dataIndex != (2 + rounds * 3))
        {
            Storage_SetSlot(taskStorageSlot(dataIndex + 1));
        }
        else
        {
            Storage_SetSlot(taskStorageSlot(rounds * 3));
        }
    }
    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[targetPosition]); // 旋转到任务指定圆环
    if (storageSlot == 2)
    {
        storageServo.wait(); // 载物台等待*****************************************
    }
    armBaseStepper.wait();                                                   // 机械臂旋转到位
    gripperStepper.runToNewPosition(STEPPER_GRIPPER[targetPosition]); // 夹爪伸出到任务指定圆环
    gripperStepper.wait();                                                    // 夹爪伸出到位
    armStepper.runToNewPosition(STEPPER_GROUND - stackLevel * MATERIAL_HEIGHT); // 机械臂下降到平面/堆垛高度
    armStepper.wait();                                                       // 机械臂下降到位
    gripper.open();                                                          // 夹爪打开，内置到位等待

    if (dataIndex == (2 + rounds * 3))
    {
        armStepper.runToNewPosition(STEPPER_ZERO);
        gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);
        gripperStepper.wait();
        armStepper.wait();
        armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);
        Storage_SetSlot(0);
    }
    else
    {
        armStepper.runToNewPosition(STEPPER_ZERO);           // 机械臂上升
        gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]); // 夹爪收回
        armStepper.wait();                                   // 机械臂上升到位
        gripperStepper.wait();                               // 夹爪收回到位
        armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);  // 机械臂朝向载物台
        armBaseStepper.wait();                               // 机械臂旋转到位
    }
}

/// @brief 从地上抓取color物料放置到载物盘
void Grab_Ground_to_Storage()
{
    const int storageSlot = taskStorageSlot(dataIndex);
    const int sourcePosition = taskCoarsePosition(dataIndex);
    if (storageSlot == 2)
    {
        Storage_SetSlot(4);                // 载物台旋转到齿条避让位
    }
    else
    {
        Storage_SetSlot(storageSlot);
    }
    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[sourcePosition]); // 对准粗加工区任务圆环
    armBaseStepper.wait();                                                  // 等待机械臂旋转到位
    gripper.Unfold();                                                      // 机械爪打开
    if (storageSlot == 2)
    {
        storageServo.wait(); // 等待载物盘旋转到位*******************
    }
    gripperStepper.runToNewPosition(STEPPER_GRIPPER[sourcePosition]); // 夹爪伸出到对应圆环
    armStepper.runToNewPosition(STEPPER_GROUND);                             // 机械臂下降到地面
    gripper.wait();                                                          // 张开到位
    gripperStepper.wait();                                                   // 等待夹爪伸出到位
    armStepper.wait();                                                       // 等待机械臂下降到位
    gripper.close();                                                         // 机械抓夹紧
    gripper.wait();                                                          // 夹爪到位
    armStepper.runToNewPosition(STEPPER_ZERO);                               // 机械臂上升至最高位置
    armStepper.wait();                                                       // 等待机械臂上升到位
    gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);                     // 夹爪收回
    gripperStepper.wait();                                                   // 夹爪收回到位

    if (storageSlot == 2)
    {
        Storage_SetSlot(storageSlot);
    }
    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]); // 机械臂朝向载物台
    armBaseStepper.wait();                              // 机械臂旋转到位
    storageServo.wait();                                // 载物台旋转到位
    armStepper.runToNewPosition(STEPPER_STORAGE);       // 机械臂下降
    armStepper.wait();                                  // 机械臂下降到位
    gripper.open();                                     // 夹爪打开，自带到位
    armStepper.runToNewPosition(STEPPER_ZERO);          // 机械臂上升到最高点
    armStepper.wait();                                  // 等待上升到位
}

// 转盘处伸出长度
/**
 * @brief 计算夹爪步进电机的目标位置，控制电机移动到该位置，并返回机械臂进入死区时的未补偿量
 * @param delta 目标位置的增量
 * @return float 机械臂进入死区时的未补偿量
 */
float gripper_reachout(float delta)
{
    gripperStepper.update_CurrentPos();                                   // 更新夹爪步进电机的当前位置
    float absolute = delta + gripperStepper.currentPosition;              // 计算绝对目标位置
    float x = (absolute > 1500) ? 1500 : ((absolute < 0) ? 0 : absolute); // 限位处理
    gripperStepper.runToNewPosition(x);                                   // 控制电机移动到限位后的目标位置
    // 计算并返回机械臂进入死区时的未补偿量
    float Actual_delta = (x - gripperStepper.currentPosition) / 10.0;
    return Actual_delta;
}

// 纯手部校准
/**
 * @brief 纯手部校准函数，根据视觉检测的偏移量调整夹爪步进电机和机械臂基座步进电机的位置
 * @param deltaX 视觉检测到的 X 轴偏移量 0.1mm单位
 * @param deltaY 视觉检测到的 Y 轴偏移量 0.1mm单位
 * @param armVel 机械臂速度 rpm
 * @param rotVel 机械臂旋转速度 rpm
 * @param armAcc 机械臂加速度
 * @param rotAccel 机械臂旋转加速度
 */
void Follow_by_Arm(float deltaX, float deltaY, uint16_t gripperVel, uint16_t rotVel, uint8_t gripperAcc, uint8_t rotAccel)
{
    (void)gripperVel;
    (void)rotVel;
    (void)gripperAcc;
    (void)rotAccel;

    // PID控制
    ARM_PID.PID_Calc(deltaY, 0);
    float gripperStepperTarget = horizontalAxisPosition - ARM_PID.output;
    gripperStepperTarget = std::clamp(gripperStepperTarget, 0.0f, 1500.0f);
    Arm_Move_Horizontal_To(gripperStepperTarget);

    // 计算角度偏移量，使用 atan2 函数替代近似计算，提高精度
    float deltaTheta = std::atan2(
                           deltaX,
                           horizontalAxisPosition - deltaY + Arm_Zero_Length) *
                       (1800.0 / M_PI);
    // PID控制
    ARM_BASE_PID.PID_Calc(deltaTheta, 0);
    armBaseStepper.update_CurrentPos();
    float armBaseStepperTarget = armBaseStepper.currentPosition + ARM_BASE_PID.output;
    armBaseStepperTarget = std::clamp(armBaseStepperTarget, -3450.0f, 3450.0f);
    armBaseStepper.setAngle(armBaseStepperTarget);
    armBaseStepper.wait();
}

/// @brief 更新颜色放置的位置参数
/// @param Color
void update_Color_Place(int Color)
{
    gripperStepper.update_CurrentPos();
    armBaseStepper.update_CurrentPos();
    STEPPER_GRIPPER[Color] = gripperStepper.currentPosition;
    ARM_BASE_STEPPER_ANGLE[Color] = armBaseStepper.currentPosition;
}

/**
 * @brief 校准后计算机械臂原始臂长
 * @param len1 第一次移动时机械臂的伸出长度
 * @param angle1 第一次移动时机械臂的角度
 * @param len2 第二次移动时机械臂的伸出长度
 * @param angle2 第二次移动时机械臂的角度
 * @param circleSpacing 两个圆环的间距
 * @return float 计算得到的机械臂原始臂长
 */
double calculateArmZeroLength(double len1, double angle1, double len2, double angle2, double circleSpacing)
{
    // 将角度从 0.1 度转换为弧度
    double rad1 = angle1 * M_PI / 1800.0;
    double rad2 = angle2 * M_PI / 1800.0;
    // 计算角度差值
    double deltaTheta = fabs(rad1 - rad2);

    // 计算一元二次方程的系数
    double a = 2 - 2 * cos(deltaTheta);
    double b = 2 * (len1 + len2) - 2 * (len1 + len2) * cos(deltaTheta);
    double c = len1 * len1 + len2 * len2 - 2 * len1 * len2 * cos(deltaTheta) - circleSpacing * circleSpacing;

    // 计算判别式
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
    {
        // 无实数解，返回默认值
        return static_cast<double>(Arm_Zero_Length);
    }

    // 计算两个根
    double root1 = (-b + sqrt(discriminant)) / (2 * a);
    double root2 = (-b - sqrt(discriminant)) / (2 * a);

    // 选择合理的根（通常取正值）
    if (root1 >= 0)
    {
        return root1;
    }
    else if (root2 >= 0)
    {
        return root2;
    }
    else
    {
        // 没有合理的根，返回默认值
        return static_cast<double>(Arm_Zero_Length);
    }
}
/**
 * @brief 根据蓝色（左边）和绿色（中间）物料的位置参数，计算红色（右边）物料的位置参数。默认从左往右依次为蓝绿红
 * 此函数通过三角函数计算红色物料的夹爪步进电机目标位置和机械臂基座步进电机目标角度，
 * 并将计算结果存储在对应的全局数组中。
 */
void Calc_Color3_Place()
{
    // 根据红、蓝、绿的位置参数，计算机械臂原始臂长
    double armZeroLength = calculateArmZeroLength(static_cast<double>(STEPPER_GRIPPER[PLACEMENT_3]),
                                                  static_cast<double>(ARM_BASE_STEPPER_ANGLE[PLACEMENT_3]),
                                                  static_cast<double>(STEPPER_GRIPPER[PLACEMENT_2]),
                                                  static_cast<double>(ARM_BASE_STEPPER_ANGLE[PLACEMENT_2]),
                                                  static_cast<double>(CirCle_Spacing));
    Arm_Zero_Length = static_cast<float>(armZeroLength);

    // 计算红色物料位置在 X 轴上的分量
    // 先将角度从 0.1 度转换为弧度，再使用余弦函数计算投影，最后计算差值
    double vectorX = 2 * (static_cast<double>(STEPPER_GRIPPER[PLACEMENT_2]) + armZeroLength) * std::cos(ARM_BASE_STEPPER_ANGLE[PLACEMENT_2] * M_PI / 1800.0) - (static_cast<double>(STEPPER_GRIPPER[PLACEMENT_3]) + armZeroLength) * std::cos(ARM_BASE_STEPPER_ANGLE[PLACEMENT_3] * M_PI / 1800.0);
    // 计算红色物料位置在 Y 轴上的分量
    double vectorY = 2 * (static_cast<double>(STEPPER_GRIPPER[PLACEMENT_2]) + armZeroLength) * std::sin(ARM_BASE_STEPPER_ANGLE[PLACEMENT_2] * M_PI / 1800.0) - (static_cast<double>(STEPPER_GRIPPER[PLACEMENT_3]) + armZeroLength) * std::sin(ARM_BASE_STEPPER_ANGLE[PLACEMENT_3] * M_PI / 1800.0);
    // 使用 hypot 函数计算向量的模长，减去零位长度得到最终位置
    STEPPER_GRIPPER[PLACEMENT_1] = static_cast<float>(std::hypot(vectorX, vectorY) - armZeroLength);
    // 计算红色物料机械臂基座步进电机的原始目标角度
    // 使用 atan2 函数计算向量的角度，再将结果从弧度转换为 0.1 度
    double rawAngle = std::atan2(vectorY, vectorX) * 1800.0 / M_PI;
    // 若原始角度为负，加上 3600（即 360 度）将其转换为 0 到 3600 范围内的正角
    if (rawAngle < 0)
    {
        rawAngle += 3600;
    }
    // 将角度限制在 0 到 3450 的范围内（对应 0 到 345 度，单位 0.1 度）
    ARM_BASE_STEPPER_ANGLE[PLACEMENT_1] = static_cast<float>(std::clamp(rawAngle, 0.0, 3450.0));
}

/**
 * @brief 控制夹爪直线移动到目标位置
 * @param deltaX  X 轴偏移量，单位：0.1mm向右为正
 * @param deltaY  Y 轴偏移量，单位：0.1mm向外为正
 */
void Gripper_Move_Direct(float deltaX, float deltaY)
{
    // 更新电机的当前位置
    gripperStepper.update_CurrentPos();
    armBaseStepper.update_CurrentPos();

    // 获取当前夹爪的位置
    float currentX = (gripperStepper.currentPosition + Arm_Zero_Length) * cos((armBaseStepper.currentPosition - ARM_BASE_Stepper_HOME_ANGLE) * M_PI / 1800.0);
    float currentY = (gripperStepper.currentPosition + Arm_Zero_Length) * sin((armBaseStepper.currentPosition - ARM_BASE_Stepper_HOME_ANGLE) * M_PI / 1800.0);

    // 计算目标位置
    float targetX = currentX + deltaX;
    float targetY = currentY + deltaY;

    // 设定控制参数
    const float tolerance = 200.0;  // 允许的误差范围，单位：0.1mm
    const int maxIterations = 1000; // 迭代次数
    int iteration = 0;
    ARM_PID.PID_Init();
    ARM_BASE_PID.PID_Init();
    float ptime = millis() - 50;
    // 循环控制电机移动
    while (iteration < maxIterations)
    {
        // 更新当前电机位置
        gripperStepper.update_CurrentPos();
        armBaseStepper.update_CurrentPos();

        // 获取当前夹爪的位置
        float currentX = (gripperStepper.currentPosition + Arm_Zero_Length) * cos((armBaseStepper.currentPosition - ARM_BASE_Stepper_HOME_ANGLE) * M_PI / 1800.0);
        float currentY = (gripperStepper.currentPosition + Arm_Zero_Length) * sin((armBaseStepper.currentPosition - ARM_BASE_Stepper_HOME_ANGLE) * M_PI / 1800.0);

        // 计算当前位置与目标位置的误差（全局坐标系）
        float errorX = targetX - currentX;
        float errorY = targetY - currentY;
        float distance = std::hypot(errorX, errorY);

        // 判断是否到达目标位置
        if (distance <= tolerance)
        {
            break;
        }
        // 获取当前坐标系角度差值
        float currentAngle = (armBaseStepper.currentPosition - ARM_BASE_Stepper_HOME_ANGLE) * M_PI / 1800.0 + 0.5 * M_PI;
        // 将全局坐标系下的误差转换到夹爪的附体坐标系
        float localErrorX = errorX * cos(currentAngle) + errorY * sin(currentAngle);
        float localErrorY = -errorX * sin(currentAngle) + errorY * cos(currentAngle);
        // 调用 Follow_by_Arm 函数进行调整
        Follow_by_Arm(localErrorX, localErrorY, 100, 75, 0, 0);
        iteration++;
    }
}
