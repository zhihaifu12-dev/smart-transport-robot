#include "MoveByPosition.h"
// 电机对象实例化
float CurrentRad = 0;
float CurrentYaw = 0;
// 全局坐标，记录当前坐标，单位为mm
float Current_X = 0;
float Current_Y = 0;
// 记录绝对误差
float Error_mm = 0.0;
float Error_Rad = 0.0;
float Delta_Rad = 0.0;
// 定义运动状态
CARSTATE CarState = IDLE;
bool isRunning = false;
// [实机必调] 所有点位格式：{X(mm), Y(mm), 朝向(rad), 平移rpm, 平移加速s, 旋转rpm, 旋转加速s}。
// 当前纯移动main使用其中的X/Y/朝向；速度统一使用main.cpp中的TEST_*参数。
// X/Y应按“车体中心”测量，并预留车宽、机械臂伸出量和障碍物安全距离。
CAR_GOAL_POINT MoveSquence[] = {
#if START_ZONE == 1
    {2250, 2250, M_PI / 2, 300, 2, 150, 2},  // [实机必调] 0 启停区1，行驶角0度
#else
    {2250, 150, M_PI / 2, 300, 2, 150, 2},   // 0 启停区2，行驶角0度
#endif
    {2250, QR_ZONE_Y, M_PI / 2, 250, 2, 120, 2},             // 1 二维码区，与启停区同X，无横移
    {RAW_ZONE_X + 50, RAW_ZONE_Y - 25, M_PI, 220, 2, 120, 2},          // [实机必调] 2 原料区：车头-X
    {RAW_ZONE_X, COARSE_ZONE_Y - 30, 0, 120, 2, 80, 2},           // [实机必调] 3 粗加工区：车头+X
    {TEMPORARY_ZONE_X - 20, TEMPORARY_ZONE_Y - 20, -M_PI / 2, 150, 2, 80, 2}, // [实机必调] 4 暂存区：车头-Y
    {MAP_CENTER, MAP_CENTER, M_PI / 2, 300, 2, 150, 2},      // 5 地图中心，仅用于必要的功能区换向
    {2250, QR_ZONE_Y, M_PI / 2, 220, 2, 100, 2},             // 6 返程二维码区（运行时按启停区覆盖）
    {0, 0, M_PI / 2, 220, 2, 100, 2},                        // 7 最终停止点（运行时按启停区覆盖）
    // 折线拐点坐标由相邻功能区坐标组合，功能区调参后会同步变化。
    {TEMPORARY_ZONE_X - 20, COARSE_ZONE_Y - 30, M_PI / 2, 220, 2, 100, 2}, // 8 离点时强制顺时针约90度并倒车去暂存区
    {TEMPORARY_ZONE_X - 20, RAW_ZONE_Y, M_PI / 2, 220, 2, 100, 2}     // 9 离点时强制顺时针约90度并倒车去原料区
};

// [第二轮地图，实机必调]
// 当前先完整复制第一轮实测参数。第二轮原料区、粗加工区、暂存区或拐点发生偏差时，
// 只修改此数组中的2、3、4、8、9号点，不会影响第一轮地图。
CAR_GOAL_POINT MoveSquenceRound2[] = {
#if START_ZONE == 1
    {2250, 2250, M_PI / 2, 300, 2, 150, 2},  // 0 启停区1：第二轮不使用，保留完整索引
#else
    {2250, 150, M_PI / 2, 300, 2, 150, 2},   // 0 启停区2：第二轮不使用，保留完整索引
#endif
    {2250, QR_ZONE_Y, M_PI / 2, 250, 2, 120, 2}, // 1 去程二维码区：第二轮不使用，保留完整索引
    {RAW_ZONE_X, RAW_ZONE_Y - 30, M_PI, 220, 2, 120, 2}, // 2 [第二轮实机必调] 原料区
    {RAW_ZONE_X -50, COARSE_ZONE_Y - 45, 0, 120, 2, 80, 2}, // 3 [第二轮实机必调] 粗加工区
    {TEMPORARY_ZONE_X - 20, TEMPORARY_ZONE_Y - 20, -M_PI / 2, 150, 2, 80, 2}, // 4 [第二轮实机必调] 暂存区
    {MAP_CENTER, MAP_CENTER, M_PI / 2, 300, 2, 150, 2}, // 5 地图中心：第二轮任务完成后使用公共中心点
    {2250, QR_ZONE_Y, M_PI / 2, 220, 2, 100, 2}, // 6 返程二维码区：使用公共返程点，本数组值不参与运行
    {0, 0, M_PI / 2, 220, 2, 100, 2}, // 7 最终停止区：使用公共终点，本数组值不参与运行
    {TEMPORARY_ZONE_X - 20, COARSE_ZONE_Y - 45, M_PI / 2, 220, 2, 100, 2}, // 8 [第二轮实机必调] 粗加工区 -> 暂存区拐点
    {TEMPORARY_ZONE_X - 20, RAW_ZONE_Y, M_PI / 2, 220, 2, 100, 2} // 9 [第二轮实机必调] 第一轮暂存区 -> 第二轮原料区拐点
};
// 全局指针，用于遍历移动动作
int MoveIndex = 0;
// 全局点位设置，用MoveIndex遍历点位
//int MovePoint[] = {0,12,13,19,20,14,15,7,16,17,11};
int MovePoint[] = {
    POINT_START,
    POINT_QR,
    POINT_RAW,
    POINT_COARSE,
    POINT_TEMPORARY,
    POINT_CENTER,
    POINT_RETURN_QR,
    POINT_FINISH};
//  关于PID闭环控制有关的阈值参数，速度计算，ClosedLoop_cal()
float ClosedLoop_mm = 0; // 进入PID闭环控制的阈值
float Slow_mm = 0;       // 闭环控制中线性减速的阈值d
// 用于间断运动
unsigned long lastMoveTime = 0;
const long MoveInterval = 0;
// 实例化电机
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);
AccelStepper stepper3 = AccelStepper(motorInterfaceType, stepPin3, dirPin3);
AccelStepper stepper4 = AccelStepper(motorInterfaceType, stepPin4, dirPin4);

void Motor_ReenableOutputs()
{
    // 四个底盘驱动器共用PE13，先失能再重新拉低使能，可恢复上电波动后
    // 个别驱动器没有正确进入使能状态的问题。
    pinMode(enPin, OUTPUT);
    digitalWrite(enPin, HIGH);
    delay(5);

    stepper1.enableOutputs();
    stepper2.enableOutputs();
    stepper3.enableOutputs();
    stepper4.enableOutputs();

    digitalWrite(enPin, LOW); // 低电平使能M1~M4
}

/// @brief 初始化电机、使能电机与复位电机
void Motor_Init() // 用于电机的复位操作
{
    Motor_ReenableOutputs();

    // 给驱动器留出更可靠的STEP高电平时间，避免某一路漏识别窄脉冲。
    stepper1.setMinPulseWidth(3);
    stepper2.setMinPulseWidth(3);
    stepper3.setMinPulseWidth(3);
    stepper4.setMinPulseWidth(3);

    stepper1.setCurrentPosition(0); // 复位步进电机初始位置
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);
    stepper4.setCurrentPosition(0);
    Current_X = MoveSquence[POINT_START].x;
    Current_Y = MoveSquence[POINT_START].y;
}

/// @brief 设定运动的转速与加速时间
/// @param Rpm
/// @param Accel_Time
void Motor_Setup(float Rpm, float Accel_Time) // Rpm为转速 转/min，最大为3000rpm,Accel_Time为到达最大速度的时间，单位为s
{
    // 用于设置电机的转速、加速度等
    float MaxSpeed = (Pulse_Num * Rpm) / 60.0;
    float Acceleration = (Pulse_Num * Rpm) / (60.0 * Accel_Time);
    // 电机初始化操作
    stepper1.setMaxSpeed(MaxSpeed); // 设置1#电机最大速度，单位为脉冲数/s；
    stepper1.setAcceleration(Acceleration);

    stepper2.setMaxSpeed(MaxSpeed); // 设置2#电机最大速度，单位为脉冲数/s；
    stepper2.setAcceleration(Acceleration);

    stepper3.setMaxSpeed(MaxSpeed); // 设置3#电机最大速度，单位为脉冲数/s；
    stepper3.setAcceleration(Acceleration);

    stepper4.setMaxSpeed(MaxSpeed); // 设置4#电机最大速度，单位为脉冲数/s；
    stepper4.setAcceleration(Acceleration);
}
/// @brief 相对移动
/// @param X_Distance
/// @param Y_Distance
/// @param Rpm
/// @param Accel_Time
void MoveCar(float X_Distance, float Y_Distance, float Rpm, float Accel_Time)
{
    Motor_Setup(Rpm, Accel_Time);

    // 车体坐标：+X前进，-X后退，+Y左移，-Y右移，单位mm。
    // 与实测正方形程序一致：
    // 前进 (+X, 0): M1-, M2+, M3-, M4+
    // 左移 (0, +Y): M1+, M2+, M3-, M4-
    // 后退 (-X, 0): M1+, M2-, M3+, M4-
    // 右移 (0, -Y): M1-, M2-, M3+, M4+
    const long xPulses = lroundf(X_Distance * PulseNum_mm);
    const long yPulses = lroundf(Y_Distance * PulseNum_mm);

    stepper1.move(motor1_CW * (xPulses - yPulses));
    stepper2.move(motor2_CW * (xPulses + yPulses));
    stepper3.move(motor3_CW * (xPulses + yPulses));
    stepper4.move(motor4_CW * (xPulses - yPulses));

    CarState = Moving;
}
/// @brief 用于全局坐标移动下的开环点位移动
/// @param X_Global_Distance
/// @param Y_Global_Distance
/// @param Rpm
/// @param Accel_Time
void Global_MoveCar(float X_Global_Distance, float Y_Global_Distance, float Rpm, float Accel_Time) // 全局坐标下x方向与y方向移动的距离，单位为mm；
{
    float X_Distance = X_Global_Distance * cos(CurrentRad) + Y_Global_Distance * sin(CurrentRad); // 换算到局部坐标
    float Y_Distance = Y_Global_Distance * cos(CurrentRad) - X_Global_Distance * sin(CurrentRad);
    MoveCar(X_Distance, Y_Distance, Rpm, Accel_Time);
}
/// @brief 用于全局坐标下的闭环点位移动
/// @param Target_X
/// @param Target_Y
/// @param Rpm
/// @param Accel_Time
void MoveCar_toTarget(float Target_X, float Target_Y, float Rpm, float Accel_Time)
{
    // 函数参数为目标坐标，转速以及加速度，加入PID反馈，适用于绝对坐标下的走点，坐标为全局坐标
    MoveX_PID.PID_Calc(Target_X, Current_X); // 获取X方向第一次的误差以及输出量
    MoveY_PID.PID_Calc(Target_Y, Current_Y); // 获取Y方向第一次的误差以及输出量
    // 动态调整速度：大误差时高速，小误差时低速
    float adaptiveRpm = Rpm;
    float Slow_mm = 100;
    if (fabs(MoveX_PID.error) < Slow_mm && fabs(MoveY_PID.error) < Slow_mm)
    // 必须是逻辑与，因为只往一个方向走时，速度会始终限制在另一个方向的Rpm,并且防止过小引发的反向放大
    {
        adaptiveRpm = max(fabs(Rpm * (MoveX_PID.error / Slow_mm)),  // 选用max而不是min原因同上
                          fabs(Rpm * (MoveY_PID.error / Slow_mm))); // 线性减速
        adaptiveRpm = max(adaptiveRpm, (float)(5.0));
    }

    Global_MoveCar(MoveX_PID.output, MoveY_PID.output, adaptiveRpm, Accel_Time);
}
/// @brief 相对角度转动
/// @param Theta
/// @param Rpm
/// @param Accel_Time
void RotateCar(float Theta, float Rpm, float Accel_Time)
{
    // 原地自转角度，只需要简化成为每次旋转M_PI/2即可，并指定方向，以俯视逆时针为正
    Motor_Setup(Rpm, Accel_Time);

    // 四舍五入保留不足一个脉冲的角度精度，避免强制转long造成系统性少转。
    const long rotationPulses = lroundf(Theta * rotationFactor);
    stepper1.move(motor1_CW * -rotationPulses);
    stepper2.move(motor2_CW * rotationPulses);
    stepper3.move(motor3_CW * -rotationPulses);
    stepper4.move(motor4_CW * rotationPulses);

    CarState = Rotating;
}
/// @brief 用于全局坐标下的闭环角度调整
/// @param TargetRad
/// @param Rpm
/// @param Accel_Time
void RotateCar_toTarget(float TargetRad, float Rpm, float Accel_Time)
{
    // 用于原地自转，函数参数为当前姿态角与目标姿态角，转速以及加速度，加入PID反馈，适用于绝对坐标下的走点
    Rot_PID.PID_Calc(TargetRad, CurrentRad); // 更新误差以及输出量
    // 动态调整速度：大误差时高速，小误差时低速
    float adaptiveRpm = Rpm;
    float SlowDown_rad = 0.1;

    if (fabs(Rot_PID.error) < SlowDown_rad) // 0.1rad ≈ 6°
    {
        adaptiveRpm = fabs(Rpm * (Rot_PID.error / (SlowDown_rad))); // 线性减速
    }
    RotateCar(Rot_PID.output, adaptiveRpm, Accel_Time);
}
/// @brief 使能电机运转，即发送脉冲
void RunMotors()
{
    // 电机运转函数，发送脉冲
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
}
/// @brief 根据移动点位计算当前移动下的闭环阈值与线性减速阈值
void CloseLoop_Cal()
{
    float delta_x;
    float delta_y;
    if (MoveIndex != 0)
    {
        delta_x = fabs(MoveSquence[MoveIndex].x - MoveSquence[MoveIndex - 1].x);
        delta_y = fabs(MoveSquence[MoveIndex].y - MoveSquence[MoveIndex - 1].y);
    }
    else
    {
        delta_x = fabs(MoveSquence[0].x - MoveSquence[Point_Num - 1].x);
        delta_y = fabs(MoveSquence[0].y - MoveSquence[Point_Num - 1].y);
    }

    Slow_mm = max((int)max(delta_x, delta_y) / 20, 30); // 防止小位移动时无法正常减速
}
/// @brief 总运动函数，用于根据运动状态与目标点位，先移动后旋转；支持线性减速，间隔运动
void RunCar()
{
    // 小车移动集成函数
    // 主要使用状态机实现不同状态的切换，全局变量MoveMent设定每次运动的情况，使用MoveIndex进行遍历
    // 用于获取当前绝对坐标偏差值，计算距离目标点的欧式距离
    // 不能使用定时器实时获取小车状态，因为会导致误差获取频率过低
    Error_mm = sqrt(pow(MoveSquence[MoveIndex].x - Current_X, 2) + pow(MoveSquence[MoveIndex].y - Current_Y, 2));
    Delta_Rad = (MoveSquence[MoveIndex].z - CurrentRad);
    if (fabs(Delta_Rad) > M_PI) // 对临界点的偏差进行修正，即180~-180度突变的点
    {
        Error_Rad = (Delta_Rad > 0) ? Delta_Rad - 2 * M_PI : Delta_Rad + 2 * M_PI; // 表示控制量
    }
    else
    {
        Error_Rad = Delta_Rad;
    }
    isRunning = (stepper1.isRunning() ||
                 stepper2.isRunning() ||
                 stepper3.isRunning() ||
                 stepper4.isRunning());
    RunMotors();      // 循环使能电机
    switch (CarState) // 状态转移,计时放在旋转完与下一次移动之间
    {
    case IDLE:
        if (millis() - lastMoveTime > MoveInterval)
        {
            MoveCar_toTarget(MoveSquence[MoveIndex].x,
                             MoveSquence[MoveIndex].y,
                             MoveSquence[MoveIndex].Move_Rpm,
                             MoveSquence[MoveIndex].Move_Accel_Time);
            CloseLoop_Cal(); // 计算本次运动的相关运动参数
        }
        break;
    case Moving:
        if (!isRunning && fabs(Error_mm) <= MaxError_Move)
        {
            CarState = Move_Complete;
        }
        else if (fabs(Error_mm) > MaxError_Move)
        {
            MoveCar_toTarget(MoveSquence[MoveIndex].x,
                             MoveSquence[MoveIndex].y,
                             MoveSquence[MoveIndex].Move_Rpm,
                             MoveSquence[MoveIndex].Move_Accel_Time);
        }
        break;
    case Move_Complete:
        Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut); // 表示已经调整到位，重新初始化PID控制器
        RotateCar_toTarget(MoveSquence[MoveIndex].z, MoveSquence[MoveIndex].Rot_Rpm, MoveSquence[MoveIndex].Rot_Accel_Time);
        break;
    case Rotating:
        if (!isRunning && fabs(Error_Rad) <= MaxError_Rot)
        {
            CarState = Rotate_Complete;
        }
        else
        {
            RotateCar_toTarget(MoveSquence[MoveIndex].z, MoveSquence[MoveIndex].Rot_Rpm, MoveSquence[MoveIndex].Rot_Accel_Time);
        }
        break;
    case Rotate_Complete:
        MoveX_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
        MoveY_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
        // 表示已经调整到位，重新初始化PID控制器
        // MoveIndex++;
        // if (MoveIndex == Point_Num)
        //     MoveIndex = 0; // 溢出，回到循环开始
        // CarState = IDLE;***************这部分操作改由操作机器人实现
        lastMoveTime = millis();
        break;
    }
}

/// @brief 使能速度控制的电机，使电机处于匀速运动
void RunMotors_Speed()
{
    // 使能电机
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
    stepper4.runSpeed();
}
/// @brief 采用对数自适应调整比例系数
/// @param Kp 基础比例系数
/// @param Rpm 设定最大速度，测试为200
/// @param Accel_time 设定加速时间，测试为2s
void Follow_bySpeed(float Kp, float Rpm, float Accel_time, float Move_X_Grab, float Move_Y_Grab)
{
    // 用于设置电机的转速、加速度等
    Motor_Setup(Rpm, Accel_time);
    // 发送脉冲驱动电机
    // RunMotors_Speed();
    float Delta_Rad = (MoveSquence[MoveIndex].z - CurrentRad);
    float Error_Rad = Delta_Rad;
    if (fabs(Delta_Rad) > M_PI) // 对临界点的偏差进行修正，即180~-180度突变的点
    {
        Error_Rad = (Delta_Rad > 0) ? Delta_Rad - 2 * M_PI : Delta_Rad + 2 * M_PI; // 表示控制量
    }

    // 自适应比例环
    float Kp_x = Kp + log10(fabs(Move_X_Grab) + 1);
    float Kp_y = Kp + log10(fabs(Move_Y_Grab) + 1);
    float Kp_z = Kp / 3 + log10(fabs(Error_Rad) + 1);

    float Global_vx = Kp_x * Move_X_Grab;
    float Global_vy = Kp_y * Move_Y_Grab;
    float Global_vz = Kp_z * Error_Rad;

    // 计算当前目标移速
    float targetStepper1Speed = motor1_CW * (Global_vx - Global_vy - Global_vz * (LR_WHEELS_DISTANCE + FR_WHEELS_DISTANCE) / 2) * 60.0 / C_Wheel;
    float targetStepper2Speed = motor2_CW * (Global_vx + Global_vy + Global_vz * (LR_WHEELS_DISTANCE + FR_WHEELS_DISTANCE) / 2) * 60.0 / C_Wheel;
    float targetStepper3Speed = motor3_CW * (Global_vx + Global_vy - Global_vz * (LR_WHEELS_DISTANCE + FR_WHEELS_DISTANCE) / 2) * 60.0 / C_Wheel;
    float targetStepper4Speed = motor4_CW * (Global_vx - Global_vy + Global_vz * (LR_WHEELS_DISTANCE + FR_WHEELS_DISTANCE) / 2) * 60.0 / C_Wheel;
    // 电机转速设置
    stepper1.setSpeed(Pulse_Num * targetStepper1Speed / 60.0);
    stepper2.setSpeed(Pulse_Num * targetStepper2Speed / 60.0);
    stepper3.setSpeed(Pulse_Num * targetStepper3Speed / 60.0);
    stepper4.setSpeed(Pulse_Num * targetStepper4Speed / 60.0);
}

/// @brief 用于OPS失效下的开环跑点
void RunCar_open()
{
    digitalWrite(LED, 1);
    RunMotors(); // 循环使能电机
    if (MoveIndex == 0)
        MoveIndex = 1; // 跳过原点的点位
    Delta_Rad = (MoveSquence[MoveIndex].z - CurrentRad);
    if (fabs(Delta_Rad) > M_PI) // 对临界点的偏差进行修正，即180~-180度突变的点
    {
        Error_Rad = (Delta_Rad > 0) ? Delta_Rad - 2 * M_PI : Delta_Rad + 2 * M_PI; // 表示控制量
    }
    else
    {
        Error_Rad = Delta_Rad;
    }

    switch (CarState) // 状态转移,计时放在旋转完与下一次移动之间
    {
    case IDLE:
        if (millis() - lastMoveTime > MoveInterval) // 用于设定定时运动
        {
            Global_MoveCar(MoveSquence[MoveIndex].x - MoveSquence[MoveIndex - 1].x,
                           MoveSquence[MoveIndex].y - MoveSquence[MoveIndex - 1].y,
                           MoveSquence[MoveIndex].Move_Rpm,
                           MoveSquence[MoveIndex].Move_Accel_Time);
        }
        break;
    case Moving:
        if (!stepper1.isRunning() &&
            !stepper2.isRunning() &&
            !stepper3.isRunning() &&
            !stepper4.isRunning())
        {
            CarState = Move_Complete;
        }
        break;
    case Move_Complete:
        Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut); // 表示已经调整到位，重新初始化PID控制器
        RotateCar(MoveSquence[MoveIndex].z, MoveSquence[MoveIndex].Rot_Rpm, MoveSquence[MoveIndex].Rot_Accel_Time);
        break;
    case Rotating:
        if (!stepper1.isRunning() &&
            !stepper2.isRunning() &&
            !stepper3.isRunning() &&
            !stepper4.isRunning() &&
            fabs(Error_Rad) <= MaxError_Rot)
        {
            CarState = Rotate_Complete;
        }
        else
        {
            RotateCar_toTarget(MoveSquence[MoveIndex].z, MoveSquence[MoveIndex].Rot_Rpm, MoveSquence[MoveIndex].Rot_Accel_Time);
        }
        break;
    case Rotate_Complete:
        Current_X = MoveSquence[MoveIndex].x;
        Current_Y = MoveSquence[MoveIndex].y;
        // MoveIndex++;
        // if (MoveIndex == Point_Num)
        //     MoveIndex = 0; // 溢出，回到循环开始
        // CarState = IDLE;
        lastMoveTime = millis();
        break;
    }
}

/// @brief 用于同时平移与旋转的运动，并无拐弯
/// @param Target_X
/// @param Target_Y
/// @param TargetRad
/// @param Rpm
/// @param Accel_Time
void RunToTarget(float Target_X, float Target_Y, float TargetRad, float Rpm, float Accel_Time, int Point)
{
    MoveX_PID.PID_Calc(Target_X, Current_X); // 获取X方向第一次的误差以及输出量
    MoveY_PID.PID_Calc(Target_Y, Current_Y); // 获取Y方向第一次的误差以及输出量
    Rot_PID.PID_Calc(TargetRad, CurrentRad); // 获取z方向第一次的误差以及输出量

    float adaptiveRpm = Rpm;
    // 若有旋转，则旋转时开始线性减速
    // 若无旋转，则根据平移开始线性减速
    float SlowDown_rad = 0;
    float Slow_mm = 0;
    switch (Point)
    {
    case 2:
    case 4:
    case 7:
    case 9:
        SlowDown_rad = 1;
        Slow_mm = 100;
        break;
    default:
        SlowDown_rad = 0;
        Slow_mm = 0;
        break;
    }

    if (fabs(MoveX_PID.error) < Slow_mm && fabs(MoveY_PID.error) < Slow_mm && fabs(Rot_PID.error) < SlowDown_rad)
    // 必须是逻辑与，因为只往一个方向走时，速度会始终限制在另一个方向的Rpm,并且防止过小引发的反向放大
    {
        adaptiveRpm = max(fabs(Rpm * (MoveX_PID.error / Slow_mm)),  // 选用max而不是min原因同上
                          fabs(Rpm * (MoveY_PID.error / Slow_mm))); // 线性减速
        adaptiveRpm = max(adaptiveRpm, fabs(Rpm * (Rot_PID.error / (SlowDown_rad))));
        adaptiveRpm = max(adaptiveRpm, (float)(5.0));
    }

    float X_Distance = MoveX_PID.output * cos(CurrentRad) + MoveY_PID.output * sin(CurrentRad); // 换算到局部坐标
    float Y_Distance = MoveY_PID.output * cos(CurrentRad) - MoveX_PID.output * sin(CurrentRad);

    Motor_Setup(adaptiveRpm, Accel_Time);
    long stepper1_move = (long)((X_Distance - Y_Distance) * PulseNum_mm - Rot_PID.output * rotationFactor);
    long stepper2_move = (long)((X_Distance + Y_Distance) * PulseNum_mm + Rot_PID.output * rotationFactor);
    long stepper3_move = (long)((X_Distance + Y_Distance) * PulseNum_mm - Rot_PID.output * rotationFactor);
    long stepper4_move = (long)((X_Distance - Y_Distance) * PulseNum_mm + Rot_PID.output * rotationFactor);
    stepper1.move(motor1_CW * stepper1_move);
    stepper2.move(motor2_CW * stepper2_move);
    stepper3.move(motor3_CW * stepper3_move);
    stepper4.move(motor4_CW * stepper4_move);

    CarState = Moving;
}
void RunCar_MaR()
{
    // 小车移动集成函数
    // 移动方式为同时旋转与平移
    // 状态改为三个状态
    float Error_X = MoveSquence[MoveIndex].x - Current_X;
    float Error_Y = MoveSquence[MoveIndex].y - Current_Y;
    Error_mm = sqrt(pow(Error_X, 2) + pow(Error_Y, 2));
    Delta_Rad = (MoveSquence[MoveIndex].z - CurrentRad);
    if (fabs(Delta_Rad) > M_PI) // 对临界点的偏差进行修正，即180~-180度突变的点
    {
        Error_Rad = (Delta_Rad > 0) ? Delta_Rad - 2 * M_PI : Delta_Rad + 2 * M_PI; // 表示控制量
    }
    else
    {
        Error_Rad = Delta_Rad;
    }
    isRunning = (stepper1.isRunning() ||
                 stepper2.isRunning() ||
                 stepper3.isRunning() ||
                 stepper4.isRunning());

    RunMotors();      // 循环使能电机
    switch (CarState) // 状态转移,计时放在旋转完与下一次移动之间
    {
    case IDLE:
        if (millis() - lastMoveTime > 0)
        {
            if (MoveIndex == 5)
            {
                Global_MoveCar(MoveSquence[MoveIndex].x - MoveSquence[MoveIndex - 1].x,
                               MoveSquence[MoveIndex].y - MoveSquence[MoveIndex - 1].y,
                               MoveSquence[MoveIndex].Move_Rpm,
                               MoveSquence[MoveIndex].Move_Accel_Time);
            }
            else
            {
                RunToTarget(MoveSquence[MoveIndex].x,
                            MoveSquence[MoveIndex].y,
                            MoveSquence[MoveIndex - 1].z,
                            MoveSquence[MoveIndex].Move_Rpm,
                            MoveSquence[MoveIndex].Move_Accel_Time,
                            MoveIndex);
            }
        }
        break;
    case Moving:
        if (!isRunning) // 让该点运动到末端时直接进入下一个点位
        {
            CarState = Rotate_Complete;
        }
        else
        {
            if (fabs(Error_mm) <= MaxError_Move && fabs(Error_Rad) <= MaxError_Rot)
            {
                CarState = Rotate_Complete;
            }
            else if (fabs(Error_mm) > Range_Of_MAR || fabs(Error_Rad) > Range_Of_MAR)
            { // 当误差大于合成运动误差范围时，只平移不旋转
                RunToTarget(MoveSquence[MoveIndex].x,
                            MoveSquence[MoveIndex].y,
                            MoveSquence[MoveIndex - 1].z,
                            MoveSquence[MoveIndex].Move_Rpm,
                            MoveSquence[MoveIndex].Move_Accel_Time,
                        MoveIndex);
            }
            else if (fabs(Error_mm) > MaxError_Move || fabs(Error_Rad) > MaxError_Rot)
            {
                RunToTarget(MoveSquence[MoveIndex].x,
                            MoveSquence[MoveIndex].y,
                            MoveSquence[MoveIndex].z,
                            MoveSquence[MoveIndex].Move_Rpm,
                            MoveSquence[MoveIndex].Move_Accel_Time,
                        MoveIndex);
            }
            // 下列点无需走非常准
            switch (MoveIndex)
            {
            case 1: // 扫码
                // x 方向误差影响不大的点(或者x是其运动方向)
                if (fabs(Error_X) < MaxError_Move * 10 && fabs(Error_Y) < MaxError_Move && fabs(Error_Rad) < MaxError_Rot)
                {
                    CarState = Rotate_Complete;
                }
                break;
            case 3: // 退出转盘
            case 5: // 十字路口
            case 6: // 粗加工区外
                // Y方向误差影响不大的点(或者y是其运动方向)
                if (fabs(Error_X) < MaxError_Move && fabs(Error_Y) < MaxError_Move * 10 && fabs(Error_Rad) < MaxError_Rot)
                {
                    CarState = Rotate_Complete;
                }
                break;
            default:
                break;
            }
        }
        break;
    case Rotate_Complete:
        MoveX_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
        MoveY_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
        Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut); // 表示已经调整到位，重新初始化PID控制器
        // MoveIndex++;
        // if (MoveIndex == Point_Num)
        // {
        //      MoveIndex = 0; // 溢出，回到循环开始
        //  }
        // CarState = IDLE;
        lastMoveTime = millis();
        break;
    }
}

/// @brief 同时移动旋转，加入点位列表
void RunCar_MaR_Point()
{
    // 小车移动集成函数
    // 移动方式为同时旋转与平移
    // 状态改为三个状态
    float Error_X = MoveSquence[MovePoint[MoveIndex]].x - Current_X;
    float Error_Y = MoveSquence[MovePoint[MoveIndex]].y - Current_Y;
    Error_mm = sqrt(pow(Error_X, 2) + pow(Error_Y, 2));
    Delta_Rad = (MoveSquence[MovePoint[MoveIndex]].z - CurrentRad);
    if (fabs(Delta_Rad) > M_PI) // 对临界点的偏差进行修正，即180~-180度突变的点
    {
        Error_Rad = (Delta_Rad > 0) ? Delta_Rad - 2 * M_PI : Delta_Rad + 2 * M_PI; // 表示控制量
    }
    else
    {
        Error_Rad = Delta_Rad;
    }
    isRunning = (stepper1.isRunning() ||
                 stepper2.isRunning() ||
                 stepper3.isRunning() ||
                 stepper4.isRunning());

    RunMotors();      // 循环使能电机
    switch (CarState) // 状态转移,计时放在旋转完与下一次移动之间
    {
    case IDLE:
        if (millis() - lastMoveTime > 0)
        {
            // if (MovePoint[MoveIndex] == 5)
            // {
            //     Global_MoveCar(MoveSquence[MovePoint[MoveIndex]].x - MoveSquence[MovePoint[MoveIndex] - 1].x,
            //                    MoveSquence[MovePoint[MoveIndex]].y - MoveSquence[MovePoint[MoveIndex] - 1].y,
            //                    MoveSquence[MovePoint[MoveIndex]].Move_Rpm,
            //                    MoveSquence[MovePoint[MoveIndex]].Move_Accel_Time);
            // }
            // else
            {
                RunToTarget(MoveSquence[MovePoint[MoveIndex]].x,
                            MoveSquence[MovePoint[MoveIndex]].y,
                            MoveSquence[MovePoint[MoveIndex-1]].z,
                            MoveSquence[MovePoint[MoveIndex]].Move_Rpm,
                            MoveSquence[MovePoint[MoveIndex]].Move_Accel_Time,
                            MovePoint[MoveIndex]);
            }
        }
        break;
    case Moving:
        if (!isRunning) // 让该点运动到末端时直接进入下一个点位
        {
            CarState = Rotate_Complete;
        }
        else
        {
            if (fabs(Error_mm) <= MaxError_Move && fabs(Error_Rad) <= MaxError_Rot)
            {
                CarState = Rotate_Complete;
            }
            else if (fabs(Error_mm) > Range_Of_MAR)
            { // 当误差大于合成运动误差范围时，只平移不旋转
                RunToTarget(MoveSquence[MovePoint[MoveIndex]].x,
                            MoveSquence[MovePoint[MoveIndex]].y,
                            MoveSquence[MovePoint[MoveIndex-1]].z,
                            MoveSquence[MovePoint[MoveIndex]].Move_Rpm,
                            MoveSquence[MovePoint[MoveIndex]].Move_Accel_Time,
                            MovePoint[MoveIndex]);
            }
            else if (fabs(Error_mm) > MaxError_Move || fabs(Error_Rad) > MaxError_Rot)
            {
                RunToTarget(MoveSquence[MovePoint[MoveIndex]].x,
                            MoveSquence[MovePoint[MoveIndex]].y,
                            MoveSquence[MovePoint[MoveIndex]].z,
                            MoveSquence[MovePoint[MoveIndex]].Move_Rpm,
                            MoveSquence[MovePoint[MoveIndex]].Move_Accel_Time,
                            MovePoint[MoveIndex]);
            }
            // 下列点无需走非常准
            // 需要对特殊点进行调整
            switch (MovePoint[MoveIndex])
            {
            case POINT_QR:
            case POINT_RAW:
            case POINT_COARSE:
            case POINT_TEMPORARY:
            case POINT_CENTER:
            case POINT_RETURN_QR:
            case POINT_FINISH:
                // Y方向误差影响不大的点(或者y是其运动方向)
                if (fabs(Error_mm) <= MaxError_Move * 5 && fabs(Error_Rad) < 2 * MaxError_Rot)
                {
                    CarState = Rotate_Complete;
                }
                break;
            default:
                break;
            }
        }
        break;
    case Rotate_Complete:
        MoveX_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
        MoveY_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
        Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut); // 表示已经调整到位，重新初始化PID控制器
        lastMoveTime = millis();
        break;
    }
}
/// @brief 转盘区移动机器人跟随，只移动Y方向
/// @param Kp
/// @param Rpm
/// @param Accel_time
/// @param Move_X_Grab
/// @param Move_Y_Grab
void Follow_bySpeed_ZhuanPan(float Kp, float Rpm, float Accel_time, float Move_X_Grab, float Move_Y_Grab)
{
    int actual_Move_Y_Grab;
    int target_Y_Position = Current_Y + Move_Y_Grab;
    actual_Move_Y_Grab = (MoveSquence[MoveIndex].y > target_Y_Position) ? (MoveSquence[MoveIndex].y - Current_Y) : Move_Y_Grab;
    Follow_bySpeed(Kp, Rpm, Accel_time, Move_X_Grab, actual_Move_Y_Grab);
}

/// @brief 用以跟随移动的位置控制函数，主要用于开环时，即ops失效时
/// @param Kp
/// @param Rpm
/// @param Accel_time
/// @param Move_X_Grab
/// @param Move_Y_Grab
void Follow_by_Position(float Kp, float Rpm, float Accel_time, float Move_X_Grab, float Move_Y_Grab, float Slow_mm)
{
    FollowX_PID.PID_Calc(Move_X_Grab, 0);
    FollowY_PID.PID_Calc(Move_Y_Grab, 0);
    float adaptiveRpm = Rpm;
    if (fabs(Move_X_Grab) < Slow_mm)
    // 必须是逻辑与，因为只往一个方向走时，速度会始终限制在另一个方向的Rpm,并且防止过小引发的反向放大
    {
        adaptiveRpm = max(fabs(Rpm * (Move_X_Grab / Slow_mm)), (float)(5.0));
    }
    MoveCar(FollowX_PID.output, FollowY_PID.output, adaptiveRpm, Accel_time);
}

/// @brief 用于OPS失效下的开环跑点
void RunCar_open_Point()
{
    digitalWrite(LED, 1);
    if (MoveIndex == 0)
        MoveIndex = 1; // 跳过原点的点位
    RunMotors(); // 循环使能电机
    Delta_Rad = (MoveSquence[MovePoint[MoveIndex]].z - CurrentRad);
    if (fabs(Delta_Rad) > M_PI) // 对临界点的偏差进行修正，即180~-180度突变的点
    {
        Error_Rad = (Delta_Rad > 0) ? Delta_Rad - 2 * M_PI : Delta_Rad + 2 * M_PI; // 表示控制量
    }
    else
    {
        Error_Rad = Delta_Rad;
    }

    switch (CarState) // 状态转移,计时放在旋转完与下一次移动之间
    {
    case IDLE:
        if (millis() - lastMoveTime > MoveInterval) // 用于设定定时运动
        {
            if(MovePoint[MoveIndex] == POINT_RAW)
            {
                Global_MoveCar(MoveSquence[MovePoint[MoveIndex]].x - Current_X,
                                MoveSquence[MovePoint[MoveIndex]].y + Grab_Margin - Current_Y,
                                MoveSquence[MovePoint[MoveIndex]].Move_Rpm,
                                MoveSquence[MovePoint[MoveIndex]].Move_Accel_Time);
            }
            else
            {
                Global_MoveCar(MoveSquence[MovePoint[MoveIndex]].x - Current_X,
                                MoveSquence[MovePoint[MoveIndex]].y - Current_Y,
                                MoveSquence[MovePoint[MoveIndex]].Move_Rpm,
                                MoveSquence[MovePoint[MoveIndex]].Move_Accel_Time);
            }
            
        }
        break;
    case Moving:
        if (!stepper1.isRunning() &&
            !stepper2.isRunning() &&
            !stepper3.isRunning() &&
            !stepper4.isRunning())
        {
            CarState = Move_Complete;
        }
        break;
    case Move_Complete:
        Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut); // 表示已经调整到位，重新初始化PID控制器
        if (fabs(Error_Rad) <= MaxError_Rot)
        {
            CarState = Rotate_Complete;
        }
        else
        {
            RotateCar(Error_Rad, MoveSquence[MovePoint[MoveIndex]].Rot_Rpm, MoveSquence[MovePoint[MoveIndex]].Rot_Accel_Time);
        }
        break;
    case Rotating:
        if (!stepper1.isRunning() &&
            !stepper2.isRunning() &&
            !stepper3.isRunning() &&
            !stepper4.isRunning() &&
            fabs(Error_Rad) <= MaxError_Rot)
        {
            CarState = Rotate_Complete;
        }
        else
        {
            RotateCar_toTarget(MoveSquence[MovePoint[MoveIndex]].z, MoveSquence[MovePoint[MoveIndex]].Rot_Rpm, MoveSquence[MovePoint[MoveIndex]].Rot_Accel_Time);
        }
        break;
    case Rotate_Complete:
        Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut); // 表示已经调整到位，重新初始化PID控制器
        
        lastMoveTime = millis();
        break;
    }
}
