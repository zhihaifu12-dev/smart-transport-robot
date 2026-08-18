#ifndef MoveByPosition_h
#define MoveByPosition_h

#include <Arduino.h>
#include "AccelStepper.h"
#include "PID.h"
// [实机必调] 底盘驱动器引脚，必须与主控接线逐项核对。
#define LED PA15
#define enPin PE13 // 共同的使能引脚
#define motorInterfaceType 1
#define dirPin1 PD6
#define stepPin1 PD4
#define dirPin2 PE9
#define stepPin2 PE11
#define dirPin3 PD14
#define stepPin3 PD15
#define dirPin4 PC3_C
#define stepPin4 PA1

// 底盘机械与驱动参数
#define D_Wheel 102.3f                    // [实机已更新] 麦轮有效直径，单位：mm
#define Step_Angle 1.8                    // [实机必调] 电机步距角°，按电机铭牌填写
#define Step_Num (360 / Step_Angle)       // 全步进一圈的步数，即转一圈走多少个步距角
#define MicroStep 32                      // [实机必调] 驱动器细分，必须与拨码/驱动器设置一致
#define Pulse_Num (Step_Num * MicroStep)  // 当前细分步数下，走一圈的脉冲个数
#define LR_WHEELS_DISTANCE 195            // [实机必调] 左右轮接地点中心距离mm，影响旋转角度
#define FR_WHEELS_DISTANCE 185            // [实机必调] 前后轮接地点中心距离mm，影响旋转角度
#define MAX_Rpm 3000                      // [按需调整] 电机/驱动器允许的最大转速
#define C_Wheel (M_PI * D_Wheel)          // 车轮转一圈的移动距离,车轮周长(mm)
#define PULSES_PER_METER 10000.0f          // [实机必调] 来自正方形实测程序的每米脉冲数
#define PulseNum_mm (PULSES_PER_METER / 1000.0f) // 每毫米10脉冲
const float rotationFactor = (LR_WHEELS_DISTANCE / 2.0 + FR_WHEELS_DISTANCE / 2.0) * PulseNum_mm; // [实机必调] 用原地旋转360°实测校准轮距

// 闭环运动相关参数说明
#define MaxError_Rot 0.01 // 用于陀螺仪修正角度的阈值，单位为rad
#define MaxError_Move 2   // 用于修正位移的阈值，单位为mm

#define Range_Of_MAR 200//开始移动和旋转合成运动的误差范围
// 旧版跟随接口仍需该安全余量；机械臂最终任务已迁移到FinalTask。
#define Grab_Margin -15
// [实机必调] 四轮方向。架空测试MoveCar(+X,0)，若某轮反转，只修改对应符号。
#define motor1_CW (-1)
#define motor2_CW (1)
#define motor3_CW (-1)
#define motor4_CW (1)
// 全局坐标，记录当前偏航角，单位为rad与°，统一以逆时针为正，且范围转换为180~-180
extern float CurrentRad;
extern float CurrentYaw; // 陀螺仪相对实测角（度），启动朝向为0，用于屏幕显示
// 全局坐标，记录当前坐标，单位为mm
extern float Current_X;
extern float Current_Y;
// 记录绝对误差
extern float Error_mm;
extern float Error_Rad; // 用于获取当前处理后的角度误差值
extern float Delta_Rad; // 用于获取当前角度偏差值，范围为180~-180°
// 定义小车运动状态
enum CARSTATE
{
    IDLE = 1,
    Moving,
    Move_Complete, // 指平移动作完成，准备进入旋转状态
    Rotating,
    Rotate_Complete
};
extern CARSTATE CarState;
extern bool isRunning; // 电机是否在运动
// 点位结构体定义下（ x , y , z , Rpm , Accel_Time , Rot_Rpm , Rot_Accel_Time ）
typedef struct
{
    float x;
    float y;
    float z;
    float Move_Rpm = 400;
    float Move_Accel_Time = 1;
    float Rot_Rpm = 400;
    float Rot_Accel_Time = 1;
} CAR_GOAL_POINT;

// 2027初赛地图坐标系：左下角为(0,0)，单位mm。
// [实机必调] 区域位置存在现场偏差，以下坐标必须以车体中心可达位置复测。
#define START_ZONE 2             // 启停区选择：1=右上，2=右下
#define MAP_LANE_OFFSET 200.0f   // 外围车道中心距场地边缘的距离mm
#define MAP_CENTER 1200.0f       // 地图中心坐标mm，仅供需要经中心换向的路线使用
#define RAW_ZONE_X 1200.0f       // 原料区实际中心X坐标mm（规则允许随机）
#define RAW_ZONE_Y 2080.0f       // 原料区实际中心Y坐标mm
#define COARSE_ZONE_Y 400.0f     // 粗加工区实际中心Y坐标mm
#define TEMPORARY_ZONE_X (MAP_LANE_OFFSET + 220.0f) // 暂存区实际中心X坐标mm
#define TEMPORARY_ZONE_Y (QR_ZONE_Y - 10.0f)        // 暂存区实际中心Y坐标mm
#define QR_ZONE_Y 1200.0f        // 二维码板实际中心Y坐标mm（规则允许随机）

enum RoutePointId
{
    POINT_START = 0,
    POINT_QR = 1,
    POINT_RAW = 2,
    POINT_COARSE = 3,
    POINT_TEMPORARY = 4,
    POINT_CENTER = 5,
    POINT_RETURN_QR = 6,
    POINT_FINISH = 7,
    // 保留0-7既有编号；以下仅为折线路线的行驶拐点，不执行功能区任务。
    POINT_COARSE_TEMP_CORNER = 8,
    POINT_TEMP_RAW_CORNER = 9
};

extern CAR_GOAL_POINT MoveSquence[];
// 第二轮独立地图。初始参数与第一轮当前实测地图一致，后续可单独调节。
extern CAR_GOAL_POINT MoveSquenceRound2[];
#define Point_Num 8
// 全局指针，用于遍历移动动作
extern int MoveIndex;
extern int MovePoint[];
// 关于PID闭环控制有关的阈值参数，速度计算，ClosedLoop_cal()
extern float ClosedLoop_mm; // 进入PID闭环控制的阈值
extern float Slow_mm;       // 闭环控制中线性减速的阈值
// 用于间断运动
extern unsigned long lastMoveTime;
extern const long MoveInterval;
// 实例化电机
extern AccelStepper stepper1;
extern AccelStepper stepper2;
extern AccelStepper stepper3;
extern AccelStepper stepper4;
// 主要函数
void Motor_Init();
void Motor_ReenableOutputs();
void Motor_Setup(float Rpm, float Accel_Time);
void MoveCar(float X_Distance, float Y_Distance, float Rpm, float Accel_Time);
void Global_MoveCar(float X_Global_Distance, float Y_Global_Distance, float Rpm, float Accel_Time);
void MoveCar_toTarget(float Target_X, float Target_Y, float Rpm, float Accel_Time);
void RotateCar(float Theta, float Rpm, float Accel_Time);
void RotateCar_toTarget(float TargetRad, float Rpm, float Accel_Time);
void RunMotors();
void CloseLoop_Cal();
void RunCar();
void RunMotors_Speed();
void Follow_bySpeed(float Kp, float Rpm, float Accel_time, float Move_X_Grab, float Move_Y_Grab);
void RunCar_open();
void RunToTarget(float Target_X, float Target_Y, float TargetRad,float Rpm, float Accel_Time, int Point);
void RunCar_MaR();
void Follow_bySpeed_ZhuanPan(float Kp, float Rpm, float Accel_time, float Move_X_Grab, float Move_Y_Grab);
void RunCar_MaR_Point();
void Follow_by_Position(float Kp, float Rpm, float Accel_time, float Move_X_Grab, float Move_Y_Grab, float Slow_mm);
void RunCar_open_Point();
void RunCar_MaR_Point_With_Local_PID();
void RunToTarget_With_Local_PID(float Target_X, float Target_Y, float TargetRad,float Rpm, float Accel_Time, int Point);
#endif
