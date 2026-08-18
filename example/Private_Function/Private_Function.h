#ifndef __PRIVATE_FUNCTION_H__
#define __PRIVATE_FUNCTION_H__

// 包含必要的头文件
#include <Arduino.h>
#include "OneButton.h"
#include "FashionStar_UartServo.h"
#include "FashionStar_SmartGripper.h"
#include "TTL_STEPPER.h"
#include "MultiStepper.h"
#include <JY901.h>
#include <HardwareTimer.h>
#include <ops9.h>
#include <stdio.h>
#include "AccelStepper.h"
#include "MoveByPosition.h"
#include "PID.h"
#include "MaixCam.h"

// 定义常量
#define InitScale 262.0  // 摄像头初始高度下，像素个数
#define ZeroScale 20.0   // 零高度下像素个数
#define InitHeight 282.0 // 摄像头安装高度
#define ItemHeight 147   // 物料上端离地高度，单位为mm
#define ImageScale 240   // 图像y方向像素个数

#define CirCle_Spacing 1500.0 // 圆间距，单位为mm

#define ARM_STEPPER_Vel 2500 // 机械臂上下速度
#define ARM_STEPPER_Acc 255  // 机械臂上下加速度
#define Gripper_STEPPER_Vel 1500 // 夹爪伸出方向速度
#define Gripper_STEPPER_Acc 254  // 夹爪伸出方向加速度
#define ARM_BASE_STEPPER_Vel 2500 // 机械臂基座旋转速度
#define ARM_BASE_STEPPER_Acc 250  // 机械臂基座旋转加速度

#define STEPPER_ZERO 0              // 上升回原点
#define LUOGAN (12 * 10)            // 12导程
#define CHILUN (36 * 1 * M_PI * 10) // 36齿
#define SYNBELT (10 * 90)           // 基座步进减速比4
// 波特率定义
#define TJCHMI_BAUDRATE 115200
#define QR_BAUDRATE 9600
#define SERVO_BAUDRATE 115200
#define STEPPER_BAUDRATE 115200
// 屏幕
#define TJCHMI_RX PB15
#define TJCHMI_TX PB14
#define DATA_NUM 19 //串口屏参数传递数量
// 扫码
#define QR_RX PE0
#define QR_TX PE1
// 按钮
#define START_BTN PB9
// 串口步进电机
#define Stepper_TX PA2
#define Stepper_RX PA3
#define ArmBaseStepper_Tx PA9
#define ArmBaseStepper_Rx PA10
#define ArmBaseStepper_ID 7
#define ARM_Stepper_ID 5
#define Gripper_Stepper_ID 6
// 串口总线舵机配置
#define SERVO_RX PC7
#define SERVO_TX PC6
#define ARM_BASE_SERVO_ID 0
#define GRIPPER_SERVO_ID 2
#define STORAGE_SERVO_ID 1

// 定义全局变量
extern float Arm_Zero_Length;
extern float Scale;
extern OneButton start_btn;
extern bool start_flag;
extern HardwareSerial Serial_TJCHMI;
extern HardwareSerial Serial_QR;
extern HardwareSerial Serial_Stepper;
extern HardwareSerial Serial_ArmBaseStepper;
extern HardwareSerial Serial_SERVO;
extern FSUS_Protocol protocol;
extern FSUS_Servo storageServo;
extern FSUS_Servo gripperServo;
extern FSGP_Gripper gripper;
extern TTL_Protocol Stepper_protocol;
extern TTL_Stepper armStepper;
extern TTL_Stepper gripperStepper;
extern TTL_Protocol ArmBaseStepper_protocol;
extern TTL_Stepper armBaseStepper;
extern HardwareTimer myTimer1;
extern HardwareTimer myTimer2;
extern const int bufferSize;
extern char receivedData[8];
extern char strQR[];
extern bool scanFlag;
extern int rounds;
extern int dataIndex;
extern int qr_int_str[6];
extern char str[100];
extern bool parameter_ok;
extern bool tm0_En;
extern double Previous_Time, Current_Time;
extern bool Ready_to_Grab;

// 状态机相关枚举
enum RunState;
extern enum RunState run_state;
enum ParaState;
extern enum ParaState Para_state;

// 函数声明
// 一键启动
void start_click();
// 收起机械臂、载物台
void Arm_Init();
// 从转盘抓取color物料，并放置在载物台上，最终机械臂朝载物台方向
void Grab_ZhuanPan_to_Storage(int color);
// 从载物台取出物料，放置在指定位置
void Grab_Storage_to_Place(int n);
// 从地面抓取物料，并放置在载物台上，最终机械臂朝载物台方向
void Grab_Ground_to_Storage();
// 获取GM75模块扫码数据
void QRcode_scanning();
// 扫码获得的字符串转换类型
void return_qr_int();
// 夹爪伸出长度
float gripper_reachout(float delta);
// 单片机更新串口屏幕参数
void Convey_Para_To_TJC();
// 串口屏命令
void TJC_Command();
// 四字节转换为int类型
int Four_Byte_To_Int(unsigned char *ubuffer);
// 存储从串口屏幕获得的参数
void Store_Para_To(unsigned char *ubuffer);
// 串口屏或单片机初始化参数
void Parameter_Data_Init_Function();
// 转盘区移动机器人跟随
void Follow_bySpeed_ZhuanPan(float Kp, float Rpm, float Accel_time, float Move_X_Grab, float Move_Y_Grab);
// 更新视觉转换比例
void Update_Scale(float CurrentHeight, float TargetHeight);
// 移动机器人误差清空
void Car_PID_Init();
// 纯手部跟随
void Follow_by_Arm(float deltaX, float deltaY, uint16_t gripperVel, uint16_t rotVel, uint8_t gripperAcc, uint8_t rotAccel);
// 更新Color物料放置参数
void update_Color_Place(int Color);
// 计算机械臂0位置原始臂长
double calculateArmZeroLength(double len1, double angle1, double len2, double angle2, double circleSpacing);
// 计算RED物料放置位置
void Calc_Color3_Place();
//  夹爪直线移动
void Gripper_Move_Direct(float deltaX, float deltaY);
//定时器2
void UART_PC();
//定时器1，更新视觉和位置参数
void Get_Position();
#endif