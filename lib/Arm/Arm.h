/**
  *****************************************************************************
  * @file               Arm.h
  * @author             邹子睿
  * @version            v1.0
  * @date               2025/7/25
  * @environment        STM32H7 (Arduino Framework)
  * @brief              Header file for functions of Arm
  *****************************************************************************
**/
#ifndef Arm_h
#define Arm_h

#include "FashionStar_UartServo.h"    // Fashion Star串口总线舵机
#include "FashionStar_SmartGripper.h" // Fashion Star智能夹具
#include "TTL_STEPPER.h"              //串口步进电机
#include "AccelStepper.h"             //基座 STEP/DIR 步进电机
#include "Scan.h"
#include "PID.h"
#include "MaixCam.h"

// 转盘处抓取的y方向安全裕量
#define Grab_Margin -15
#define ARM_TO_CAR_CENTER_DIS 90.49968557//机械臂转轴距离整车中心
#define SERVO_BAUDRATE 115200       // 串口舵机波特率115200
#define STEPPER_BAUDRATE 115200     // 串口步进电机波特率115200
// 串口步进电机
// TTL串口通讯控制丝杆步进和齿轮步进，机械臂基座步进
#define Stepper_TX PA2
#define Stepper_RX PA3
#define ARM_BASE_EN_PIN PE10
#define ARM_BASE_DIR_PIN PE15
#define ARM_BASE_STEP_PIN PB11
#define ARM_Stepper_ID 7        // 丝杠升降轴：张大头28步进电机ID 7
#define Gripper_Stepper_ID 6    // 悬臂齿轮齿条轴：张大头28步进电机ID 6
#define STEPPER_BAUDRATE 115200 // 串口步进电机波特率115200
// 串口总线舵机配置
#define SERVO_RX PC7
#define SERVO_TX PC6

#define GRIPPER_SERVO_ID 4  // RA8-U25(H)-M手爪总线舵机ID 4
#define STORAGE_SERVO_ID 5  // 载物盘总线舵机ID 5
#define SERVO_BAUDRATE 115200       // 串口舵机波特率115200

// 步进电机参数设置
#define LUOGAN (12 * 10)            // T6丝杠导程12mm，即每转120个0.1mm
#define CHILUN (36 * 1 * M_PI * 10) // 模数1、36齿，每转行程36*PI mm
#define SYNBELT (10 * 90)           // TTL 基座步进的旧换算参数（当前脉冲基座不使用）

// 基座角度沿用原工程的 0.1 度单位。42 步进为 200 脉冲/圈，16 细分，减速比 5:1。
#define ARM_BASE_MOTOR_STEPS 200.0f
#define ARM_BASE_MICROSTEPS 16.0f
#define ARM_BASE_GEAR_RATIO 5.0f
#define ARM_BASE_PULSES_PER_TENTH_DEGREE \
    (ARM_BASE_MOTOR_STEPS * ARM_BASE_MICROSTEPS * ARM_BASE_GEAR_RATIO / 3600.0f)

// 电机加速度与速度设置
#define ARM_STEPPER_Vel 4000
#define ARM_STEPPER_Acc 255
#define Gripper_STEPPER_Vel 500
#define Gripper_STEPPER_Acc 254
#define ARM_BASE_STEPPER_Vel 600
#define ARM_BASE_STEPPER_Acc 250

#define CirCle_Spacing 1500.0       // 圆间距150mm
extern FSUS_Protocol protocol; // 协议V2版本新增
extern FSUS_Servo storageServo;  // 载物盘舵机
extern FSUS_Servo gripperServo;  // 手爪舵机
// 创建智能机械爪实例
extern FSGP_Gripper gripper;

// 创建步进电机的通信协议对象
extern TTL_Protocol Stepper_protocol;
extern TTL_Stepper armStepper;         // 机械臂竖直自由度步进，12导程，16细分步数
extern TTL_Stepper gripperStepper; // 机械臂水平自由度电机
class PulseBaseStepper
{
public:
    PulseBaseStepper(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin);
    void init();
    void setAngle(float angle);
    void setAngle(float angle, uint16_t vel, uint8_t acc);
    void runToNewPosition(float angle);
    void wait();
    void update_CurrentPos();
    void update_state();

    float currentPosition = 0.0f; // 0.1 度
    bool onPos_state = true;

private:
    void moveToAngle(float angle);
    AccelStepper stepper;
    uint8_t enablePin;
};

extern PulseBaseStepper armBaseStepper;
//             串口步进
extern HardwareSerial Serial_Stepper;
//             串口机械臂步进
//             串口舵机          RX          TX
extern HardwareSerial Serial_SERVO;
// 载物盘舵机角度   0出发位置   1R  2G  3B  不干涉位置，储物盘三个盘位正对机械臂的角度
extern float storage[5];
// 手爪舵机参数设置
extern int GRIPPER_OPEN_ANGLE;     // 爪子张开时的角度
extern int GRIPPER_CLOSE_ANGLE;   // 爪子闭合时的角度
extern int GRIPPER_OPEN_MAX_ANGLE; // 爪子张开最大大角度
// 水平自由度步进电机参数设置
/*机械臂伸出*/
extern float STEPPER_GRIPPER[4]; // ID6近端零点，向外伸出为正，随后为R/G/B位置
extern int STEPPER_GRIPPER_ZHUANPAN_CENTER;
/*实际上机时，机械臂抓夹中心距离转轴的距离0.1mm*/
extern float Arm_Zero_Length; 
// 竖直自由度步进电机参数设置
extern int STEPPER_ZERO;     // ID7高位零点，向下为正
extern int STEPPER_ZHUANPAN; // 下降到转盘抓取单位0.1毫米
extern int STEPPER_GROUND;   // 下降到地面
extern int STEPPER_STORAGE;  // 下降放到载物台
extern int STEPPER_UP;       // 不干涉高点
extern int MATERIAL_HEIGHT;  // 物料高度单位0.1毫米
// 基座步进电机参数设置
extern int ARM_BASE_Stepper_HOME_ANGLE;// 机械臂发车初始角度
extern float ARM_BASE_STEPPER_ANGLE[4]; // 机械臂底部舵机角度{storage,R,G,B}
// 是否准备好抓取
extern bool Ready_to_Grab;
// 夹爪命令状态
extern bool gripper_state;
extern bool Ready_to_Grab;
void Arm_Init();
void Arm_Hardware_Init(bool initializeCamera = true);
void Arm_Base_Only_Init();
void Arm_Startup_Self_Test();
void Arm_Move_Horizontal_To(float targetLength);
void Arm_Move_Vertical_To(float targetHeight);
void Arm_Base_Horizontal_Init();
void Arm_Base_Vertical_Init();
void Storage_SetSlot(int slot);
bool Grab_Recognized_Material_To_Storage(int expectedColor,
                                         bool rotateToPickupHeading = true,
                                         float pickupHeight = -1.0f);
void Grab_ZhuanPan_to_Storage(int color);
void Grab_Storage_to_Place(int stackLevel, bool temporaryArea = false);
void Grab_Ground_to_Storage();
float gripper_reachout(float delta);
void Follow_by_Arm(float deltaX, float deltaY, uint16_t gripperVel, uint16_t rotVel, uint8_t gripperAcc, uint8_t rotAccel);
void update_Color_Place(int Color);
double calculateArmZeroLength(double len1, double angle1, double len2, double angle2, double circleSpacing);
void Calc_Color3_Place();
void Gripper_Move_Direct(float deltaX, float deltaY);
#endif
