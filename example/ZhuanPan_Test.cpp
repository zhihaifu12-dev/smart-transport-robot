/***主函数，正常使用隐去测试宏定义***/
#include <Arduino.h>
#include "OneButton.h"                //一键启动
#include "FashionStar_UartServo.h"    // Fashion Star串口总线舵机
#include "FashionStar_SmartGripper.h" // Fashion Star智能夹具
#include "TTL_STEPPER.h"              //串口步进电机
#include "MultiStepper.h"
#include <JY901.h>
#include <HardwareTimer.h>
#include <ops9.h>
#include <stdio.h>
#include "AccelStepper.h"
#include "MoveByPosition.h"
#include "PID.h"
#include "MaixCam.h"

//#define ARM_TWO   // 第二套参数
//#define Move_Test // 优先级3
// #define ZanCun_Test   //优先级2
 #define ZhuanPan_Test //优先级1
// #define CuJiaGon_Test //优先级0
#define InitScale 262.0  // 摄像头初始高度下，像素个数
#define ZeroScale 20.0   // 零高度下像素个数
#define InitHeight 282.0 // 摄像头安装高度
#define ItemHeight 147   // 物料上端离地高度，单位为mm
#define ImageScale 240   // 图像y方向像素个数

#define Arm_Zero_Length 1234 // 实际上机时，机械臂抓夹中心距离转轴的距离
// int CameraHeight = InitHeight; //摄像头离地高度，单位为mm，初始为282mm
// int DeltaHeight = CameraHeight;//任意工况下测量平面与当前摄像头的高度差，单位为mm
float Scale = InitHeight / ImageScale; // 任意工况下测量平面单位像素的实际距离

#define PI 3.1415926
#define LUOGAN (12 * 10)          // 12导程
#define CHILUN (36 * 1 * PI * 10) // 36齿
#define SYNBELT (10 * 90)            // 基座步进减速比4
#define TJCHMI_BAUDRATE 115200    // 串口屏幕波特率
#define QR_BAUDRATE 9600          // 串口扫码模块波特率 默认波特率9600
#define SERVO_BAUDRATE 115200     // 串口舵机波特率115200
#define STEPPER_BAUDRATE 115200   // 串口步进电机波特率115200
// 屏幕
#define TJCHMI_RX PB15
#define TJCHMI_TX PB14
#define DATA_NUM 19 // 串口屏需要传输的变量个数
// 扫码
#define QR_RX PE0
#define QR_TX PE1
// 按钮
#define START_BTN PB9                       // v1 PB8 V2 PB9 V3 PB9/PB4根据实际按键引脚修改
OneButton start_btn(START_BTN, true, true); // true:按下为低电平
bool start_flag = 0;
// 串口步进电机
// TTL串口通讯控制丝杆步进和齿轮步进，机械臂基座步进
#define Stepper_TX PA2       
#define Stepper_RX PA3
#define ArmBaseStepper_Tx PA9 
#define ArmBaseStepper_Rx PA10
#define ArmBaseStepper_ID 7  // 机械臂基座步进电机
#define ARM_Stepper_ID 5     // 竖直自由度控制步进
#define Gripper_Stepper_ID 6 // 水平自由度控制步进
// 串口总线舵机配置
#define SERVO_RX PC7
#define SERVO_TX PC6
#define ARM_BASE_SERVO_ID 0 // 舵机0的ID号 基座舵机
#define GRIPPER_SERVO_ID 2  // 舵机4的ID号 手爪
#define STORAGE_SERVO_ID 1  // 舵机1的ID号 载物盘舵机
/*参数设置 */
CAR_GOAL_POINT MoveSquence[] =
    {
        {0, 0, 0, 600, 2, 200, 2},               // 0原点
        {750, 200, 0, 600, 2, 200, 2},           // 1扫码
        {1400, 50, 0, 600, 2, 200, 2},           // 2抓取
        {1020, 200, -M_PI / 2, 400, 2, 200, 2},  // 3路口旋转
        {1020, 1700, -M_PI / 2, 800, 2, 200, 2}, // 4粗加工区
        {1000, 1850, -M_PI, 600, 2, 200, 2},      // 5粗加工中心
        {1850, 1850, M_PI / 2, 600, 2, 200, 2},  // 6拐角1
        {1900, 1020, M_PI / 2, 600, 2, 200, 2},  // 7存储区中心
        {1850, 200, 0, 600, 2, 50, 2},           // 8拐角2
        {0, 0, 0, 600, 2, 200, 2}                // 9原点
}; // 每次移动的距离，基于局部坐标
/*高度*/
#define STEPPER_ZERO  0      // 上升回原点
int STEPPER_ZHUANPAN = 750; // 下降到转盘抓取单位0.1毫米
int STEPPER_GROUND = 1500;  // 下降到地面
int STEPPER_STORAGE = 450;  // 下降放到载物台
int STEPPER_UP = 200;           //不干涉高点
int MATERIAL_HEIGHT = 700;  // 物料高度单位0.1毫米
/*机械臂伸出*/
int STEPPER_GRIPPER[4] = {0, 920, 300, 920}; // 零点，R,G,B
int STEPPER_GRIPPER_ZHUANPAN_CENTER = 800;

#ifndef ARM_TWO
/*爪子*/
int GRIPPER_OPEN_ANGLE = -78;     // 爪子张开时的角度
int GRIPPER_CLOSE_ANGLE = -99;    // 爪子闭合时的角度
int GRIPPER_OPEN_MAX_ANGLE = -61; // 爪子张开最大大角度
/*基座*/
int ARM_BASE_Stepper_HOME_ANGLE = 315;             // 机械臂发车初始角度
int ARM_BASE_STEPPER_ANGLE[4] = {2180, 755, 1215, 1675}; // 机械臂底部舵机角度{storage,R,G,B}
// 载物盘舵机角度   0出发位置   1R  2G  3B  不干涉位置，储物盘三个盘位正对机械臂的角度
int storage[5] = {135, -44, 44 , 132, -44};
#else
/*爪子*/
int GRIPPER_OPEN_ANGLE = -50;     // 爪子张开时的角度
int GRIPPER_CLOSE_ANGLE = -72;    // 爪子闭合时的角度
int GRIPPER_OPEN_MAX_ANGLE = -20; // 爪子张开最大大角度
/*基座*/
int ARM_BASE_Stepper_HOME_ANGLE = 45;                 // 机械臂发车初始角度
int ARM_BASE_Stepper_ANGLE[4] = {-1440, -10, -450, -890}; // 机械臂底部舵机角度{storage,R,G,B}
// 载物盘舵机角度   0出发位置   1R  2G  3B  不干涉位置，储物盘三个盘位正对机械臂的角度
int storage[5] = {70, -113, -25, 63, -113};
#endif
/*点位*/
// 定义全局点位
// 每次移动的距离，基于局部坐标  全局指针MoveIndex
/**串口****************************************************/
//              屏幕模块         RX          TX
HardwareSerial Serial_TJCHMI(TJCHMI_RX, TJCHMI_TX); // 屏幕串口
//             串口扫码模块     RX     TX
HardwareSerial Serial_QR(QR_RX, QR_TX);
//             串口步进
HardwareSerial Serial_Stepper(Stepper_RX, Stepper_TX);
//             串口机械臂步进
HardwareSerial Serial_ArmBaseStepper(ArmBaseStepper_Rx,ArmBaseStepper_Tx);
//             串口舵机          RX          TX
HardwareSerial Serial_SERVO(SERVO_RX, SERVO_TX);
/**舵机***************************************************/
// 创建舵机的通信协议对象
FSUS_Protocol protocol(&Serial_SERVO, SERVO_BAUDRATE); // 协议V2版本新增
FSUS_Servo storageServo(STORAGE_SERVO_ID, &protocol);  // 载物盘舵机
FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &protocol);  // 手爪
// 创建智能机械爪实例
FSGP_Gripper gripper(&gripperServo, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE);

/**步进电机***************************************************/
// 创建步进电机的通信协议对象
TTL_Protocol Stepper_protocol(&Serial_Stepper, STEPPER_BAUDRATE);
TTL_Stepper armStepper(ARM_Stepper_ID, &Stepper_protocol);         // 机械臂竖直自由度步进，12导程，16细分步数
TTL_Stepper gripperStepper(Gripper_Stepper_ID, &Stepper_protocol); // 机械臂水平自由度电机
TTL_Protocol ArmBaseStepper_protocol(&Serial_ArmBaseStepper, STEPPER_BAUDRATE);
TTL_Stepper armBaseStepper(ArmBaseStepper_ID, &ArmBaseStepper_protocol);         // 机械臂旋转自由度步进，4减速比，32细分步数
/*定时器对象*/
// 创建一个HardwareTimer对象，选择使用TIM3，用于获取小车姿态
HardwareTimer myTimer1(TIM3);
// 创建一个HardwareTimer对象，选择使用TIM4，用于定时通讯
HardwareTimer myTimer2(TIM4);
// 扫码模块相关
//  33 32 31 2B 31 32 33 0D 0A
//  GM75默认是CR
//  设置后可改为CRLF
//  CR（Carriage Return），回车符，用符号’\r’表示， 十进制ASCII代码是13，16进制0x0D；
//  LF（Line Feed），换行符，用符号’\n’表示，十进制ASCII代码是10，16进制0x0A；
const int bufferSize = 8;      // 7字节数据 + 1字节回车符+（1字节换行符）
char receivedData[bufferSize]; // 存储接收到的数据
char strQR[] = "000+000";
bool scanFlag = false;
int rounds=0;            // 圈数
int dataIndex = 0;       // 数据索引
int qr_int_str[6] = {0}; // 表示存储的颜色，1R2G3B
char str[100];
bool parameter_ok = false;
bool tm0_En = false;
double Previous_Time, Current_Time;
bool Ready_to_Grab;
// 状态机
enum RunState
{
    Parameter_Data_Init,
    One_button_check, // 一键启动检测
    QR_Scan,
    ZhuanPan,
    CuJiaGon,
    ZanCun,
    Move
};
enum RunState run_state = Parameter_Data_Init;
enum ParaState
{
    Prepare,
    Header,
    Data,
    End,
    Finish_Data,
    Finish_Cmd
};
enum ParaState Para_state = Prepare;
int Data_Index = 0;
unsigned char Data_Buffer[DATA_NUM * 4 + 6];
int num = 1;
int state = 0x00;
bool vision_updated = false;
bool gripper_state = false; // 夹爪命令状态
/***************************Private Function*************************/
// 一键启动
void start_click()
{
    start_flag = 1;
}
// 收起机械臂、载物台
void Arm_Init()
{
    armStepper.runToNewPosition(STEPPER_ZERO);
    armStepper.wait();
    gripperStepper.runToNewPosition(0);
    gripperStepper.wait();
    gripper.close();                                  // 爪子闭合
    storageServo.setAngle(storage[0]);                // 载物盘归位
    armBaseStepper.runToNewPosition(ARM_BASE_Stepper_HOME_ANGLE); // 机械臂归位
    gripper.wait();
    storageServo.wait();
    armBaseStepper.wait();
};
// 从转盘抓取color物料，并放置在载物台上，最终机械臂朝载物台
void Grab_ZhuanPan_to_Storage(int color)
{
    // armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]); // 机械臂朝外
    storageServo.setAngle(storage[color]); // 载物盘旋转
    // gripper.openMax();                              // 机械爪打开
    // armBaseStepper.wait();                            // 等待机械臂旋转到位
    // armStepper.runToNewPosition(STEPPER_ZHUANPAN);  // 机械臂下降到转盘
    // armStepper.wait();                              // 等待机械臂下降到位
    // gripper.close();                                // 机械抓夹紧
    // gripper.wait();                                 // 夹紧到位
    armStepper.runToNewPosition(STEPPER_ZERO);     // 机械臂上升至最高位置
    gripperStepper.runToNewPosition(STEPPER_ZERO); // 机械臂收回减少转动惯量
    armStepper.wait();                             // 上升到位

    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]); // 机械臂朝向载物盘
    gripperStepper.wait();                          // 机械臂等待收回到位
    armBaseStepper.wait();                            // 等待机械臂旋转到位
    storageServo.wait();                            // 等待载物台旋转到位
    armStepper.runToNewPosition(STEPPER_STORAGE);   // 机械臂下降到载物台
    armStepper.wait();                              // 等待下降到位
    gripper.open();                                 // 夹爪打开，内置到位等待
    armStepper.runToNewPosition(STEPPER_ZERO);      // 机械臂上升到最高位置
    armStepper.wait();                              // 上升到位
}
// 从载物台抓取color物料，并放置在地/堆垛物料上表面，最终机械臂朝向载物台
void Grab_Storage_to_Place(int n)
{
    storageServo.setAngle(storage[qr_int_str[dataIndex]]); // 载物盘旋转
    gripper.open();                                        // 爪子打开
    armBaseStepper.wait();                                   // 机械臂旋转到位
    storageServo.wait();                                   // 载物台旋转到位
    armStepper.runToNewPosition(STEPPER_STORAGE);          // 机械臂下降到载物台
    armStepper.wait();                                     // 等待下降到位
    gripper.close();                                       // 夹爪关闭
    gripper.wait();                                        // 夹爪夹紧到位
    armStepper.runToNewPosition(STEPPER_ZERO);             // 机械臂上升
    armStepper.wait();                                     // 上升到位
    if (qr_int_str[dataIndex] == 2)
    {
        storageServo.setAngle(storage[4]); // 载物台旋转朝外，避免齿条干涉***********************
    }
    else
    {
        if (dataIndex != (2 + rounds * 3))
        {
            storageServo.setAngle(storage[qr_int_str[dataIndex + 1]]);
        }
        else
        {
            storageServo.setAngle(storage[qr_int_str[rounds * 3]]);
        }
    }
    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[qr_int_str[dataIndex]]); // 旋转到对应物料放置角度
    if (qr_int_str[dataIndex] == 2)
    {
        storageServo.wait(); // 载物台等待*****************************************
    }
    armBaseStepper.wait();                                                     // 机械臂旋转到位
    gripperStepper.runToNewPosition(STEPPER_GRIPPER[qr_int_str[dataIndex]]); // 夹爪伸出对应长度
    armStepper.runToNewPosition(STEPPER_GROUND - n * MATERIAL_HEIGHT);       // 机械臂下降到地面/堆垛物料上表面
    gripperStepper.wait();                                                   // 夹爪伸出到位
    armStepper.wait();                                                       // 机械臂下降到位
    gripper.open();                                                          // 夹爪打开，内置到位等待

    if (dataIndex == (2 + rounds * 3))
    {
        armStepper.runToNewPosition(STEPPER_ZHUANPAN);
        gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);
        gripperStepper.wait();
        armStepper.wait();
    }
    else
    {
        armStepper.runToNewPosition(STEPPER_ZERO);           // 机械臂上升
        gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]); // 夹爪收回
        armStepper.wait();                                   // 机械臂上升到位
        gripperStepper.wait();                               // 夹爪收回到位
        armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);      // 机械臂朝向载物台
        armBaseStepper.wait();                                 // 机械臂旋转到位
    }
}
// 从地上抓取color物料放置到载物盘
void Grab_Ground_to_Storage()
{
    if (qr_int_str[dataIndex] == 2)
    {
        storageServo.setAngle(storage[4]); // 载物台旋转朝外，避免齿条干涉***********************
    }
    else
    {
        storageServo.setAngle(storage[qr_int_str[dataIndex]]);
    }
    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[qr_int_str[dataIndex]]); // 机械臂朝外
    armBaseStepper.wait();                                                // 等待机械臂旋转到位
    gripper.openMax();                                                  // 机械爪打开
    if (qr_int_str[dataIndex] == 2)
    {
        storageServo.wait(); // 等待载物盘旋转到位*******************
    }
    gripperStepper.runToNewPosition(STEPPER_GRIPPER[qr_int_str[dataIndex]]); // 夹爪伸出对应长度
    armStepper.runToNewPosition(STEPPER_GROUND);                             // 机械臂下降到地面
    gripper.wait();                                                          // 张开到位
    gripperStepper.wait();                                                   // 等待夹爪伸出到位
    armStepper.wait();                                                       // 等待机械臂下降到位
    gripper.close();                                                         // 机械抓夹紧
    gripper.wait();                                                          // 夹爪到位
    armStepper.runToNewPosition(STEPPER_ZERO);                               // 机械臂上升至最高位置
    gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);                     // 夹爪收回
    armStepper.wait();                                                       // 上升到位
    gripperStepper.wait();                                                   // 夹爪收回到位

    if (qr_int_str[dataIndex] == 2)
    {
        storageServo.setAngle(storage[qr_int_str[dataIndex]]);
    }
    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]); // 机械臂朝向载物台
    armBaseStepper.wait();                            // 机械臂旋转到位
    storageServo.wait();                            // 载物台旋转到位
    armStepper.runToNewPosition(STEPPER_STORAGE);   // 机械臂下降
    armStepper.wait();                              // 机械臂下降到位
    gripper.open();                                 // 夹爪打开，自带到位
    armStepper.runToNewPosition(STEPPER_ZERO);      // 机械臂上升到最高点
    armStepper.wait();                              // 等待上升到位
}
// 获取GM75模块扫码数据
void QRcode_scanning()
{
    int i = 0;
    if (scanFlag == false)
    {
        while (Serial_QR.available())
        {
            char incomingByte = Serial_QR.read(); // 读取一个字节数据

            // 检查是否接收到换行符，如果是换行符则重新开始
            if (incomingByte == 0x0A)
            {
                dataIndex = 0; // 重置数据索引
            }
            else
            {
                // 保存字符
                if (dataIndex < (bufferSize - 1))
                {
                    receivedData[dataIndex] = incomingByte; // 将数据存储到数组中
                    dataIndex++;
                }
                if (incomingByte == 0x0D)
                {
                    receivedData[dataIndex] = '\0'; // 在数据末尾添加字符串结束符
                    dataIndex = 0;                  // 重置数据索引
                    scanFlag = true;                // 数据接收成功
                }
            }
        }
    }
}
// 扫码获得字符串转换类型
void return_qr_int()
{
    int i = 0;
    for (i = 0; i < 3; i++)
    {
        switch (receivedData[i])
        { // qr_int_str 123
        case '1':
            qr_int_str[i] = 1;
            break;
        case '2':
            qr_int_str[i] = 2;
            break;
        case '3':
            qr_int_str[i] = 3;
            break;
        }
    }
    for (i = 4; i < 7; i++)
    {
        switch (receivedData[i])
        { // qr_int_str 456
        case '1':
            qr_int_str[i - 1] = 1;
            break;
        case '2':
            qr_int_str[i - 1] = 2;
            break;
        case '3':
            qr_int_str[i - 1] = 3;
            break;
        }
    }
}
// 转盘处伸出长度
/**
 * @brief 计算夹爪步进电机的目标位置，控制电机移动到该位置，并返回机械臂进入死区时的未补偿量
 * @param delta 目标位置的增量
 * @return float 机械臂进入死区时的未补偿量
 */
float 
gripper_reachout(float delta)
{
    gripperStepper.update_CurrentPos(); // 更新夹爪步进电机的当前位置
    float absolute = delta + gripperStepper.currentPosition; // 计算绝对目标位置
    float x = (absolute > 1500) ? 1500 : ((absolute < 0) ? 0 : absolute); // 限位处理
    gripperStepper.runToNewPosition(x); // 控制电机移动到限位后的目标位置
    // 计算并返回机械臂进入死区时的未补偿量
    float Actual_delta = (x-gripperStepper.currentPosition)/10.0;
    return Actual_delta;
}

// 单片机更新串口屏幕参数
void Convey_Para_To_TJC()
{
    sprintf(str, "parameter.n0.val=%d\xff\xff\xff", storage[0]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n1.val=%d\xff\xff\xff", storage[1]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n2.val=%d\xff\xff\xff", storage[2]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n3.val=%d\xff\xff\xff", storage[3]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n4.val=%d\xff\xff\xff", storage[4]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n6.val=%d\xff\xff\xff", STEPPER_STORAGE);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n7.val=%d\xff\xff\xff", STEPPER_ZHUANPAN);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n8.val=%d\xff\xff\xff", STEPPER_GROUND);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n9.val=%d\xff\xff\xff", MATERIAL_HEIGHT);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n11.val=%d\xff\xff\xff", STEPPER_GRIPPER[1]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n12.val=%d\xff\xff\xff", STEPPER_GRIPPER[2]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n13.val=%d\xff\xff\xff", STEPPER_GRIPPER[3]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n14.val=%d\xff\xff\xff", ARM_BASE_STEPPER_ANGLE[0]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n15.val=%d\xff\xff\xff", ARM_BASE_STEPPER_ANGLE[1]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n16.val=%d\xff\xff\xff", ARM_BASE_STEPPER_ANGLE[2]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n17.val=%d\xff\xff\xff", GRIPPER_CLOSE_ANGLE);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n18.val=%d\xff\xff\xff", GRIPPER_OPEN_ANGLE);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n19.val=%d\xff\xff\xff", GRIPPER_OPEN_MAX_ANGLE);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n20.val=%d\xff\xff\xff", ARM_BASE_STEPPER_ANGLE[3]);
    Serial_TJCHMI.print(str);
}
// 串口屏幕命令
void TJC_Command()
{
    while (Serial_TJCHMI.available() >= 5)
    {
        unsigned char ubuffer[6];
        unsigned char frame_header = Serial_TJCHMI.peek(); // 从串口缓冲区读取一个字节但不删除
        if (frame_header == 0xCC)
        { // 为所需帧头时
            Serial_TJCHMI.readBytes(ubuffer, 5);
            if (ubuffer[4] == 0xFF)
            {
                int data = (int)((ubuffer[3] << 8) | ubuffer[2]);
                if (ubuffer[3] & 0x80)
                {                       // 检查高位字节的最高位
                    data |= 0xFFFF0000; // 对32位int进行符号扩展
                }
                switch (ubuffer[1])
                {
                case 0x00:
                    storageServo.setAngle(data);
                    storage[0] = data;
                    break;
                case 0x01:
                    storageServo.setAngle(data);
                    storage[1] = data;
                    break;
                case 0x02:
                    storageServo.setAngle(data);
                    storage[2] = data;
                    break;
                case 0x03:
                    storageServo.setAngle(data);
                    storage[3] = data;
                    break;
                case 0x04:
                    storageServo.setAngle(data);
                    storage[4] = data;
                    break;
                case 0x10:
                    armStepper.runToNewPosition(STEPPER_ZERO);
                    break;
                case 0x11:
                    armStepper.runToNewPosition(data);
                    STEPPER_STORAGE = data;
                    break;
                case 0x12:
                    armStepper.runToNewPosition(data);
                    STEPPER_ZHUANPAN = data;
                    break;
                case 0x13:
                    armStepper.runToNewPosition(data);
                    STEPPER_GROUND = data;
                    break;
                case 0x14:
                    armStepper.runToNewPosition(STEPPER_GROUND - data);
                    MATERIAL_HEIGHT = data;
                    break;
                case 0x20:
                    gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);
                    break;
                case 0x21:
                    gripperStepper.runToNewPosition(data);
                    STEPPER_GRIPPER[1] = data;
                    break;
                case 0x22:
                    gripperStepper.runToNewPosition(data);
                    STEPPER_GRIPPER[2] = data;
                    break;
                case 0x23:
                    gripperStepper.runToNewPosition(data);
                    STEPPER_GRIPPER[3] = data;
                    break;
                case 0x30:
                    armBaseStepper.setAngle(data);
                    ARM_BASE_STEPPER_ANGLE[0] = data;
                    break;
                case 0x31:
                    armBaseStepper.setAngle(data);
                    ARM_BASE_STEPPER_ANGLE[1] = data;
                    break;
                case 0x32:
                    armBaseStepper.setAngle(data);
                    ARM_BASE_STEPPER_ANGLE[2] = data;
                    break;
                case 0x33:
                    armBaseStepper.setAngle(data);
                    ARM_BASE_STEPPER_ANGLE[3] = data;
                    break;
                case 0x40:
                    gripperServo.setAngle(data);
                    GRIPPER_CLOSE_ANGLE = data;
                    break;
                case 0x41:
                    gripperServo.setAngle(data);
                    GRIPPER_OPEN_ANGLE = data;
                    break;
                case 0x42:
                    gripperServo.setAngle(data);
                    GRIPPER_OPEN_MAX_ANGLE = data;
                    break;
                default:
                    break;
                }
            }
        }
        else if (frame_header == 0xDD)
        {
            Serial_TJCHMI.readBytes(ubuffer, 6);
            if (ubuffer[1] == 0xDD && ubuffer[2] == 0xDD && ubuffer[3] == 0xFF && ubuffer[4] == 0xFF && ubuffer[5] == 0xFF)
            {
                Convey_Para_To_TJC();
            }
        }
        else
        {
            Serial_TJCHMI.read();
        }
    }
}
// 四字节数据转换成Int类型
int Four_Byte_To_Int(unsigned char *ubuffer)
{
    return (int)((ubuffer[3] << 24) | (ubuffer[2] << 16) | (ubuffer[1] << 8) | ubuffer[0]);
}
// 存储从串口屏幕获得的参数
void Store_Para_To(unsigned char *ubuffer)
{
    storage[0] = Four_Byte_To_Int(ubuffer);
    storage[1] = Four_Byte_To_Int(ubuffer + 4);
    storage[2] = Four_Byte_To_Int(ubuffer + 8);
    storage[3] = Four_Byte_To_Int(ubuffer + 12);
    storage[4] = Four_Byte_To_Int(ubuffer + 16);
    STEPPER_STORAGE = Four_Byte_To_Int(ubuffer + 20);
    STEPPER_ZHUANPAN = Four_Byte_To_Int(ubuffer + 24);
    STEPPER_GROUND = Four_Byte_To_Int(ubuffer + 28);
    MATERIAL_HEIGHT = Four_Byte_To_Int(ubuffer + 32);
    STEPPER_GRIPPER[1] = Four_Byte_To_Int(ubuffer + 36);
    STEPPER_GRIPPER[2] = Four_Byte_To_Int(ubuffer + 40);
    STEPPER_GRIPPER[3] = Four_Byte_To_Int(ubuffer + 44);
    ARM_BASE_STEPPER_ANGLE[0] = Four_Byte_To_Int(ubuffer + 48);
    ARM_BASE_STEPPER_ANGLE[1] = Four_Byte_To_Int(ubuffer + 52);
    ARM_BASE_STEPPER_ANGLE[2] = Four_Byte_To_Int(ubuffer + 56);
    ARM_BASE_STEPPER_ANGLE[3] = Four_Byte_To_Int(ubuffer + 72);
    GRIPPER_CLOSE_ANGLE = Four_Byte_To_Int(ubuffer + 60);
    GRIPPER_OPEN_ANGLE = Four_Byte_To_Int(ubuffer + 64);
    GRIPPER_OPEN_MAX_ANGLE = Four_Byte_To_Int(ubuffer + 68);
}
//  串口屏与单片机参数初始化
void Parameter_Data_Init_Function()
{
    while (Serial_TJCHMI.available())
    {
        unsigned char b = Serial_TJCHMI.read();
        switch (Para_state)
        {
        case Prepare:
            sprintf(str, "start.t0.txt=\"Prepare\"\xff\xff\xff");
            Serial_TJCHMI.print(str);
            if (!tm0_En && !parameter_ok)
            {
                sprintf(str, "start.tm0.en=1\xff\xff\xff");
                tm0_En = true;
                Serial_TJCHMI.print(str);
                Previous_Time = millis(); // 开始传输的时间
            }
            Data_Index = 0;
            Para_state = Header; // 无需break，直接进入下一状态
        case Header:
            Data_Buffer[Data_Index++] = b;
            if (Data_Index == 3)
            {
                if (Data_Buffer[0] == 0xAB && Data_Buffer[1] == 0xCC && Data_Buffer[2] == 0xDD)
                {
                    Para_state = Data;
                }
                else if (Data_Buffer[0] == 0xDD && Data_Buffer[1] == 0xDD && Data_Buffer[2] == 0xDD)
                {
                    Para_state = End;
                }
                else
                {
                    // 滑动数据，保留第二、三个字节作为帧头候选
                    Data_Buffer[0] = Data_Buffer[1];
                    Data_Buffer[1] = Data_Buffer[2];
                    Data_Index = 2;
                }
            }
            break;
        case Data:
            // 接收数据Data_Index 3~Data_NUM*4+2
            Data_Buffer[Data_Index++] = b;
            if (Data_Index == (DATA_NUM * 4 + 3))
            {
                Para_state = End;
            }
            break;
        case End:
            // 帧尾 Data_Index 2或者 Data_NUM*4+3~Data_NUM*4+5
            Data_Buffer[Data_Index] = b;
            if (Data_Index <= 5)
            {
                if (Data_Index == 5)
                {
                    if (Data_Buffer[Data_Index - 2] == 0xFF && Data_Buffer[Data_Index - 1] == 0xFF && Data_Buffer[Data_Index] == 0xFF)
                    {
                        Para_state = Finish_Cmd;
                    }
                    else
                    {
                        Para_state = Prepare;
                    }
                }
                else
                {
                    Data_Index++;
                }
            }
            else if (Data_Index < (DATA_NUM * 4 + 5))
            {
                Data_Index++;
            }
            else
            {
                if (Data_Buffer[Data_Index - 2] == 0xDD && Data_Buffer[Data_Index - 1] == 0xCC && Data_Buffer[Data_Index] == 0xFF)
                {
                    Para_state = Finish_Data;
                }
                else
                {
                    Para_state = Prepare;
                }
            }
            break;
        case Finish_Data:
            sprintf(str, "start.tm0.en=0\xff\xff\xff");
            Serial_TJCHMI.print(str);
            tm0_En = false;
            Store_Para_To(Data_Buffer + 3);
            parameter_ok = true;
            Convey_Para_To_TJC(); // 保证串口屏显示的即为程序内运行的
            break;
        case Finish_Cmd:
            sprintf(str, "start.tm0.en=0\xff\xff\xff");
            Serial_TJCHMI.print(str);
            tm0_En = false;
            Convey_Para_To_TJC();
            parameter_ok = true;
            break;
        default:
            break;
        }
        Current_Time = millis(); // 当前的时间
    }
}
// 转盘区移动机器人跟随
void Follow_bySpeed_ZhuanPan(float Kp, float Rpm, float Accel_time, float Move_X_Grab, float Move_Y_Grab)
{
    int actual_Move_Y_Grab;
    int target_Y_Position = Current_Y + Move_Y_Grab;
    actual_Move_Y_Grab = (MoveSquence[MoveIndex].y > target_Y_Position) ? (MoveSquence[MoveIndex].y - Current_Y) : Move_Y_Grab;
    Follow_bySpeed(Kp, Rpm, Accel_time, Move_X_Grab, actual_Move_Y_Grab);
}
// 更新视觉比例系数
void Update_Scale(float CurrentHeight, float TargetHeight)
{
    float CameraHeight = InitHeight - CurrentHeight;
    float DeltaHeight = CameraHeight - TargetHeight;
    Scale = ((InitScale - ZeroScale) / InitHeight * DeltaHeight + ZeroScale) / ImageScale; // 当前工况下
}
//移动机器人pid参数归零
void Car_PID_Init(){
    MoveX_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
    MoveY_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
    Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut);
}
// // 纯手部校准（开发中）
// void Follow_by_Arm(float deltaX, float deltaY)
// {
//     gripperStepper.Calculate_CurrentPos();
//     float Target_Vector_X = gripperStepper.currentPosition - deltaY; // 步进向外为正，视觉向内为正
//     float Target_Vector_Y = -deltaX;                                 // 舵机顺时针为正，视觉逆时针切向为正
//     float GirpperStepper_Target = Target_Vector_X;
//     float Delat_Theta = Target_Vector_Y / Target_Vector_X * 360 / PI; // arctan使用近似
//     float GripperStepper_Target = deltaX + gripperStepper.currentPosition;
//     GripperStepper_Target = (GripperStepper_Target > 1500) ? 1500 : ((GripperStepper_Target < 0) ? 0 : GripperStepper_Target); // 限位
//     float ArmBaseServo_Target = 1;
// }
/***************************Private Function*************************/
/***************************定时器***********************************/
void UART_PC()
{
    // 使用中断通讯
    if (Serial_TJCHMI.availableForWrite()) // 获得待写的字节数
    {
        // sprintf(str,"Run.t0.txt=\"X%dY%d\"\xff\xff\xff",Move_X_Grab,Move_Y_Grab);
#ifdef ZhuanPan_Test
        if (run_state == ZhuanPan || run_state == CuJiaGon)
        {
            Serial_TJCHMI.print(str);
        }
#endif
#ifdef CuJiaGon_Test
#endif
        // Serial_TJCHMI.print("Current_X =");
        // Serial_TJCHMI.print(Current_X,3);
        // Serial_TJCHMI.print("Current_Y =");
        // Serial_TJCHMI.print(Current_Y,3);
        // Serial_TJCHMI.print("OPS0Yaw =");
        // Serial_TJCHMI.println(,3);
        // Serial_TJCHMI.print("CurrentYaw =");
        // Serial_TJCHMI.println(CurrentYaw,3);
        // Serial_TJCHMI.print("pidx =");
        // Serial_TJCHMI.print(MoveX_PID.output,3);
        // Serial_TJCHMI.print("pidy =");
        // Serial_TJCHMI.println(MoveY_PID.output,3);
    }
}
void Get_Position()
{
    // 如果IMU只在发送数据时打开串口，即200hz，则可以用while，不会阻塞程序
    // 用的是定时器中断，因此需要将定时器频率设置大于IMU
    // 如果会一直打开串口，则可以用if，接收频率等于IMU发送频率
    while (Serial_WTIMU.available())
    {
        JY901.CopeSerialData(Serial_WTIMU.read()); // Call JY901 data cope function
        // 储存当前的偏航角
        CurrentRad = (float)JY901.stcAngle.Angle[2] / 32768 * M_PI;
        CurrentYaw = (float)JY901.stcAngle.Angle[2] / 32768 * 180;
    }
    // OPS9数据接收
    while (Serial_OPS9.available())
    {
        OPS9.readData(Serial_OPS9.read());
        Current_X = OPS9.pos_y;
        Current_Y = -OPS9.pos_x;
    }
    while (Serial_Maix.available())
    {
        MaixCam.Maix_ReadData(Serial_Maix.read());
        Move_X_Grab = MaixCam.Delta_X * Scale; // 相对位移，以车为坐标系
        Move_Y_Grab = MaixCam.Delta_Y * Scale;
        Current_Color = MaixCam.Color;
        vision_updated = true; // 标记视觉数据已更新
    }
}
/*****************************************************************/
void setup()
{
    delay(1000); // 等待各外设上电
    // 串口屏幕
    Serial_TJCHMI.begin(TJCHMI_BAUDRATE);
    sprintf(str, "rest\xff\xff\xff");
    Serial_TJCHMI.print(str); // 串口屏重启
    MaixCam.Maix_Init();
    // 电机复位操作
    Motor_Init();
    // IMU初始化
    IMU_Init();
    delay(100);
    // OPS9初始化
    Serial_OPS9.begin(OPS9_BAUDRATE);
    OPS9.Update_X(0);
    delay(10);
    OPS9.Update_Y(0);
    delay(10);
    // 配置定时器为1000Hz（1ms周期）IMU的回传频率最高为1000Hz
    myTimer1.setOverflow(1000, HERTZ_FORMAT);
    myTimer1.attachInterrupt(Get_Position); // 附加中断回调
    myTimer1.setInterruptPriority(1, 0);    // 设置中断优先级（可选）（抢占，响应）
    myTimer1.resume();
    // 配置定时器为5Hz（200ms周期）
    myTimer2.setOverflow(1, HERTZ_FORMAT);
    myTimer2.attachInterrupt(UART_PC);   // 附加中断回调
    myTimer2.setInterruptPriority(2, 0); // 设置中断优先级（可选）
    myTimer2.resume();
    // 串口步进初始化
    Stepper_protocol.init(&Serial_Stepper, STEPPER_BAUDRATE); // 电机通信协议初始
    armStepper.init();
    gripperStepper.init();
    armStepper.set(2500, 255, 0, LUOGAN, 16);
    gripperStepper.set(150, 254, 0, CHILUN, 256);

    ArmBaseStepper_protocol.init(&Serial_ArmBaseStepper,STEPPER_BAUDRATE);
    armBaseStepper.init();
    armBaseStepper.set(250,250,1, SYNBELT,32);
    // 舵机初始化
    protocol.init(&Serial_SERVO, SERVO_BAUDRATE); // 舵机通信协议初始化
    storageServo.init();                          // 储物盘舵机初始化
    gripper.init();                               // 手爪舵机初始化，原始程序爪子会开启
    gripper.setMaxPower(700);                     // 设置最大功率，单位mW
    storageServo.setSpeed(400);                   // 舵机1初始化速度 储物盘
    // 一键启动按钮初始化
    start_btn.reset(); // 清除一下按钮状态机的状态
    start_btn.attachClick(start_click);
    // GM75模块
    Serial_QR.begin(QR_BAUDRATE);
    // PID初始化设置
    Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut);
    MoveX_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
    MoveY_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
    ARM_PID.PID_Init(ARM_PID_Kp, ARM_PID_Ki, ARM_PID_Kd, ARM_PID_MItg, ARM_PID_MOut);
    // 注步进电机上电自动回零
    Arm_Init(); // 舵机、步进收起
    while (Serial_TJCHMI.read())
        ; // 读空
    Previous_Time = millis();
}

void loop()
{
    switch (run_state)
    {
    case Parameter_Data_Init:
        Parameter_Data_Init_Function();
        if (parameter_ok)
        {
            while (Serial_TJCHMI.read() > 0)
                ; // 清空缓冲区域
            // 初始化完成提示在串口屏幕上
            sprintf(str, "start.t0.txt=\"OK\"\xff\xff\xff");
            Serial_TJCHMI.print(str);
            run_state = One_button_check;
        }
        else if (Current_Time - Previous_Time >= 1000)
        {
            sprintf(str, "start.tm0.en=0\xff\xff\xff");
            Serial_TJCHMI.print(str);
            tm0_En = false;
            Convey_Para_To_TJC(); // 使用默认数据
            parameter_ok = true;
            while (Serial_TJCHMI.read() > 0)
                ; // 清空缓冲区域
            // 初始化完成提示在串口屏幕上
            sprintf(str, "start.t0.txt=\"OK\"\xff\xff\xff");
            Serial_TJCHMI.print(str);
            run_state = One_button_check;
        }
        break;
    case One_button_check:
        start_btn.tick();
        if (start_flag)
        {
            sprintf(str, "page Run\xff\xff\xff"); // 转到Run界面
            Serial_TJCHMI.print(str);
            run_state = Move;
            MoveIndex++;
            CarState = IDLE;
#ifdef ZhuanPan_Test
            run_state = ZhuanPan;
            qr_int_str[0] = 3;
            qr_int_str[1] = 1;
            qr_int_str[2] = 2;
            MoveIndex = 0;
#endif
#ifdef CuJiaGon_Test
            run_state = CuJiaGon;
            qr_int_str[0] = 1;
            qr_int_str[1] = 2;
            qr_int_str[2] = 3;
            MoveIndex = 0;
#endif
        }
        else
        {
            TJC_Command();
        }
        break;
    case QR_Scan:
        QRcode_scanning();
        if (scanFlag)
        {
            return_qr_int(); // 获取任务顺序
            sprintf(str, "Run.t0.txt=\"%s\"\xff\xff\xff", receivedData);
            Serial_TJCHMI.print(str); // 显示顺序在串口屏幕上

            armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]);
            storageServo.setAngle(storage[qr_int_str[0]]);
            gripper.openMax();
            MaixCam.Maix_Follow(qr_int_str[rounds * 3]);

            run_state = Move;
            MoveIndex++;
            CarState = IDLE;
        }
        break;
    case ZhuanPan:
        dataIndex = rounds * 3;
        state = 0x00;
        MaixCam.Maix_Follow(qr_int_str[dataIndex]);
        ARM_PID.PID_Init();
        while (dataIndex <= (2 + rounds * 3))
        {
#ifdef ZhuanPan_Test
            // sprintf(str, "Run.t0.txt=\"%d\"\xff\xff\xff", num);
#endif
            bool isRunning = (stepper1.isRunning() ||
                              stepper2.isRunning() ||
                              stepper3.isRunning() ||
                              stepper4.isRunning());
            switch (state)
            {
            case 0x00:
                Update_Scale(STEPPER_ZERO, ItemHeight);
                armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]);        // 机械臂朝外
                storageServo.setAngle(storage[qr_int_str[dataIndex]]); // 载物台旋转
                gripper.openMax();
                gripper_state = false;
                gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);
                state = 0x11;
                break;
            case 0x11:
                if (vision_updated && MaixCam.Color == qr_int_str[dataIndex])
                {
                    gripper_reachout(-Move_Y_Grab * 10);
                    state = 0x22;
                    vision_updated = false;
                    Ready_to_Grab = false;
                }
                break;
            case 0x22:
                RunMotors_Speed();
                /***********更新步进到位状态**********/
                if (!armStepper.onPos_state)
                {
                    armStepper.update_state(); // 更新到位
                }
                /*****更新视觉转换比例Scale****************/
                if (!armStepper.onPos_state)
                {
                    armStepper.update_CurrentPos();//直接更新更好
                    Update_Scale(armStepper.currentPosition / 10, ItemHeight);
                }
                else if (Ready_to_Grab)
                {
                    Update_Scale(STEPPER_ZHUANPAN / 10, ItemHeight);
                }
                else
                {
                    Update_Scale(STEPPER_ZERO / 10, ItemHeight);
                }
                /**********************判断物料位置******************/
                if (vision_updated && MaixCam.Color == qr_int_str[dataIndex])
                {
                    /**********************向车身偏置夹爪位置*******************/
                    if (Ready_to_Grab && armStepper.onPos_state)
                    {
                        Move_Y_Grab += 0;
                    }
                    else
                    {
                        Move_Y_Grab += 10;
                    }
                    /*********************************************************/
                    if (fabs(Move_X_Grab) < 20 && Move_Y_Grab < 5 && !gripper_state && (!Ready_to_Grab||!armStepper.onPos_state))//未夹持时（上方和下降中）允许物料的范围
                    {
                        /*机械臂下降*/
                        if (!Ready_to_Grab)
                        {
                            armStepper.runToNewPosition(STEPPER_ZHUANPAN);
                            armStepper.onPos_state = false;
                        }
                        Ready_to_Grab = true;
                        Current_Time = millis();
                    }else if(Ready_to_Grab && armStepper.onPos_state  && fabs(Move_X_Grab) < 20 && Move_Y_Grab > -5){//下降完成（包含夹持时）后阈值范围需要变化
                        Current_Time = millis();
                    }
                    else
                    {
                        if (gripper_state && fabs(Move_Y_Grab) > 5)
                        { // 判断抓取是否成功,检测未成功则进行下列判断
                            if (gripper.servo->isStop())
                            { // 判断夹爪到位
                                gripper.openMax();
                                gripper_state = false;
                                Previous_Time = millis();
                            }
                        }
                        else if (!gripper_state)
                        {
                            Previous_Time = millis();
                        }
                    }
                    /**********抓取*************/
                    if (Ready_to_Grab && armStepper.onPos_state && Current_Time - Previous_Time > 50)
                    {
                        if (!gripper_state)
                        {
                            Previous_Time=millis();
                            gripper.close();
                            gripper_state = true;
                        }
                        else
                        {
                            if (gripper.servo->isStop())
                            { // 查询舵机是否停止
                                state = 0x33;
                                break;
                            }
                            else if (gripper.isFrozen())
                            {
                                gripper.openMax();
                                gripper_state = false;
                            }
                        }
                    }
                    ARM_PID.PID_Calc(0, Move_Y_Grab * 10 * 0.8);
                    float gripper_delta=gripper_reachout(ARM_PID.output);
                    Follow_bySpeed_ZhuanPan(1.5, 30, 5, Move_X_Grab, Move_Y_Grab+gripper_delta);//gripper_delta正方向与Move_Y_Grab相反
                    vision_updated = false;
                }
                else if (vision_updated && MaixCam.Color != qr_int_str[dataIndex])
                { // 如果校准过程中丢失物料
                    gripper.openMax();
                    gripper_state = false;
                    armStepper.runToNewPosition(STEPPER_ZERO);
                    gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);
                    Ready_to_Grab = false;
                    state = 0x44;
                    CarState = IDLE;
                    vision_updated = false;
                }
                break;
            case 0x33:
                ARM_PID.PID_Init();
                if (dataIndex != (2 + rounds * 3))
                {
                    MaixCam.Maix_Follow(qr_int_str[dataIndex + 1]);
                }
                Grab_ZhuanPan_to_Storage(qr_int_str[dataIndex]);
                dataIndex++;
                if (dataIndex != (3 + rounds * 3))
                {
                    CarState = IDLE;
                    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]); // 机械臂朝外
                    storageServo.setAngle(qr_int_str[dataIndex]);   // 载物台旋转
                    gripper.openMax();
                    gripper_state = false;
                    gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);
                    state = 0x44;
                }
                break;
            case 0x44:
                RunCar_MaR(); // 回正位置
                if (CarState == Rotate_Complete)
                {
                    state = 0x00;
                }
                else if (vision_updated && MaixCam.Color == qr_int_str[dataIndex])
                {
                    state = 0x11;
                    CarState = Rotate_Complete;
                    Car_PID_Init();
                }
                break;
            default:
                break;
            }
        }
#ifdef ZhuanPan_Test
        Arm_Init();
        CarState = IDLE;
        while (CarState != Rotate_Complete)
        {
            RunCar_MaR();
        }
        run_state = One_button_check;
        dataIndex = 0;
        break;
#endif
        run_state = Move;
        MoveIndex++;
        storageServo.setAngle(storage[0]);
        MaixCam.Maix_Init();
        CarState = IDLE;
        break;
    case CuJiaGon:
        dataIndex = rounds * 3;
        MaixCam.Maix_Detect(Green);
        state = 0x00;
        while (dataIndex <= (2 + rounds * 3))
        {
#ifdef CuJiaGon_Test
            sprintf(str, "Run.t0.txt=\"%d\"\xff\xff\xff", state);
#endif
            switch (state)
            {
            case 0x00:
                Update_Scale(STEPPER_ZERO / 10, 0);
                armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]); // 机械臂朝外
                storageServo.setAngle(storage[4]);              // 载物台旋转
                gripper.openMax();
                gripperStepper.runToNewPosition(STEPPER_GRIPPER[Green]);
                gripperStepper.wait();
                armBaseStepper.wait();
                state = 0x11;
                break;
            case 0x11:
                if (vision_updated)
                {
                    state = 0x22;
                    vision_updated = false;
                    Ready_to_Grab = false; // 未命令下降
                }
                break;
            case 0x22:
                /***********更新步进到位状态**********/
                if (!armStepper.onPos_state)
                {
                    armStepper.update_state(); // 更新到位
                }
                RunMotors_Speed();
                if (vision_updated && MaixCam.Color == Green)
                {
                    if (!Ready_to_Grab)
                    {
                        if (storageServo.isStop())
                        {
                            armStepper.runToNewPosition(STEPPER_GROUND);
                            armStepper.onPos_state = false;
                            Ready_to_Grab = true;
                        }
                    }
                    if (fabs(Move_X_Grab) < 2 && fabs(Move_Y_Grab) < 2)
                    {
                        Current_Time = millis();
                    }
                    else
                    {
                        Previous_Time = millis();
                    }
                    /**********识别准确*************/
                    if (Ready_to_Grab && armStepper.onPos_state && Current_Time - Previous_Time > 50)
                    {
                        state = 0x33;
                        armStepper.runToNewPosition(STEPPER_ZERO);
                        gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);
                        armStepper.wait();

                        gripperStepper.wait();
                        armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);
                        break;
                    }
                    /*****更新视觉转换比例Scale****************/
                    if (!armStepper.onPos_state)
                    {
                        armStepper.Calculate_CurrentPos();
                        Update_Scale(armStepper.currentPosition / 10, 0);
                    }
                    else if (Ready_to_Grab)
                    {
                        Update_Scale(STEPPER_GROUND / 10, 0);
                    }
                    else
                    {
                        Update_Scale(STEPPER_ZERO / 10, 0);
                    }
                    Follow_bySpeed(1.5, 30, 5, Move_X_Grab, Move_Y_Grab);
                    vision_updated = false;
                }
                break;
            case 0x33:
                Grab_Storage_to_Place(0);
                dataIndex++;
                break;
            default:
                break;
            }
        }
        dataIndex = rounds * 3;
        while (dataIndex <= (2 + rounds * 3))
        {
            Grab_Ground_to_Storage();
            dataIndex++;
        }
#ifdef ZhuanPan_Test
        Arm_Init();
        CarState = IDLE;
        while (CarState != Rotate_Complete)
        {
            RunCar_MaR();
        }
        run_state = One_button_check;
        dataIndex = 0;
        break;
#endif
        run_state = Move;
        MoveIndex++;
        CarState = IDLE;
        storageServo.setAngle(storage[0]);
        MaixCam.Maix_Init();
        break;
    case ZanCun:
        dataIndex = rounds * 3;
        MaixCam.Maix_Detect(Green);
        state = 0x00;
        while (dataIndex <= (2 + rounds * 3))
        {
#ifdef ZanCun_Test
            sprintf(str, "Run.t0.txt=\"%d\"\xff\xff\xff", state);
#endif
            switch (state)
            {
            case 0x00:
                Update_Scale(STEPPER_ZERO / 10, 0);
                armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]); // 机械臂朝外
                if (rounds)
                {
                    storageServo.setAngle(storage[qr_int_str[dataIndex]]);
                }
                else
                {
                    storageServo.setAngle(storage[4]); // 载物台旋转
                }
                gripper.openMax();
                gripperStepper.runToNewPosition(STEPPER_GRIPPER[Green]);
                gripperStepper.wait();
                armBaseStepper.wait();
                state = 0x11;
                break;
            case 0x11:
                if (vision_updated)
                {
                    state = 0x22;
                    vision_updated = false;
                    Ready_to_Grab = false; // 未命令下降
                }
                break;
            case 0x22:
                /***********更新步进到位状态**********/
                if (!armStepper.onPos_state)
                {
                    armStepper.update_state(); // 更新到位
                }
                RunMotors_Speed();
                if (vision_updated && MaixCam.Color == Green)
                {
                    if (!Ready_to_Grab)
                    {
                        if (storageServo.isStop())
                        {
                            if (!rounds)
                            {
                                armStepper.runToNewPosition(STEPPER_GROUND);
                            }
                            armStepper.onPos_state = false;
                            Ready_to_Grab = true;
                        }
                    }
                    if (fabs(Move_X_Grab) < 2 && fabs(Move_Y_Grab) < 2)
                    {
                        Current_Time = millis();
                    }
                    else
                    {
                        Previous_Time = millis();
                    }
                    /**********识别准确*************/
                    if (Ready_to_Grab && armStepper.onPos_state && Current_Time - Previous_Time > 50)
                    {
                        state = 0x33;
                        armStepper.runToNewPosition(STEPPER_ZERO);
                        gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);
                        armStepper.wait();

                        gripperStepper.wait();
                        armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);
                        break;
                    }
                    /*****更新视觉转换比例Scale****************/
                    if (!armStepper.onPos_state)
                    {
                        armStepper.Calculate_CurrentPos();
                        Update_Scale(armStepper.currentPosition / 10, 0);
                    }
                    else if (Ready_to_Grab)
                    {
                        Update_Scale(STEPPER_GROUND / 10, 0);
                    }
                    else
                    {
                        Update_Scale(STEPPER_ZERO / 10, 0);
                    }
                    Follow_bySpeed(1.5, 30, 5, Move_X_Grab, Move_Y_Grab);
                    vision_updated = false;
                }
                break;
            case 0x33:
                Grab_Storage_to_Place(rounds);
                dataIndex++;
                break;
            default:
                break;
            }
        }
        run_state = Move;
        MoveIndex++;
        MaixCam.Maix_Init();
        armStepper.runToNewPosition(STEPPER_ZERO);
        storageServo.setAngle(qr_int_str[3]);
        CarState = IDLE;
        break;
    case Move:
        RunCar_MaR();
#ifdef Move_Test
        if (CarState == Rotate_Complete)
        {
            switch (MoveIndex)
            {
            case 8:
                if (rounds == 0)
                {
                    MoveIndex = 2;
                    rounds++;
                }
                else
                {
                    // MoveIndex++;
                }
                break;
            case 9:
                run_state = One_button_check;
                MoveIndex = 0;
                start_flag = 0;
                break;
            default:
                MoveIndex++;
                break;
            }
            // CarState=IDLE;
        }
        break;
#endif
        if (CarState == Rotate_Complete)
        {
            switch (MoveIndex)
            {
            case 1:
                run_state = QR_Scan;
                break;
            case 2:
                run_state = ZhuanPan;
                break;
            case 5:
                run_state = CuJiaGon;
                break;
            case 7:
                run_state = ZanCun;
                break;
            case 8:
                if (rounds == 0)
                {
                    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]);
                    storageServo.setAngle(storage[qr_int_str[rounds * 3]]);
                    MoveIndex = 2;
                    rounds++;
                    MaixCam.Maix_Detect(qr_int_str[rounds * 3]);
                }
                else
                {
                    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);
                    storageServo.setAngle(storage[0]);
                    MoveIndex++;
                }
                CarState = IDLE;
                break;
            case 9:
                run_state=One_button_check;
            default:
                if (MoveIndex == 6 || MoveIndex == 7)//仍武器前一个点位
                {
                    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]);
                    storageServo.setAngle(storage[qr_int_str[rounds * 3]]);
                    MaixCam.Maix_Detect(Green);
                }
                MoveIndex++;
                CarState = IDLE;
                break;
            }
        }
        break;
    default:
        break;
    }
}
