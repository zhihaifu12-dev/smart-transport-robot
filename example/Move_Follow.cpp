/**
  *****************************************************************************
  * @file               Move_Follow.cpp
  * @author             邹子睿
  * @version            v1.0
  * @date               2025/6/24
  * @environment        STM32H7 (Arduino Framework)
  * @brief              用于与MaixCam的跟随移动测试文件
  *                     跟随控制采用速度控制+前馈，速度控制中未加入PID，若跟随误差较大考虑加入PI控制
  *****************************************************************************
**/
#include <Arduino.h>
#include "AccelStepper.h"
#include "MultiStepper.h"
#include <JY901.h>
#include <HardwareTimer.h>
#include <ops9.h>
#include "OneButton.h" //一键启动
#include <stdio.h>
#include "MoveByPosition.h"
#include "PID.h"
#include "MaixCam.h"
// 按钮
#define START_BTN PB9                       // v1 PB8 V2 PB9 V3 PB9/PB4根据实际按键引脚修改
OneButton start_btn(START_BTN, true, true); // true:按下为低电平
// PC串口通讯
#define PC_RX PB15
#define PC_TX PB14
#define PC_BAUDRATE 115200 // 陀螺仪波特率

// 创建一个HardwareTimer对象，选择使用TIM3，用于获取小车姿态
HardwareTimer myTimer1(TIM3);
// 创建一个HardwareTimer对象，选择使用TIM4，用于定时通讯
HardwareTimer myTimer2(TIM4);
//              PC       RX          TX
HardwareSerial Serial_PC(PC_RX, PC_TX);


int task_state = 0x00;
bool start_flag = 0;
// 添加前馈控制变量
bool vision_updated = false;  // 视觉数据更新标志
float ops9raw = 0;
void start_click()
{
    start_flag = 1;
    MaixCam.Maix_Follow(Green); // 给MaixCam发送跟随指令
    task_state = 0x22;
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
        ops9raw = OPS9.zangle;

    }

    while (Serial_Maix.available())
    {
        MaixCam.Maix_ReadData(Serial_Maix.read());
        Move_X_Grab = MaixCam.Delta_X; //相对位移，以车为坐标系
        Move_Y_Grab = MaixCam.Delta_Y;

        vision_updated = true;  // 标记视觉数据已更新
    }
}
void UART_PC()
{
    // 使用中断通讯
    if (Serial_PC.availableForWrite()) // 获得待写的字节数
    { 
        Serial_PC.print("Current_X =");
        Serial_PC.print(Current_X,3);
        Serial_PC.print("Current_Y =");
        Serial_PC.print(Current_Y,3);
        Serial_PC.print("OPS0Yaw =");
        Serial_PC.println(ops9raw,3);
        Serial_PC.print("CurrentYaw =");
        Serial_PC.println(CurrentYaw,3);
        Serial_PC.print("pidx =");
        Serial_PC.print(MoveX_PID.output,3);
        Serial_PC.print("pidy =");
        Serial_PC.println(MoveY_PID.output,3);
    }
}
void setup()
{
    pinMode(LED, OUTPUT);
    // 延时，等其他外设上电
    delay(1000);
    // 电机复位操作
    Motor_Init();
    //MaixCam初始化
    MaixCam.Maix_Init();
    // IMU初始化
    IMU_Init();
    // PC串口初始化
    Serial_PC.begin(PC_BAUDRATE);
    delay(100);
    // OPS9初始化
    Serial_OPS9.begin(OPS9_BAUDRATE);
    OPS9.Update_X((float)0.0);
    delay(10);
    OPS9.Update_Y((float)0.0);
    delay(10);
    OPS9.Update_A(CurrentYaw);
    delay(10);
    // 配置定时器为1000Hz（1ms周期）IMU的回传频率最高为1000Hz
    myTimer1.setOverflow(1000, HERTZ_FORMAT);
    myTimer1.attachInterrupt(Get_Position); // 附加中断回调
    myTimer1.setInterruptPriority(1, 0);    // 设置中断优先级（可选）
    myTimer1.resume();
    // 配置定时器为5Hz（200ms周期）
    
    myTimer2.setOverflow(5, HERTZ_FORMAT);
    myTimer2.attachInterrupt(UART_PC);   // 附加中断回调
    myTimer2.setInterruptPriority(2, 0); // 设置中断优先级（可选）
    myTimer2.resume();
    
    // 一键启动初始化
    start_btn.reset(); // 清除一下按钮状态机的状态
    start_btn.attachClick(start_click);
    // PID初始化设置
    Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut);
    MoveX_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
    MoveY_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);

    // PC发送消息
    Serial_PC.println("INIT Finish!");

}
void loop()
{
    if (Serial_PC.available())
    {
        task_state = Serial_PC.read();
        Serial_PC.write(task_state);
    }
    switch (task_state)
    {
    case 0x00: // 一键启动
        start_btn.tick();
        break;
    case 0x11: // 走点
        RunCar();
        break;
    case 0x22: // 跟随移动
        RunMotors_Speed();
        if(vision_updated)
        {
            Follow_bySpeed(4.0, 200, 2, Move_X_Grab, Move_Y_Grab);
            vision_updated = false;
        }   
        break;        
    }
}
