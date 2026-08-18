/**
  *****************************************************************************
  * @file               Local_MoveCar_Test.cpp
  * @author             金烜
  * @version            v1.0
  * @date               2025/7/25
  * @environment        STM32H7 (Arduino Framework)
  * @brief              小车附体坐标系下XY两方向使用不同的pid参数运动
  *****************************************************************************
/***主函数，正常使用隐去测试宏定义***/
// #define ARM_TWO      // 第二套参数
#define Move_Test    // 优先级3
// #define ZanCun_Test  // 优先级2
// #define ZhuanPan_Test// 优先级1
// #define CuJiaGon_Test// 优先级0
#define PLACE_DIRECT //不校准直接放置
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
#include "Scan.h"
#include "Arm.h"
#include "Screen.h"

// 按钮
#define START_BTN PB9                       // v1 PB8 V2 PB9 V3 PB9/PB4根据实际按键引脚修改
OneButton start_btn(START_BTN, true, true); // true:按下为低电平
bool start_flag = 0;

/*定时器对象*/
// 创建一个HardwareTimer对象，选择使用TIM3，用于获取小车姿态
HardwareTimer myTimer1(TIM3);
// 创建一个HardwareTimer对象，选择使用TIM4，用于定时通讯
HardwareTimer myTimer2(TIM4);

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
int state = 0x00;
// 选择是否开环
int isOpen = 0;
// 一键启动
void start_click()
{
    start_flag = 1;
}

/***************************定时器***********************************/
void UART_PC()
{
    if((run_state==CuJiaGon||run_state==ZanCun)&&state==0x33){
        double Ctime=millis();
        if(Ctime-Previous_Time>5000 && Ctime-Current_Time>5000){//表明被wait函数卡死
            state=0x44;
            armStepper.runToNewPosition(STEPPER_ZERO);
            gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);
            armStepper.wait();
            gripperStepper.wait();
            armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);
        }
    }
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
    if(!isOpen)
    {
        while (Serial_OPS9.available())
        {

            OPS9.readData(Serial_OPS9.read());
            if(CarState!=Rotate_Complete||run_state==ZhuanPan){
                Current_X = OPS9.pos_y;
                Current_Y = -OPS9.pos_x;
            }
        }
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
    OPS9.Update_A((float)0.0);
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
    gripperStepper.set(150, 254, 0, CHILUN, 64);

    ArmBaseStepper_protocol.init(&Serial_ArmBaseStepper, STEPPER_BAUDRATE);
    armBaseStepper.init();
    armBaseStepper.set(250, 250, 1, SYNBELT, 32);
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
    LocalRot_PID.PID_Init(LocalRot_PID_Kp, LocalRot_PID_Ki, LocalRot_PID_Kd, LocalRot_PID_MItg, LocalRot_PID_MOut);
    LocalX_PID.PID_Init(LocalX_PID_Kp, LocalX_PID_Ki, LocalX_PID_Kd, LocalX_PID_MItg, LocalX_PID_MOut);
    LocalY_PID.PID_Init(LocalY_PID_Kp, LocalY_PID_Ki, LocalY_PID_Kd, LocalY_PID_MItg, LocalY_PID_MOut);

    ARM_PID.PID_Init(ARM_PID_Kp, ARM_PID_Ki, ARM_PID_Kd, ARM_PID_MItg, ARM_PID_MOut);
    ARM_BASE_PID.PID_Init(ARM_BASE_PID_Kp, ARM_BASE_PID_Ki, ARM_BASE_PID_Kd, ARM_BASE_PID_MItg, ARM_BASE_PID_MOut);
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
            start_flag=0;
            MoveIndex++;
            // 再次更新OPS信息
            if(!isOpen){
                OPS9.Update_X(0);
                delay(10);
                OPS9.Update_Y(0);
                delay(10);
                OPS9.Update_A((float)0.0);
                delay(10);
            }
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
            //任务区进行任务时不更新OPS信息，避免数据飘移影响
            if(!isOpen){
                OPS9.Update_XY(-Current_Y,Current_X);
                delay(10);
                OPS9.Update_A((float)CurrentYaw);
                delay(10);
            }
            CarState = IDLE;
        }
        break;
    case ZhuanPan:
        dataIndex = rounds * 3;
        state = 0x00;
        MaixCam.Maix_Follow(qr_int_str[dataIndex]);
        while (dataIndex <= (2 + rounds * 3))
        {
            #ifdef ZhuanPan_Test
                        // sprintf(str, "Run.t0.txt=\"%d\"\xff\xff\xff", num);
            #endif
            if(isOpen){
                bool isRunning = (stepper1.isRunning() ||
                                  stepper2.isRunning() ||
                                  stepper3.isRunning() ||
                                  stepper4.isRunning());
            }
            switch (state)
            {
            case 0x00:
                Update_Scale(STEPPER_ZERO, ItemHeight);
                armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]);    // 机械臂朝外
                storageServo.setAngle(storage[qr_int_str[dataIndex]]); // 载物台旋转
                gripper.openMax();
                gripper_state = false;
                armBaseStepper.wait();
                gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);
                state = 0x11;
                break;
            case 0x11:
                if (vision_updated && MaixCam.Color == qr_int_str[dataIndex])
                {
                    gripper_reachout(-Move_Y_Grab * 10);
                    state = 0x22;

                    ARM_PID.PID_Init();
                    if (isOpen)
                    {
                        ARM_BASE_PID.PID_Init(ARM_BASE_ZHUANPAN_PID_Kp,
                                              ARM_BASE_ZHUANPAN_PID_Ki, 
                                              ARM_BASE_ZHUANPAN_PID_Kd, 
                                              ARM_BASE_ZHUANPAN_PID_MItg, 
                                              ARM_BASE_ZHUANPAN_PID_MOut);
                    }
                    vision_updated = false;
                    Ready_to_Grab = false;
                    Current_Time = Previous_Time = millis();
                }
                break;
            case 0x22:  
                if(!isOpen)
                {
                    RunMotors_Speed();
                }
                
                /***********更新步进到位状态**********/
                if (!armStepper.onPos_state)
                {
                    armStepper.update_state(); // 更新到位
                }
                /*****更新视觉转换比例Scale****************/
                if (!armStepper.onPos_state)
                {
                    armStepper.update_CurrentPos(); // 直接更新更好
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
                    if (fabs(Move_X_Grab) < 8 + isOpen * 5 && Move_Y_Grab < 8 + isOpen * 5 && !gripper_state && (!Ready_to_Grab || !armStepper.onPos_state)) // 未夹持时（上方和下降中）允许物料的范围
                    {
                        /*机械臂下降*/
                        if (!Ready_to_Grab)
                        {
                            armStepper.runToNewPosition(STEPPER_ZHUANPAN);
                            Ready_to_Grab = true;
                            armStepper.onPos_state = false;
                        }
                        Current_Time = millis();
                    }
                    else if (Ready_to_Grab && armStepper.onPos_state && fabs(Move_X_Grab) < 15 && Move_Y_Grab > -8)
                    { // 下降完成（包含夹持时）后阈值范围需要变化
                        Current_Time = millis();
                    }
                    else
                    {
                        if(Ready_to_Grab && armStepper.onPos_state && fabs(Move_X_Grab) > 30)//如果下降完成后物料x偏差过大，说明在夹爪范围外了，需要重新升起
                        {
                            armStepper.runToNewPosition(STEPPER_ZERO);
                            armStepper.onPos_state=false;
                            Ready_to_Grab = false;
                        }
                        else if (gripper_state && fabs(Move_Y_Grab) > 8)
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
                            Previous_Time = millis();
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
                    

                    if(!isOpen)// 若闭环跑点
                    {
                        ARM_PID.PID_Calc(0, Move_Y_Grab * 10 * 0.8);
                        float gripper_delta = gripper_reachout(ARM_PID.output);
                        
                        Follow_bySpeed_ZhuanPan(2, 35, 5, Move_X_Grab, Move_Y_Grab + gripper_delta); // gripper_delta正方向与Move_Y_Grab相反
                    }
                    else
                    {
                        Follow_by_Arm(Move_X_Grab * 10, Move_Y_Grab*10,200,50,254,254);
                    }
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
                if (dataIndex != (2 + rounds * 3))
                {
                    MaixCam.Maix_Follow(qr_int_str[dataIndex + 1]);
                }
                Grab_ZhuanPan_to_Storage(qr_int_str[dataIndex]);
                dataIndex++;
                if (dataIndex != (3 + rounds * 3))
                {
                    CarState = IDLE;
                    //机械臂准备抓取
                    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]); // 机械臂朝外
                    storageServo.setAngle(qr_int_str[dataIndex]);       // 载物台旋转
                    gripper.openMax();

                    gripper_state = false;
                    armBaseStepper.wait();
                    gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);
                    state = 0x44;
                }
                break;
            case 0x44:
                if(!isOpen)
                {
                   RunCar_MaR_Point_With_Local_PID(); // 回正位置
                }
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
            switch (state)
            {
            case 0x00:
                Update_Scale(STEPPER_ZERO / 10, 0);
                armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]); // 机械臂朝外
                storageServo.setAngle(storage[4]);                  // 载物台旋转
                gripper.Unfold();
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
                    Current_Time = Previous_Time = millis(); // 用以保证识别稳定
                }
                break;
            case 0x22: // 脚校准
                 RunMotors_Speed();
                /***********更新步进到位状态**********/
                if (!armStepper.onPos_state)
                {
                    armStepper.update_state(); // 更新到位
                }
                /*****更新视觉转换比例Scale****************/
                if (!armStepper.onPos_state)
                {
                    armStepper.update_CurrentPos();
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
                /***********获取视觉数据****************/
                if (vision_updated && MaixCam.Color == Green)
                {
                    if (fabs(Move_X_Grab) != 0 || fabs(Move_Y_Grab) != 0)
                    {
                        Previous_Time = millis();
                    }
                    if (fabs(Move_X_Grab) < 500 && fabs(Move_Y_Grab) < 500)
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
                        else if (armStepper.onPos_state && fabs(Move_X_Grab) == 0 && fabs(Move_Y_Grab) == 0)
                        { // 识别准确
                            Current_Time = millis();
                            if (Current_Time - Previous_Time > 800)
                            {
                                state = 0x33;
                                // 更新坐标 
                                update_Color_Place(Green);
                                Current_X = Task_Circle[0].x;
                                Current_Y = (Task_Circle[0].y-ARM_TO_CAR_CENTER_DIS-Arm_Zero_Length/10.0-STEPPER_GRIPPER[Green]/10.0);
                                Gripper_Move_Direct(-CirCle_Spacing, 0); // 直线前往蓝色避免干涉
                                MaixCam.Maix_Detect(Blue);
                                // 机械臂两个自由度PID初始化
                                ARM_PID.PID_Init();
                                ARM_BASE_PID.PID_Init(ARM_BASE_PID_Kp,
                                                      ARM_BASE_PID_Ki, 
                                                      ARM_BASE_PID_Kd, 
                                                      ARM_BASE_PID_MItg, 
                                                      ARM_BASE_PID_MOut);
                                // 更新记录的时间，以便下次使用
                                Current_Time = Previous_Time = millis(); // 用以保证识别稳定
                                break;
                            }
                        }
                    }
                    Follow_bySpeed(2, 75, 5, Move_X_Grab, Move_Y_Grab);
                    vision_updated = false;
                }
                break;
            case 0x33: // 手校准----蓝色
                /***********获取视觉数据****************/
                if (vision_updated && MaixCam.Color == Blue)
                {
                    if (fabs(Move_X_Grab) <= MaxError_Detect * Scale && fabs(Move_Y_Grab) <= MaxError_Detect * Scale)
                    { // 识别准确(误差阈值为0个像素点)
                        gripperStepper.update_state();
                        armBaseStepper.update_state();
                        // 到位后再记录位置，否则不准确
                        if (gripperStepper.onPos_state && armBaseStepper.onPos_state)
                        {
                            Current_Time = millis();
                            if (Current_Time - Previous_Time > 800)
                            { // 有一定的保持时间，保证稳定
                                state = 0x44;
                                update_Color_Place(Blue);
                                armStepper.runToNewPosition(STEPPER_ZERO);
                                gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);
                                Calc_Color3_Place();

                                armStepper.wait();
                                gripperStepper.wait();
                                armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);
                                break;
                            }
                        }
                    }
                    else
                    {
                        Previous_Time = millis();
                        if (Previous_Time - Current_Time >= 5000)
                        {
                            gripperStepper.wait();
                            armBaseStepper.wait();
                            state = 0x44;
                            MaixCam.Maix_Init();
                            update_Color_Place(Blue);
                            armStepper.runToNewPosition(STEPPER_ZERO);
                            gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);
                            Calc_Color3_Place();
                            armStepper.wait();
                            gripperStepper.wait();
                            armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);
                            break;
                        }
                    }
                    Follow_by_Arm(Move_X_Grab * 10.0, Move_Y_Grab * 10.0, Gripper_STEPPER_Vel, ARM_BASE_STEPPER_Vel, Gripper_STEPPER_Acc, ARM_BASE_STEPPER_Acc);
                    if (std::hypot(Move_X_Grab, Move_Y_Grab) < 10 * Scale)
                   {
                        armBaseStepper.wait();
                        gripperStepper.wait();
                   }
                    vision_updated = false;
                }
                break;
            case 0x44:
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
        #ifdef CuJiaGon_Test
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
        if(!isOpen)
        {
            //OPS9.Update_XY(-(Task_Circle[0].y-ARM_TO_CAR_CENTER_DIS-Arm_Zero_Length/10.0-STEPPER_GRIPPER[Green]/10.0),Task_Circle[0].x);//更新ops坐标
            OPS9.Update_XY(-Current_Y,Current_X);
            delay(10);
            OPS9.Update_A((float)CurrentYaw);
            delay(10);
        }
        CarState = IDLE;
        storageServo.setAngle(storage[0]);
        break;
    case ZanCun:
        dataIndex = rounds * 3;
        MaixCam.Maix_Detect(Green);
        state = 0x00;
        while (dataIndex <= (2 + rounds * 3))
        {
            switch (state)
            {
            case 0x00:
                Update_Scale(STEPPER_ZERO / 10, 0);
                armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]); // 机械臂朝外
                storageServo.setAngle(storage[4]);                  // 载物台旋转
                gripper.Unfold();
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
                    Ready_to_Grab = false;                   // 未命令下降
                    Current_Time = Previous_Time = millis(); // 用以保证识别稳定
                }
                break;
            case 0x22: // 脚校准
                RunMotors_Speed();
                /***********更新步进到位状态**********/
                if (!armStepper.onPos_state)
                {
                    armStepper.update_state(); // 更新到位
                }
                /*****更新视觉转换比例Scale****************/
                if (!armStepper.onPos_state)
                {
                    armStepper.update_CurrentPos();
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
                /***********获取视觉数据****************/
                if (vision_updated && MaixCam.Color == Green)
                {
                    if (fabs(Move_X_Grab) > rounds * 5 || fabs(Move_Y_Grab) > rounds * 5)
                    {
                        Previous_Time = millis();
                    }
                    if (fabs(Move_X_Grab) < 500 && fabs(Move_Y_Grab) < 500)
                    {
                        if (!Ready_to_Grab)
                        {
                            if (storageServo.isStop())
                            {
                                armStepper.runToNewPosition(STEPPER_GROUND - rounds * MATERIAL_HEIGHT);
                                armStepper.onPos_state = false;
                                Ready_to_Grab = true;
                            }
                        }
                        else if (armStepper.onPos_state && fabs(Move_X_Grab) <= rounds * 5 && fabs(Move_Y_Grab) <= rounds * 5)
                        { // 识别准确
                            Current_Time = millis();
                            if (Current_Time - Previous_Time > 500)
                            {
                                if (rounds)
                                {
                                    update_Color_Place(Green);
                                    state = 0x44; // 直接放置
                                    armStepper.runToNewPosition(STEPPER_ZERO);
                                    gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);

                                    armStepper.wait();
                                    gripperStepper.wait();
                                    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);
                                }
                                else
                                {
                                    state = 0x33;
                                    update_Color_Place(Green);
                                    Gripper_Move_Direct(-CirCle_Spacing, 0); // 直线前往蓝色避免干涉
                                }
                                
                                // 更新坐标
                                Current_X = Task_Circle[1].x-ARM_TO_CAR_CENTER_DIS-Arm_Zero_Length/10.0-STEPPER_GRIPPER[Green]/10.0;
                                Current_Y = Task_Circle[1].y;
                                MaixCam.Maix_Detect(Blue);
                                // 机械臂两个自由度PID初始化
                                ARM_PID.PID_Init();
                                ARM_BASE_PID.PID_Init(ARM_BASE_PID_Kp,
                                                      ARM_BASE_PID_Ki, 
                                                      ARM_BASE_PID_Kd, 
                                                      ARM_BASE_PID_MItg, 
                                                      ARM_BASE_PID_MOut);
                                // 更新记录的时间，以便下次使用
                                Current_Time = Previous_Time = millis(); // 用以保证识别稳定
                                break;
                            }
                        }
                    }
                    /*降低控制频率*/
                    // Follow_by_Arm(Move_X_Grab * 10.0 ,Move_Y_Grab *10.0 );
                    Follow_bySpeed(2, 75, 5, Move_X_Grab, Move_Y_Grab);
                    vision_updated = false;
                }
                break;
            case 0x33: // 手校准----蓝色
                /***********获取视觉数据****************/
                if (vision_updated && MaixCam.Color == Blue)
                {
                    if (fabs(Move_X_Grab) <= rounds * 5 + MaxError_Detect* Scale && fabs(Move_Y_Grab) <= rounds * 5 + MaxError_Detect * Scale)
                    { // 识别准确或超时
                        gripperStepper.update_state();
                        armBaseStepper.update_state();
                        // 到位后再记录位置，否则不准确
                        if (gripperStepper.onPos_state && armBaseStepper.onPos_state)
                        {
                            Current_Time = millis();
                            if (Current_Time - Previous_Time > 500)
                            { // 有一定的保持时间，保证稳定
                                state = 0x44;
                                update_Color_Place(Blue);
                                armStepper.runToNewPosition(STEPPER_ZERO);
                                gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);
                                Calc_Color3_Place();

                                armStepper.wait();
                                gripperStepper.wait();
                                armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);
                                break;
                            }
                        }
                    }
                    else
                    {
                        Previous_Time = millis();
                        if (Previous_Time - Current_Time >= 5000)
                        {
                            state = 0x44;
                            MaixCam.Maix_Init();
                            update_Color_Place(Blue);
                            armStepper.runToNewPosition(STEPPER_ZERO);
                            gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);
                            Calc_Color3_Place();

                            armStepper.wait();
                            gripperStepper.wait();
                            armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);
                            break;
                        }
                    }
                    Follow_by_Arm(Move_X_Grab * 10.0, Move_Y_Grab * 10.0, Gripper_STEPPER_Vel, ARM_BASE_STEPPER_Vel, Gripper_STEPPER_Acc, ARM_BASE_STEPPER_Acc);
                    if (std::hypot(Move_X_Grab, Move_Y_Grab) < 10 * Scale)
                    {
                        armBaseStepper.wait();
                        gripperStepper.wait();
                    }
                }
                break;
            case 0x44:
                Grab_Storage_to_Place(rounds);
                dataIndex++;
                break;
            default:
                break;
            }
        }
        if(!isOpen)
        {
            //OPS9.Update_XY(-Task_Circle[1].y,Task_Circle[1].x-ARM_TO_CAR_CENTER_DIS-Arm_Zero_Length/10.0-STEPPER_GRIPPER[Green]/10.0);//更新pos坐标
            OPS9.Update_XY(-Current_Y,Current_X);
            delay(10);
            OPS9.Update_A((float)CurrentYaw);
            delay(10);
        }
        run_state = Move;
        MoveIndex++;
        CarState = IDLE;
        storageServo.setAngle(storage[0]);
        break;
    case Move:
        if(isOpen)
        {
            RunCar_open_Point();
        }
        else
        {
            RunCar_MaR_Point_With_Local_PID();
        }

        #ifdef Move_Test
                if (CarState == Rotate_Complete)
                {
                    switch (MovePoint[MoveIndex])
                    {
                    case 10:
                        if (rounds == 0)
                        {
                            MoveIndex = 2;
                            rounds++;
                        }
                        else
                        {
                            MoveIndex++;
                        }
                        break;
                    case 4:
                        MoveIndex=6;
                        break;
                    case 11:
                        run_state = One_button_check;
                        MoveIndex = 0;
                        start_flag = 0;
                        break;
                    case 17:
                        if (rounds == 0)
                        {
                            MoveIndex = 2;
                            rounds++;
                        }
                        else
                        {
                            MoveIndex++;
                        }
                        break;
                    default:
                        MoveIndex++;
                        break;
                    }
                    CarState=IDLE;
                }
                break;
        #endif
        if (CarState == Rotate_Complete)
        {
            // 更新坐标
            if(isOpen)
            {
                Current_X = MoveSquence[MovePoint[MoveIndex]].x;
                Current_Y = MoveSquence[MovePoint[MoveIndex]].y;
            }
            // else
            // {
            //     OPS9.Update_A((float)CurrentYaw);
            //     delay(10);
            // }
            
            switch (MovePoint[MoveIndex])
            {
            case 1:
                    run_state = QR_Scan;
                break;
            case 2:
                run_state = ZhuanPan;
                if(isOpen)
                {
                    Current_X = MoveSquence[MovePoint[MoveIndex]].x;
                    Current_Y = MoveSquence[MovePoint[MoveIndex]].y + Grab_Margin;
                }
                break;
            case 7:
                run_state = CuJiaGon;
                break;
            case 9:
                run_state = ZanCun;
                break;
            case 10:
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
                    MoveIndex = 11; 
                }
                CarState = IDLE;
                break;
            case 11:
                run_state = One_button_check;
                break;
            default:
                if (MovePoint[MoveIndex] == 6 || MovePoint[MoveIndex] == 8) // 任务前一个点位
                {
                    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[2]);
                    storageServo.setAngle(storage[4]);
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
