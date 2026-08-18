#include "Private_Function.h"

// 定义全局变量
float Arm_Zero_Length = 1206.96826;

int STEPPER_ZHUANPAN = 750; // 下降到转盘抓取单位0.1毫米
int STEPPER_GROUND = 1530;  // 下降到地面
int STEPPER_STORAGE = 440;  // 下降放到载物台
int STEPPER_UP = 200;       // 不干涉高点
int MATERIAL_HEIGHT = 700;  // 物料高度单位0.1毫米
/*机械臂伸出*/
float STEPPER_GRIPPER[4] = {0, 920, 300, 920}; // 零点，R,G,B
int STEPPER_GRIPPER_ZHUANPAN_CENTER = 800;
/*基座*/
int ARM_BASE_Stepper_HOME_ANGLE = 315;                     // 机械臂发车初始角度
float ARM_BASE_STEPPER_ANGLE[4] = {2180, 775, 1215, 1655}; // 机械臂底部舵机角度{storage,R,G,B}

#ifndef ARM_TWO
// 载物盘舵机角度   0出发位置   1R  2G  3B  不干涉位置，储物盘三个盘位正对机械臂的角度
int storage[5] = {135, -44, 44, 132, -44};
/*爪子*/
int GRIPPER_OPEN_ANGLE = -70;     // 爪子张开时的角度
int GRIPPER_CLOSE_ANGLE = -102;   // 爪子闭合时的角度
int GRIPPER_OPEN_MAX_ANGLE = -50; // 爪子张开最大大角度
#else
/*爪子*/
int GRIPPER_OPEN_ANGLE = -50;     // 爪子张开时的角度
int GRIPPER_CLOSE_ANGLE = -72;    // 爪子闭合时的角度
int GRIPPER_OPEN_MAX_ANGLE = -20; // 爪子张开最大大角度
// 载物盘舵机角度   0出发位置   1R  2G  3B  不干涉位置，储物盘三个盘位正对机械臂的角度
int storage[5] = {70, -113, -25, 63, -113};
#endif

float Scale = InitHeight / ImageScale;

OneButton start_btn(START_BTN, true, true);
bool start_flag = 0;
HardwareSerial Serial_TJCHMI(TJCHMI_RX, TJCHMI_TX);
HardwareSerial Serial_QR(QR_RX, QR_TX);
HardwareSerial Serial_Stepper(Stepper_RX, Stepper_TX);
HardwareSerial Serial_ArmBaseStepper(ArmBaseStepper_Rx, ArmBaseStepper_Tx);
HardwareSerial Serial_SERVO(SERVO_RX, SERVO_TX);

FSUS_Protocol protocol(&Serial_SERVO, SERVO_BAUDRATE);
FSUS_Servo storageServo(STORAGE_SERVO_ID, &protocol);
FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &protocol);

FSGP_Gripper gripper(&gripperServo, GRIPPER_OPEN_ANGLE, GRIPPER_OPEN_ANGLE,GRIPPER_OPEN_MAX_ANGLE);
TTL_Protocol Stepper_protocol(&Serial_Stepper, STEPPER_BAUDRATE);
TTL_Stepper armStepper(ARM_Stepper_ID, &Stepper_protocol);
TTL_Stepper gripperStepper(Gripper_Stepper_ID, &Stepper_protocol);
TTL_Protocol ArmBaseStepper_protocol(&Serial_ArmBaseStepper, STEPPER_BAUDRATE);
TTL_Stepper armBaseStepper(ArmBaseStepper_ID, &ArmBaseStepper_protocol);
//定时器
HardwareTimer myTimer1(TIM3);
HardwareTimer myTimer2(TIM4);
//串口屏幕接收区
int Data_Index = 0;
unsigned char Data_Buffer[DATA_NUM * 4 + 6];

const int bufferSize = 8;
char receivedData[8];
char strQR[] = "000+000";
bool scanFlag = false;
int rounds = 0;//圈数
int dataIndex = 0;//任务索引指针
int qr_int_str[6] = {0};//任务存储
char str[100];
bool parameter_ok = false;
bool tm0_En = false;
double Previous_Time, Current_Time;
bool Ready_to_Grab;

// 状态机相关枚举定义
enum RunState {
    Parameter_Data_Init,
    One_button_check,
    QR_Scan,
    ZhuanPan,
    CuJiaGon,
    ZanCun,
    Move
};
enum RunState run_state = Parameter_Data_Init;

enum ParaState {
    Prepare,
    Header,
    Data,
    End,
    Finish_Data,
    Finish_Cmd
};
enum ParaState Para_state = Prepare;

/**
 * @brief 一键启动函数，用于触发启动标志。
 * 
 * 该函数会将全局变量 start_flag 设置为 1，以此来表示启动操作已被触发。
 * 通常在用户按下启动按钮时调用此函数，程序会依据该标志来启动后续的操作流程。
 */
void start_click()
{
    // 将启动标志设置为 1，表示启动操作已触发
    start_flag = 1;
}
// 收起机械臂、载物台
void Arm_Init()
{
    armStepper.runToNewPosition(STEPPER_ZERO);
    armStepper.wait();
    gripperStepper.runToNewPosition(0);
    gripperStepper.wait();
    gripper.close();                                              // 爪子闭合
    storageServo.setAngle(storage[0]);                            // 载物盘归位
    armBaseStepper.runToNewPosition(ARM_BASE_Stepper_HOME_ANGLE); // 机械臂归位
    gripper.wait();
    storageServo.wait();
    armBaseStepper.wait();
};
/**
 * @brief 从转盘抓取指定颜色的物料，并放置到载物台上
 * 
 * 机械臂流程：上升-收回-旋转-下降-打开-上升（到位等待）
 * 
 * @param color 要抓取的物料颜色对应的索引，用于确定载物盘的位置
 */
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
    armStepper.wait();                             // 上升到位避免干扰其他物料
    gripperStepper.runToNewPosition(STEPPER_ZERO); // 机械臂收回减少转动惯量
    gripperStepper.wait();

    armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]); // 机械臂朝向载物盘
    gripperStepper.wait();                              // 机械臂等待收回到位
    armBaseStepper.wait();                              // 等待机械臂旋转到位
    storageServo.wait();                                // 等待载物台旋转到位
    armStepper.runToNewPosition(STEPPER_STORAGE);       // 机械臂下降到载物台
    armStepper.wait();                                  // 等待下降到位
    gripper.open();                                     // 夹爪打开，内置到位等待
    armStepper.runToNewPosition(STEPPER_ZERO);          // 机械臂上升到最高位置
    armStepper.wait();                                  // 上升到位
}
/**
 * @brief 从载物台抓取物料，并放置到地面/堆垛物料上表面
 * 
 * 机械臂流程：打开-旋转-下降-闭合-上升-旋转-下降-打开-无/机械臂折叠/准备拿起物料
 * 
 * @param n 堆垛层数，用于计算机械臂下降到堆垛物料上表面的高度
 */
void Grab_Storage_to_Place(int n)
{
    storageServo.setAngle(storage[qr_int_str[dataIndex]]); // 载物盘旋转
    gripper.open();                                        // 爪子打开
    armBaseStepper.wait();                                 // 机械臂旋转到位
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
    armBaseStepper.wait();                                                   // 机械臂旋转到位
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
        armBaseStepper.setAngle(ARM_BASE_STEPPER_ANGLE[0]);  // 机械臂朝向载物台
        armBaseStepper.wait();                               // 机械臂旋转到位
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
    armBaseStepper.wait();                                                  // 等待机械臂旋转到位
    gripper.Unfold();                                                      // 机械爪打开
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
    armBaseStepper.wait();                              // 机械臂旋转到位
    storageServo.wait();                                // 载物台旋转到位
    armStepper.runToNewPosition(STEPPER_STORAGE);       // 机械臂下降
    armStepper.wait();                                  // 机械臂下降到位
    gripper.open();                                     // 夹爪打开，自带到位
    armStepper.runToNewPosition(STEPPER_ZERO);          // 机械臂上升到最高点
    armStepper.wait();                                  // 等待上升到位
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
    sprintf(str, "parameter.n11.val=%d\xff\xff\xff", (int)STEPPER_GRIPPER[1]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n12.val=%d\xff\xff\xff", (int)STEPPER_GRIPPER[2]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n13.val=%d\xff\xff\xff", (int)STEPPER_GRIPPER[3]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n14.val=%d\xff\xff\xff", (int)ARM_BASE_STEPPER_ANGLE[0]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n15.val=%d\xff\xff\xff", (int)ARM_BASE_STEPPER_ANGLE[1]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n16.val=%d\xff\xff\xff", (int)ARM_BASE_STEPPER_ANGLE[2]);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n17.val=%d\xff\xff\xff", GRIPPER_CLOSE_ANGLE);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n18.val=%d\xff\xff\xff", GRIPPER_OPEN_ANGLE);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n19.val=%d\xff\xff\xff", GRIPPER_OPEN_MAX_ANGLE);
    Serial_TJCHMI.print(str);
    sprintf(str, "parameter.n20.val=%d\xff\xff\xff", (int)ARM_BASE_STEPPER_ANGLE[3]);
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
        else if(frame_header == 0xEE)
        {
            Serial_TJCHMI.readBytes(ubuffer, 6);
            if(ubuffer[1] == 0xDD && ubuffer[2] == 0xDD && ubuffer[3] == 0xFF && ubuffer[4] == 0xFF && ubuffer[5] == 0xFF){
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
// 移动机器人pid参数归零
void Car_PID_Init()
{
    MoveX_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
    MoveY_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
    Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut);
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
    // 更新电机的当前位置
    gripperStepper.update_CurrentPos();
    // PID控制
    ARM_PID.PID_Calc(deltaY, 0);
    // 计算夹爪步进电机的目标位置
    float gripperStepperTarget = gripperStepper.currentPosition - ARM_PID.output;
    // 对步进电机的目标位置进行限位处理
    gripperStepperTarget = std::clamp(gripperStepperTarget, 0.0f, 1500.0f);

    gripperStepper.runToNewPosition(gripperStepperTarget, gripperVel, gripperAcc);

    // 计算角度偏移量，使用 atan2 函数替代近似计算，提高精度
    float deltaTheta = std::atan2(deltaX, (gripperStepper.currentPosition - deltaY + Arm_Zero_Length)) * (1800.0 / M_PI);
    // PID控制
    ARM_BASE_PID.PID_Calc(deltaTheta, 0);
    // 更新当前位置
    armBaseStepper.update_CurrentPos();
    // 计算机械臂基座步进电机的目标角度
    float armBaseStepperTarget = armBaseStepper.currentPosition + ARM_BASE_PID.output;
    // 对基座步进电机的目标位置进行限位处理
    armBaseStepperTarget = std::clamp(armBaseStepperTarget, 0.0f, 3450.0f);
    // 控制机械臂基座步进电机旋转到目标角度
    armBaseStepper.setAngle(armBaseStepperTarget, rotVel, rotAccel);
}
// 更新颜色放置的位置参数
void update_Color_Place(int Color)
{
    // 多次采样取平均值（减少随机噪声）
    const int SAMPLE_COUNT = 5;         // 采样次数（可根据稳定性调整）
    long gripperSum = 0, armBaseSum = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        gripperStepper.update_CurrentPos();  // 读取夹爪当前位置
        armBaseStepper.update_CurrentPos();  // 读取基座当前角度
        gripperSum += gripperStepper.currentPosition;
        armBaseSum += armBaseStepper.currentPosition;
        delay(10);  // 采样间隔（避免机械抖动影响）
    }
    // 保存平均值（降低单次采样误差）
    STEPPER_GRIPPER[Color] = gripperSum / SAMPLE_COUNT;
    ARM_BASE_STEPPER_ANGLE[Color] = armBaseSum / SAMPLE_COUNT;
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
 * @brief 根据蓝色（左边）和绿色（中间）物料的位置参数，计算红色（右边）物料的位置参数。默认从左往右依次为蓝绿红，
 *
 * 此函数通过三角函数计算红色物料的夹爪步进电机目标位置和机械臂基座步进电机目标角度，
 * 并将计算结果存储在对应的全局数组中。
 */
void Calc_Color3_Place()
{
    // 根据红、蓝、绿的位置参数，计算机械臂原始臂长
    double armZeroLength = calculateArmZeroLength(static_cast<double>(STEPPER_GRIPPER[Blue]),
                                                  static_cast<double>(ARM_BASE_STEPPER_ANGLE[Blue]),
                                                  static_cast<double>(STEPPER_GRIPPER[Green]),
                                                  static_cast<double>(ARM_BASE_STEPPER_ANGLE[Green]),
                                                  static_cast<double>(CirCle_Spacing));
    Arm_Zero_Length = static_cast<float>(armZeroLength);

    // 计算红色物料位置在 X 轴上的分量
    // 先将角度从 0.1 度转换为弧度，再使用余弦函数计算投影，最后计算差值
    double vectorX = 2 * (static_cast<double>(STEPPER_GRIPPER[Green]) + armZeroLength) * std::cos(ARM_BASE_STEPPER_ANGLE[Green] * M_PI / 1800.0) - (static_cast<double>(STEPPER_GRIPPER[Blue]) + armZeroLength) * std::cos(ARM_BASE_STEPPER_ANGLE[Blue] * M_PI / 1800.0);
    // 计算红色物料位置在 Y 轴上的分量
    double vectorY = 2 * (static_cast<double>(STEPPER_GRIPPER[Green]) + armZeroLength) * std::sin(ARM_BASE_STEPPER_ANGLE[Green] * M_PI / 1800.0) - (static_cast<double>(STEPPER_GRIPPER[Blue]) + armZeroLength) * std::sin(ARM_BASE_STEPPER_ANGLE[Blue] * M_PI / 1800.0);
    // 使用 hypot 函数计算向量的模长，减去零位长度得到最终位置
    STEPPER_GRIPPER[Red] = static_cast<float>(std::hypot(vectorX, vectorY) - armZeroLength);
    // 计算红色物料机械臂基座步进电机的原始目标角度
    // 使用 atan2 函数计算向量的角度，再将结果从弧度转换为 0.1 度
    double rawAngle = std::atan2(vectorY, vectorX) * 1800.0 / M_PI;
    // 若原始角度为负，加上 3600（即 360 度）将其转换为 0 到 3600 范围内的正角
    if (rawAngle < 0)
    {
        rawAngle += 3600;
    }
    // 将角度限制在 0 到 3450 的范围内（对应 0 到 345 度，单位 0.1 度）
    ARM_BASE_STEPPER_ANGLE[Red] = static_cast<float>(std::clamp(rawAngle, 0.0, 3450.0));
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
