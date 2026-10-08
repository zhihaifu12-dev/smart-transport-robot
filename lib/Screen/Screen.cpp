#include "Screen.h"

bool parameter_ok = false;
bool tm0_En = false;
double Previous_Time, Current_Time;
int Data_Index = 0;
unsigned char Data_Buffer[DATA_NUM * 4 + 6];
//              屏幕模块         RX          TX
HardwareSerial Serial_TJCHMI(TJCHMI_RX, TJCHMI_TX); // 屏幕串口

// 串口屏状态机变量
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

/// @brief // 单片机更新串口屏幕参数
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


/// @brief 串口屏幕命令
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
        else if( frame_header == 0xEE)
        {
            Serial_TJCHMI.readBytes(ubuffer, 6);
            if (ubuffer[1] == 0xEE && ubuffer[2] == 0xEE && ubuffer[3] == 0xFF && ubuffer[4] == 0xFF && ubuffer[5] == 0xFF)
            {
                isOpen=1-isOpen;
                if(isOpen){
                    sprintf(str, "start.t3.txt=\"Now:IsOpen\"\xff\xff\xff");
                }
                else{
                    sprintf(str, "start.t3.txt=\"Now:NotOpen\"\xff\xff\xff");
                }
                Serial_TJCHMI.print(str);
            }
        }
        else
        {
            Serial_TJCHMI.read();
        }
    }
}


/// @brief 四字节数据转换成Int类型
/// @param ubuffer 
/// @return 
int Four_Byte_To_Int(unsigned char *ubuffer)
{
    return (int)((ubuffer[3] << 24) | (ubuffer[2] << 16) | (ubuffer[1] << 8) | ubuffer[0]);
}


/// @brief 传入从串口屏幕获得的参数
/// @param ubuffer 
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


/// @brief 串口屏与单片机参数初始化
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