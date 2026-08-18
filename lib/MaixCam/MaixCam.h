/**
  *****************************************************************************
  * @file               MaixCam.h
  * @author             邹子睿
  * @version            v1.0
  * @date               2025/6/24
  * @environment        STM32H7 (Arduino Framework)
  * @brief              Header file for functions of Using MaixCam
  *****************************************************************************
**/

#ifndef MaixCam_h
#define MaixCam_h
#include <Arduino.h>
// 定义接收参数
#define PACKET_SIZE 5   // 数据包大小
#define TIMEOUT_MS 10   // 帧接收超时时间（毫秒）

// MaixCam串口通信
#define Maix_RX PE7
#define Maix_TX PE8
#define Maix_BAUDRATE 115200 // MaixCam波特率

// 2027任务码颜色编号
enum MaterialColor : uint8_t
{
    Red = 1,
    Yellow = 2,
    Blue = 3,
    Green = 4,
    Black = 5,
    LightBlue = 6
};

// 圆环标定仍使用视觉程序原有的2、3号目标协议。
constexpr uint8_t CALIBRATION_MARKER_A = 2;
constexpr uint8_t CALIBRATION_MARKER_B = 3;
constexpr int PLACEMENT_1 = 1;
constexpr int PLACEMENT_2 = 2;
constexpr int PLACEMENT_3 = 3;

// 视觉随高度比例系数的定义
#define InitScale 262.0  // 摄像头初始高度下，y方向实际长度
#define ZeroScale 20.0   // 零高度下像素个数
#define InitHeight 282.0 // 摄像头安装高度
#define ItemHeight 147   // 物料上端离地高度，单位为mm
#define ImageScale 240   // 图像y方向像素个数

// 允许误差
#define MaxError_Detect 1 //圆环校准允许的最大误差,单位为像素点偏差个数
class Maix
{
public:
    // 传递的x方向与y方向偏差
    int Delta_X = 0;
    int Delta_Y = 0;
    int Head = 0;
    int End = 0;
    int Color = 0;
    // 接收数据变量
    uint8_t packetBuffer[PACKET_SIZE]; // 数据包
    uint8_t byteCount = 0;             // 数据长度
    unsigned long lastByteTime = 0;    // 接收时间

    // 接收数据函数
    void Maix_ReadData(uint8_t data);

    // 初始化以及复位函数
    void Maix_Init();

    // 指定跟随颜色函数
    void Maix_Follow(int RGB);

    //检测圆环
    void Maix_Detect(int RGB);
};

extern Maix MaixCam;
extern HardwareSerial Serial_Maix;
extern float Move_X_Grab;
extern float Move_Y_Grab;
extern int Current_Color;
extern float Scale;
extern bool vision_updated;
void Update_Scale(float CurrentHeight, float TargetHeight);
#endif
