/**
  *****************************************************************************
  * @file               ops9.h
  * @author             黄哲
  * @version            v2.0
  * @date               2024/10/20
  * @environment        ESP32 (Arduino Framework)
  * @brief              Header file for OPS class and related functions for ESP32
  *****************************************************************************
**/

#ifndef __OPS9_H
#define __OPS9_H

#include <Arduino.h>  // 包含Arduino核心库，简化头文件引入
//ops串口定义
#define OPS9_RX PB12
#define OPS9_TX PB13
#define OPS9_BAUDRATE 115200 //OPS9串口波特率

// OPS类定义
class OPS
{
public:
    // 构造函数，用于初始化串口
    OPS(HardwareSerial *uartPort,  unsigned long baud);

    // 初始化UART函数
    void initUART();

    // 清空UART缓存
    void emptyCache();

    // 数据接收函数
    void readData(uint8_t data);

    // 串口发送函数
    void sendData(char *data, uint8_t num); 
    void Update_A(float angle);           // 更新角度函数
    void Update_X(float posx);            // 更新X坐标函数
    void Update_Y(float posy);            // 更新Y坐标函数
    void Update_XY(float posx, float posy); // 更新XY坐标函数

    uint8_t RX3_Buf[50]  = {0};  //接受缓存区
    uint8_t TX3_Buf[256] = {0};	//发送缓存区
    uint8_t TX3_Count = 0;  		//跟踪发送的字节
    uint8_t Count3 = 0;


    float pos_x  = 0;
    float pos_y  = 0;
    float zangle = 0;
    float xangle = 0;
    float yangle = 0;
    float w_z    = 0;

private:
    HardwareSerial *_uartPort;  // 硬件串口对象指针
    int8_t _rxPin;              // 接收引脚
    int8_t _txPin;              // 发送引脚
    unsigned long _baud;        // 波特率
};

// 函数声明

void Strcat(char str1[], char str2[], uint8_t num);  // 字符串连接
extern OPS OPS9;//创立OPS对象
extern HardwareSerial Serial_OPS9;
#endif /* __OPS9_H__ */

/* end of ops9.h */