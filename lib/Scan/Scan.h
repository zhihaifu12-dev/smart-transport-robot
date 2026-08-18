/**
  *****************************************************************************
  * @file               Scan.h
  * @author             邹子睿
  * @version            v1.0
  * @date               2025/7/25
  * @environment        STM32H7 (Arduino Framework)
  * @brief              Header file for functions of ScanQR
  *****************************************************************************
**/

#ifndef SCAN_H
#define SCAN_H

#include <Arduino.h>
// 扫码串口通讯设置
#define QR_RX PE0
#define QR_TX PE1
#define QR_BAUDRATE 9600            // 串口扫码模块波特率 默认波特率9600

constexpr int TASK_ITEM_COUNT = 6;
constexpr int TASK_CODE_LENGTH = 15;

extern const int bufferSize;
extern char receivedData[]; // 存储接收到的数据
extern char strQR[];
extern bool scanFlag;
extern int rounds;          // 圈数
extern int dataIndex;       // 数据索引
extern int qr_int_str[TASK_ITEM_COUNT]; // 两批物料颜色，编号1~6
extern int qr_position[TASK_ITEM_COUNT]; // 两批物料在粗加工区的位置，编号1~3
extern char str[100];

extern HardwareSerial Serial_QR;
void QRcode_scanning();
bool return_qr_int();
int taskStorageSlot(int taskIndex);
int taskCoarsePosition(int taskIndex);
int taskTemporaryPosition(int taskIndex);
#endif
