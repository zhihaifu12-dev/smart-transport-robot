/**
  *****************************************************************************
  * @file               Screen.h
  * @author             邹子睿
  * @version            v1.0
  * @date               2025/7/25
  * @environment        STM32H7 (Arduino Framework)
  * @brief              Header file for functions of Screen
  *****************************************************************************
**/
#ifndef Screen_H
#define Screen_H

#include "Scan.h"
#include "Arduino.h"
#include "Arm.h"
// 屏幕串口参数定义
#define TJCHMI_RX PB15
#define TJCHMI_TX PB14
#define DATA_NUM 19 // 串口屏需要传输的变量个数
#define TJCHMI_BAUDRATE 115200      // 串口屏幕波特率

extern bool parameter_ok;
extern bool tm0_En;
extern double Previous_Time, Current_Time;
extern int Data_Index;
extern unsigned char Data_Buffer[];
extern HardwareSerial Serial_TJCHMI;
extern int isOpen;

void Convey_Para_To_TJC();
void TJC_Command();
int Four_Byte_To_Int(unsigned char *ubuffer);
void Store_Para_To(unsigned char *ubuffer);
void Parameter_Data_Init_Function();
#endif