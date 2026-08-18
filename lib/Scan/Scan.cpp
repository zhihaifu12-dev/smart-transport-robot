#include "Scan.h"
#include "TaskPlanner.h"
//             串口扫码模块     RX     TX
HardwareSerial Serial_QR(QR_RX, QR_TX);
// 扫码模块相关
//  33 32 31 2B 31 32 33 0D 0A
//  GM75默认是CR
//  设置后可改为CRLF
//  CR（Carriage Return），回车符，用符号’\r’表示， 十进制ASCII代码是13，16进制0x0D；
//  LF（Line Feed），换行符，用符号’\n’表示，十进制ASCII代码是10，16进制0x0A；
const int bufferSize = 30;
char receivedData[bufferSize]; // 存储接收到的数据
char strQR[] = "000+000+000+000";
bool scanFlag = false;
int rounds = 0;          // 圈数
int dataIndex = 0;       // 数据索引
int qr_int_str[TASK_ITEM_COUNT] = {0};
int qr_position[TASK_ITEM_COUNT] = {0};
char str[100];

/// @brief 获取GM75模块扫码数据
void QRcode_scanning()
{
    if (scanFlag == false)
    {
        while (Serial_QR.available())
        {
            char incomingByte = Serial_QR.read(); // 读取一个字节数据

            if (incomingByte == '\n')
            {
                continue;
            }
            if (incomingByte == '\r')
            {
                receivedData[dataIndex] = '\0';
                // 先上报原始扫码内容，任务码格式由return_qr_int()单独校验。
                scanFlag = dataIndex > 0;
                dataIndex = 0;
            }
            else if (dataIndex < bufferSize - 1)
            {
                receivedData[dataIndex++] = incomingByte;
            }
            else
            {
                dataIndex = 0;
            }
        }
    }
}

/// @brief 扫码获得字符串转换类型
bool return_qr_int()
{
    TaskPlanner::TaskCode parsed;
    if (!TaskPlanner::parseTaskCode(receivedData, parsed))
    {
        return false;
    }
    for (int i = 0; i < TASK_ITEM_COUNT; ++i)
    {
        qr_int_str[i] = parsed.colors[i];
        qr_position[i] = parsed.positions[i];
    }
    return true;
}

int taskStorageSlot(int taskIndex)
{
    return constrain(taskIndex % 3 + 1, 1, 3);
}

int taskCoarsePosition(int taskIndex)
{
    return qr_position[constrain(taskIndex, 0, TASK_ITEM_COUNT - 1)];
}

int taskTemporaryPosition(int taskIndex)
{
    taskIndex = constrain(taskIndex, 0, TASK_ITEM_COUNT - 1);
    if (taskIndex < 3)
    {
        return qr_position[taskIndex];
    }

    for (int firstBatchIndex = 0; firstBatchIndex < 3; ++firstBatchIndex)
    {
        if (qr_int_str[firstBatchIndex] == qr_int_str[taskIndex])
        {
            return qr_position[firstBatchIndex];
        }
    }
    return qr_position[taskIndex];
}
