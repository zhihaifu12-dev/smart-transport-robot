#include "MaixCam.h"
float Move_X_Grab = 0.0; //抓取时的移动量
float Move_Y_Grab = 0.0;
int Current_Color = 0; //当前识别的颜色
float Scale = InitHeight / ImageScale; // 任意工况下测量平面单位像素的实际距离
bool vision_updated = false;
//              MaixCam       RX          TX
HardwareSerial Serial_Maix(Maix_RX, Maix_TX);
// 构造对象
Maix MaixCam;
void Maix::Maix_ReadData(uint8_t data)
{
    uint8_t receivedByte = data;
    // 重置超时计时器
    lastByteTime = millis();
    // 将字节存入缓冲区
    if (byteCount < PACKET_SIZE) 
    {
        if(receivedByte == 0xAA)
        {
            byteCount = 0;
        }
        packetBuffer[byteCount] = receivedByte;
        byteCount++;
    }
    // 检查是否收到完整数据包
    if (byteCount == PACKET_SIZE) 
    {
        // 验证帧头和帧尾
        if (packetBuffer[0] == 0xAA && packetBuffer[4] == 0xBB) 
        {
            Head = 0xAA;
            Delta_X = (int8_t)packetBuffer[1]; // 第一个有效数据
            Delta_Y = (int8_t)packetBuffer[2]; // 第二个有效数据
            Color = (int8_t)packetBuffer[3];   // 第三个有效数据
            End = 0xBB;
            vision_updated = true;
        }
        else
        {
            Delta_X = 0;
            Delta_Y = 0;
            Color = 0;
            Head = 0;
            End = 0;
        }
        
        // 重置接收状态
        byteCount = 0;
    }
    // 检查接收超时
    if (byteCount > 0 && (millis() - lastByteTime) > TIMEOUT_MS) 
    {
        byteCount = 0; // 超时重置接收状态
    }
  
}

void Maix::Maix_Init()
{
    unsigned char resetBuffer[4] = {0xAA, 0xFF, 0x00, 0xBB};
    Serial_Maix.begin(Maix_BAUDRATE);
    Serial_Maix.write(resetBuffer,4);
    Serial_Maix.flush();                // 等待发送完成

    Delta_X = 0;
    Delta_Y = 0;
    Color = 0;
    Head = 0;
    End = 0;
    vision_updated = false;

    byteCount = 0;       // 数据长度
    lastByteTime = 0;    // 接收时间
    delay(100);
}

void Maix::Maix_Follow(int RGB)
{
    unsigned char SendBuffer[4] = {0xAA, 0xCC, 0x00, 0xBB};
    SendBuffer[2] = constrain(RGB, (int)Red, (int)LightBlue);
    Serial_Maix.write(SendBuffer,4);
    Serial_Maix.flush();                // 等待发送完成
}

void Maix::Maix_Detect(int RGB)
{
    unsigned char SendBuffer[4] = {0xAA, 0xEE, 0x00, 0xBB};
    SendBuffer[2] = constrain(RGB, 1, 255);
    Serial_Maix.write(SendBuffer,4);
    Serial_Maix.flush();                // 等待发送完成
}

void Update_Scale(float CurrentHeight, float TargetHeight)
{
    float CameraHeight = InitHeight - CurrentHeight;
    float DeltaHeight = CameraHeight - TargetHeight;
    Scale = ((InitScale - ZeroScale) / InitHeight * DeltaHeight + ZeroScale) / ImageScale; // 当前工况下
}
