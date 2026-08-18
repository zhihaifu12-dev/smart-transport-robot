#include "ops9.h"

// 构造函数OPS
OPS::OPS(HardwareSerial *uartPort, unsigned long baud)
    : _uartPort(uartPort), _baud(baud)
{
    this->initUART(); // 串口初始化
}
//                OPS          RX      TX
HardwareSerial Serial_OPS9(OPS9_RX, OPS9_TX);//OPS串口
OPS OPS9(&Serial_OPS9, OPS9_BAUDRATE);

void OPS::initUART()
{
    // 初始化串口, STM32 不需要手动指定 RX 和 TX 引脚
    (*_uartPort).begin(_baud);
    // STM32 默认的串口缓冲区大小通常已经足够
    // 如果需要修改，可以通过 STM32 的低级驱动修改
}

void OPS::emptyCache()
{
    // 清空UART接收缓冲区
    while ((*_uartPort).available())
    {
        (*_uartPort).read();
    }
}

void OPS::readData(uint8_t data)
{
    static uint8_t ch;
	static union
	{
		uint8_t date[24];
		float ActVal[6];
	}posture;
	static uint8_t count=0;
	static uint8_t i=0;
    ch=data;
	switch(count)
	{
		case 0:
			if(ch==0x0d)
				count++;
			else
				count=0;
			break;
		case 1:
			if(ch==0x0a)
			{
				i=0;
				count++;
			}
			else if(ch==0x0d);
			else
				count=0;
			break;
		case 2:
			posture.date[i]=ch;
			i++;
			if(i>=24)
			{
				i=0;
				count++;
			}
			break;
		case 3:
			if(ch==0x0a)
				count++;
			else
				count=0;
			break;
		case 4:
			if(ch==0x0d)
			{
				zangle=posture.ActVal[0];
				xangle=posture.ActVal[1];
				yangle=posture.ActVal[2];
				pos_x=posture.ActVal[3];
				pos_y=posture.ActVal[4];
				w_z=posture.ActVal[5];
			}
			count=0;
			break;
		default:
			count=0;
		break;
	} 
}

// 串口发送
void OPS::sendData(char *data, uint8_t num)
{
    for (uint8_t i = 0; i < num; i++) // 循环发送num字节的数据
    {
        _uartPort->write(data[i]); // 通过UART逐字节发送数据
    }
}

// x字节+num字节字符串组合
void Strcat(char str1[], char str2[], uint8_t num)
{
    int i = 0, j = 0;

    while (str1[i] != '\0')
        i++;

    for (j = 0; j < num; j++)
    {
        str1[i++] = str2[j];
    }
}

// 更新yaw角 -180~180
void OPS::Update_A(float angle)
{
    char update_yaw[8] = "ACTJ";
    static union
    {
        float A;
        char data[4];
    } set;

    set.A = angle;

    Strcat(update_yaw, set.data, 4);

    sendData(update_yaw, sizeof(update_yaw));
}

// 更新X
void OPS::Update_X(float posx)
{
    char update_x[8] = "ACTX";
    static union
    {
        float X;
        char data[4];
    } set;

    set.X = posx;

    Strcat(update_x, set.data, 4);

    sendData(update_x, sizeof(update_x));
}

// 更新Y
void OPS::Update_Y(float posy)
{
    char update_y[8] = "ACTY";
    static union
    {
        float Y;
        char data[4];
    } set;

    set.Y = posy;

    Strcat(update_y, set.data, 4);

    sendData(update_y, sizeof(update_y));
}

// 更新XY
void OPS::Update_XY(float posx, float posy)
{
    char update_xy[12] = "ACTD";
    static union
    {
        float XY[2];
        char data[8];
    } set;

    set.XY[0] = posx;
    set.XY[1] = posy;

    Strcat(update_xy, set.data, 8);

    sendData(update_xy, sizeof(update_xy));
}

