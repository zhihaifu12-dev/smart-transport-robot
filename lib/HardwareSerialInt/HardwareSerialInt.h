#ifndef HARDWARESERIALINT_H
#define HARDWARESERIALINT_H

#include "stm32h750xx.h"
#include <stdint.h>

// STM32H7 USART 寄存器标志位修正
#if defined(STM32H7xx)
// STM32H7 寄存器标志位定义
#define USART_ISR_RXNE  USART_ISR_RXNE_RXFNE   // 接收数据寄存器非空标志
#define USART_ISR_TXE   USART_ISR_TXE_TXFNF    // 发送数据寄存器空标志
#define USART_ISR_TC_FLAG    USART_ISR_TC     // 传输完成标志
#endif
#define UART_BUFFER_SIZE 256
// 定义回调函数类型
typedef void (*UartReceiveCallback)(uint8_t data);

class HardwareSerialInt {
public:
  // 构造函数，传入USART外设指针
  HardwareSerialInt(USART_TypeDef *uart);
  
  // 初始化串口
  void begin(uint32_t baudrate);
  
  // 中断处理函数
  void handleInterrupt();
  
  // 检查是否有数据可读
  int available() const;
  
  // 读取一个字节
  int read();
  
  // 发送单个字节
  void write(uint8_t data);
  
  // 发送字符串
  void write(const char *str);
  
  // 发送数据缓冲区
  void write(const uint8_t *buffer, size_t size);
  
  // 刷新发送缓冲区
  void flush();
  
  // 获取错误标志
  uint32_t getErrors() const { return _errors; }
  // 设置接收回调函数
  void setReceiveCallback(UartReceiveCallback callback) {
    _receiveCallback = callback;
  }
private:
  USART_TypeDef *_uart;     // USART外设指针
  IRQn_Type _irq;           // 中断号
  
  // 接收环形缓冲区
  uint8_t _rxBuffer[UART_BUFFER_SIZE];
  volatile uint16_t _rxHead; // 缓冲区头指针
  volatile uint16_t _rxTail; // 缓冲区尾指针
  // 错误计数器
  volatile uint32_t _errors;
  
  UartReceiveCallback _receiveCallback; // 接收回调函数指针
  // 初始化GPIO
  void initGPIO();
};

// 声明全局串口实例
#if defined(USART1)
extern HardwareSerialInt Serial1Int;
#endif

#if defined(USART2)
extern HardwareSerialInt Serial2Int;
#endif

#if defined(USART3)
extern HardwareSerialInt Serial3Int;
#endif

#if defined(UART4)
extern HardwareSerialInt Serial4Int;
#endif

// 声明中断服务函数
#ifdef __cplusplus
extern "C" {
#endif
  void USART1_IRQHandler(void);
  void USART2_IRQHandler(void);
  void USART3_IRQHandler(void);
  void UART4_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif // HARDWARESERIALINT_H