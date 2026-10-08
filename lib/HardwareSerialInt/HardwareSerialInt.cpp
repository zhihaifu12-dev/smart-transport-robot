#include "HardwareSerialInt.h"
#include "stm32h7xx_hal.h"
#include <Arduino.h>

// 全局串口实例定义
#if defined(USART1)
HardwareSerialInt Serial1Int(USART1);
#endif

#if defined(USART2)
HardwareSerialInt Serial2Int(USART2);
#endif

#if defined(USART3)
HardwareSerialInt Serial3Int(USART3);
#endif

#if defined(UART4)
HardwareSerialInt Serial4Int(UART4);
#endif

// 构造函数
HardwareSerialInt::HardwareSerialInt(USART_TypeDef *uart) 
  : _uart(uart), _rxHead(0), _rxTail(0), _errors(0),_receiveCallback(nullptr) {}

// 初始化串口
void HardwareSerialInt::begin(uint32_t baudrate) {
  // 1. 启用时钟
  if (_uart == USART1) {
    __HAL_RCC_USART1_CLK_ENABLE();
    _irq = USART1_IRQn;
  } 
  else if (_uart == USART2) {
    __HAL_RCC_USART2_CLK_ENABLE();
    _irq = USART2_IRQn;
  } 
  else if (_uart == USART3) {
    __HAL_RCC_USART3_CLK_ENABLE();
    _irq = USART3_IRQn;
  }
  else if (_uart == UART4) {
    __HAL_RCC_UART4_CLK_ENABLE();
    _irq = UART4_IRQn;
  }
  
  // 2. 初始化GPIO
  initGPIO();
  
  // 3. 计算波特率
  // 系统时钟 / 波特率 = 分频值
  uint32_t uartdiv = SystemCoreClock / baudrate;
  _uart->BRR = uartdiv;
  
  // 4. 配置UART控制寄存器
  // 使能发送、接收、UART
  _uart->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
  
  // 5. 启用接收中断
  _uart->CR1 |= USART_CR1_RXNEIE;
  
  // 6. 配置NVIC
  HAL_NVIC_SetPriority(_irq, 0, 0);
  HAL_NVIC_EnableIRQ(_irq);
  
  // 7. 重置缓冲区
  _rxHead = _rxTail = 0;
  _errors = 0;
}

// 初始化GPIO
void HardwareSerialInt::initGPIO() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  if (_uart == USART1) {
    // PA9=TX, PA10=RX
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  } 
  else if (_uart == USART2) {
    // PA2=TX, PA3=RX
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  } 
  else if (_uart == USART3) {
    // PB10=TX, PB11=RX
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if (_uart == UART4) {
    // PC10=TX, PC11=RX
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}

// 中断处理函数
void HardwareSerialInt::handleInterrupt() {
  uint32_t isr = _uart->ISR;
  
  // 处理接收中断
  if (isr & USART_ISR_RXNE) {
    uint8_t data = static_cast<uint8_t>(_uart->RDR & 0xFF);
    
    // 计算下一个位置
    uint16_t next = (_rxHead + 1) % UART_BUFFER_SIZE;
    
    if (next != _rxTail) { // 缓冲区未满
      _rxBuffer[_rxHead] = data;
      _rxHead = next;
    } else { // 缓冲区溢出
      _errors++;
    }
    // 2. 调用回调函数（如果已设置）
    if (_receiveCallback != nullptr) {
      _receiveCallback(data);
    }
  }
  
  // 处理错误中断
  if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
    _errors++; // 错误计数
    _uart->ICR = isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE); // 清除错误标志
  }
}

// 检查是否有数据可读
int HardwareSerialInt::available() const {
  return (_rxHead != _rxTail);
}

// 读取一个字节
int HardwareSerialInt::read() {
  if (_rxHead == _rxTail) {
    return -1; // 无数据
  }
  
  uint8_t data = _rxBuffer[_rxTail];
  _rxTail = (_rxTail + 1) % UART_BUFFER_SIZE;
  return data;
}

// 发送单个字节
void HardwareSerialInt::write(uint8_t data) {
  // 等待发送寄存器为空
  while (!(_uart->ISR & USART_ISR_TXE));
  
  // 写入数据
  _uart->TDR = data;
  
  // 等待发送完成
  while (!(_uart->ISR & USART_ISR_TC));
}

// 发送字符串
void HardwareSerialInt::write(const char *str) {
  while (*str) {
    write(static_cast<uint8_t>(*str++));
  }
}

// 发送数据缓冲区
void HardwareSerialInt::write(const uint8_t *buffer, size_t size) {
  for (size_t i = 0; i < size; i++) {
    write(buffer[i]);
  }
}

// 刷新发送缓冲区
void HardwareSerialInt::flush() {
  // 等待发送完成
  while (!(_uart->ISR & USART_ISR_TC));
}

// 中断服务函数实现
extern "C" {
  void USART1_IRQHandler(void) {
    Serial1Int.handleInterrupt();
  }
  
  void USART2_IRQHandler(void) {
    Serial2Int.handleInterrupt();
  }
  
  void USART3_IRQHandler(void) {
    Serial3Int.handleInterrupt();
  }
  
  void UART4_IRQHandler(void) {
    Serial4Int.handleInterrupt();
  }
}