#ifndef REWIND_ROBOT_CONFIG_H
#define REWIND_ROBOT_CONFIG_H

#include <Arduino.h>

namespace RobotConfig
{
// 作用：整车调试串口、任务屏和机械臂总线的统一波特率。
// 单位：bit/s
// 风险：必须与外设配置一致，修改后会导致通信完全失效。
constexpr uint32_t SERIAL_BAUDRATE = 115200;

// 作用：实体一键启动按钮，内部上拉，按下时接地。
// 电平：LOW=按下，HIGH=释放。
constexpr uint8_t START_BUTTON_PIN = PB9;

// 作用：TJC任务显示屏串口引脚。
// 风险：与实际接线不一致时无法选区或显示任务状态。
constexpr uint8_t DISPLAY_RX_PIN = PB15;
constexpr uint8_t DISPLAY_TX_PIN = PB14;
constexpr uint32_t DISPLAY_BAUDRATE = 115200;

// 单位：ms。作用：实体按键去抖，已验证值30。
constexpr uint32_t START_BUTTON_DEBOUNCE_MS = 30;
} // namespace RobotConfig

#endif
