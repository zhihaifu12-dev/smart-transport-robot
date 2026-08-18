#include <Arduino.h>
#include "DigitAreaRuntime.h"

// 测试入口只负责调度；M7引脚、方向、速度、加速度和目标高度均复用正式数字区模块。
void setup() { M7DigitHeightTest_Setup(); }
void loop() { M7DigitHeightTest_Loop(); }
