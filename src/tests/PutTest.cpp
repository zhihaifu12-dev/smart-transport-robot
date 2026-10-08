#include <Arduino.h>
#include "DigitAreaRuntime.h"

// 数字区测试只调用正式运行库，视觉和机械臂实现不再复制到测试入口。
void setup() { DigitAreaTest_Setup(); }
void loop() { DigitAreaTest_Loop(); }
