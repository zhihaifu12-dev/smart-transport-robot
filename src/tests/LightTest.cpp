#include <Arduino.h>
#include "LightTestRuntime.h"

// 补光灯测试入口只调用VisionController中的协议测试实现。
void setup() { LightTest_Setup(); }
void loop() { LightTest_Loop(); }
