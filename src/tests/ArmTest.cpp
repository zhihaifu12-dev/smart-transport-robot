#include <Arduino.h>
#include "RawPickupRuntime.h"

// 测试入口只调正式运行库，不包含机械臂、串口或引脚实现。
void setup() { RawPickupTest_Setup(); }
void loop() { RawPickupTest_Loop(); }
