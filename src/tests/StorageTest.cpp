#include <Arduino.h>
#include "StorageRuntime.h"

// 放料测试由正式StorageRuntime提供硬件和动作实现。
void setup() { StorageTestRuntime_Setup(); }
void loop() { StorageTestRuntime_Loop(); }
