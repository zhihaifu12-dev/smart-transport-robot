#include <Arduino.h>

#include "ShowcaseRuntime.h"

// 测试入口只负责转发Arduino生命周期，全部流程由ShowcaseRuntime状态机管理。
void setup() { Showcase_Setup(); }
void loop() { Showcase_Loop(); }
