#ifndef REWIND_RAW_PICKUP_RUNTIME_H
#define REWIND_RAW_PICKUP_RUNTIME_H

// 原料抓取测试入口。实现与最终任务共用同一份兼容运行库。
extern "C" void RawPickupTest_Setup();
extern "C" void RawPickupTest_Loop();

#endif
