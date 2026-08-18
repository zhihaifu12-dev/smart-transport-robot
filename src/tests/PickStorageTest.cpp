#include <Arduino.h>
#include "StorageRuntime.h"

// pickstoragetest环境通过STORAGE_PICK_MODE选择同一正式实现的取料分支。
void setup() { StorageTestRuntime_Setup(); }
void loop() { StorageTestRuntime_Loop(); }
