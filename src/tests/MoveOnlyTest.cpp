#include <Arduino.h>
#include "RobotApp.h"

void setup()
{
    // 禁用项由moveonlytest环境的编译宏控制，正式应用代码不复制。
    RobotApp_Setup();
}

void loop()
{
    RobotApp_Loop();
}
