#include <Arduino.h>
#include "RobotApp.h"

// Arduino入口只负责把控制权交给整车应用。
// 引脚、串口、电机和任务动作全部封装在RobotApp及其下层模块中。
void setup()
{
    RobotApp_Setup();
}

void loop()
{
    RobotApp_Loop();
}
