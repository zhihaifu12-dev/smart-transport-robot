#ifndef REWIND_ROBOT_APP_H
#define REWIND_ROBOT_APP_H

// 初始化整车应用。仅由src/main.cpp调用一次。
void RobotApp_Setup();

// 推进整车有限状态机。必须由Arduino loop()持续调用。
void RobotApp_Loop();

#endif
