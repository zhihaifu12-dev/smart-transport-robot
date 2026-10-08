#ifndef REWIND_CHASSIS_H
#define REWIND_CHASSIS_H

#include <Arduino.h>

class Chassis
{
public:
    // 初始化四个底盘电机并保持原有方向、脉冲比例和坐标零点。
    void begin();
    void reenableOutputs();

    // 非阻塞服务：每次主循环都必须调用，负责输出步进脉冲。
    void service();
    bool isIdle() const;

    // 单位：mm、rpm、s。函数只下发运动目标，不等待到位。
    void moveLocal(float forwardMm, float leftMm,
                   float speedRpm, float accelerationTimeS);
    void moveGlobal(float fieldXmm, float fieldYmm,
                    float speedRpm, float accelerationTimeS);

    // 单位：rad、rpm、s。正负方向完全沿用原RotateCar约定。
    void rotate(float angleRad, float speedRpm, float accelerationTimeS);
};

#endif
