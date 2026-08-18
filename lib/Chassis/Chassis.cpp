#include "Chassis.h"
#include "MoveByPosition.h"

void Chassis::begin()
{
    Motor_Init();
}

void Chassis::reenableOutputs()
{
    Motor_ReenableOutputs();
}

void Chassis::service()
{
    RunMotors();
}

bool Chassis::isIdle() const
{
    return !stepper1.isRunning() && !stepper2.isRunning() &&
           !stepper3.isRunning() && !stepper4.isRunning();
}

void Chassis::moveLocal(float forwardMm, float leftMm,
                        float speedRpm, float accelerationTimeS)
{
    MoveCar(forwardMm, leftMm, speedRpm, accelerationTimeS);
}

void Chassis::moveGlobal(float fieldXmm, float fieldYmm,
                         float speedRpm, float accelerationTimeS)
{
    Global_MoveCar(fieldXmm, fieldYmm, speedRpm, accelerationTimeS);
}

void Chassis::rotate(float angleRad, float speedRpm,
                     float accelerationTimeS)
{
    RotateCar(angleRad, speedRpm, accelerationTimeS);
}
