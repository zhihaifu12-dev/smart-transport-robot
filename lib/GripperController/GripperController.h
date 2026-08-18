#ifndef REWIND_GRIPPER_CONTROLLER_H
#define REWIND_GRIPPER_CONTROLLER_H

#include <Arduino.h>

class GripperController
{
public:
    void closeForStartPreparation();
    void openForStart();
    void openForGroundPick();
    void closeForHolding();
    void openFully();
};

class StorageController
{
public:
    // ID5储料盘总线舵机，不是M5机械臂底座步进电机。
    bool moveToInitialAvoidance();
    void moveToSlot(uint8_t slotNumber);
};

#endif
