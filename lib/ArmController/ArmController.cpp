#include "ArmController.h"
#include "FinalTask.h"

bool ArmController::begin() { return FinalTask_InitArm(); }
void ArmController::moveBaseTo(float value) { FinalTask_MoveBaseTo(value); }
void ArmController::moveHorizontalTo(float value) { FinalTask_MoveHorizontalTo(value); }
void ArmController::moveVerticalTo(float value) { FinalTask_MoveVerticalTo(value); }
void ArmController::setStowed(bool stowed) { FinalTask_SetArmStowed(stowed); }
void ArmController::beginAreaReset(bool rawArea) { FinalTask_BeginAreaArmReset(rawArea); }
void ArmController::service() { FinalTask_ServiceArmReset(); }
bool ArmController::resetComplete() const { return FinalTask_ArmResetComplete(); }
bool ArmController::prepareDigitAreaOnArrival() { return FinalTask_PrepareDigitAreaOnArrival(); }

bool ArmController::pickRawBatch(const uint8_t colors[3],
                                 const uint8_t slots[3],
                                 bool secondRoundArrival)
{
    return FinalTask_PickRawBatch(colors, slots, secondRoundArrival);
}

bool ArmController::processDigitArea(const uint8_t sourceSlots[3],
                                     const uint8_t targetDigits[3],
                                     bool pickBack,
                                     bool reuseCoordinates,
                                     float depthReduction,
                                     const uint8_t targetColors[3])
{
    return FinalTask_ProcessDigitArea(sourceSlots, targetDigits, pickBack,
                                      reuseCoordinates, depthReduction,
                                      targetColors);
}
