#include "GripperController.h"
#include "FinalTask.h"

void GripperController::closeForStartPreparation() { FinalTask_CloseGripperForStart(); }
void GripperController::openForStart() { FinalTask_OpenGripperForStart(); }
void GripperController::openForGroundPick() { FinalTask_OpenGripperForGroundPick(); }
void GripperController::closeForHolding() { FinalTask_CloseGripperForHolding(); }
void GripperController::openFully() { FinalTask_OpenGripperFully(); }
bool StorageController::moveToInitialAvoidance() { return FinalTask_ResetStorageTray(); }
void StorageController::moveToSlot(uint8_t slot) { FinalTask_MoveStorageToSlot(slot); }
