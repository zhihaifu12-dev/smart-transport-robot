#include "FinalTask.h"
#include "GripperConfig.h"

// 直接复用已完成实机调试的ArmTest实现，测试入口改名以避免与最终main冲突。
#define ARMTEST_SETUP_NAME armTestEmbeddedSetup
#define ARMTEST_LOOP_NAME armTestEmbeddedLoop
#include "../RawPickupRuntime/RawPickupRuntimeImpl.h"
#undef ARMTEST_SETUP_NAME
#undef ARMTEST_LOOP_NAME

namespace
{
bool rawAreaResetActive = false;
uint32_t rawAreaResetReadyAt = 0;
}

void FinalTask_BeginRawAreaArmReset()
{
    setTimeCoefficient(Config::PLACE_RETURN_TIME_COEFFICIENT);
    camera.stop();
    Serial.println("RAW EXIT: start M5/M6/M7 reset while chassis departs");

    if (fabsf(verticalCommandPosition - Config::VERTICAL_HIGH_POSITION) > 0.5f)
    {
        startVerticalMove(Config::VERTICAL_HIGH_POSITION,
                          Config::M7_LIFT_VELOCITY,
                          Config::M7_LIFT_ACCELERATION);
    }
    if (fabsf(horizontalCommandPosition - Config::HORIZONTAL_RESET_POSITION) > 0.5f)
    {
        startHorizontalMove(Config::HORIZONTAL_RESET_POSITION);
    }
    if (fabsf(baseAxis.position()) > 0.5f)
    {
        baseAxis.startMoveTo(0.0f);
    }

    rawAreaResetReadyAt = millis() + std::max(
        phaseDelayMs(Config::VERTICAL_SETTLE_MS),
        phaseDelayMs(Config::HORIZONTAL_SETTLE_MS));
    rawAreaResetActive = true;
}

void FinalTask_ServiceRawAreaArmReset()
{
    if (!rawAreaResetActive)
    {
        return;
    }
    baseAxis.service();
    if (!baseAxis.isRunning() &&
        static_cast<int32_t>(millis() - rawAreaResetReadyAt) >= 0)
    {
        rawAreaResetActive = false;
        Serial.println("RAW EXIT: asynchronous arm reset complete");
    }
}

bool FinalTask_RawAreaArmResetComplete()
{
    return !rawAreaResetActive;
}

bool FinalTask_PickRawBatch(const uint8_t colors[3],
                            const uint8_t storageSlots[3],
                            bool secondRoundArrival)
{
    if (colors == nullptr || storageSlots == nullptr)
    {
        return false;
    }

    Serial.println("=== FINAL TASK: RAW PICKUP START ===");
    if (!preparePickupHardwareForFinalTask())
    {
        Serial.println("FINAL RAW ERROR: controller preparation failed");
        return false;
    }

    if (secondRoundArrival)
    {
        // 第二轮到达原料区后立即张到-90度。首件抓取准备会保留此角度，
        // 不再调用原有-80度的openMax()，且不增加额外到达等待。
        Serial.print("SECOND RAW ARRIVAL: ID4 gripper angle(deg)=");
        Serial.println(GripperConfig::SECOND_ROUND_RAW_ENTRY_OPEN_ANGLE_DEG);
        gripper.setAngle(GripperConfig::SECOND_ROUND_RAW_ENTRY_OPEN_ANGLE_DEG,
                         phaseServoTimeMs(Config::GRIPPER_CLOSE_TIME_MS),
                         Config::GRIPPER_MAX_POWER_MW);
    }

    // 每一批原料重新建立首件固定坐标；黄色、蓝色、绿色均可作为固定坐标基准。
    pickupReferenceValid = false;
    pickupReferenceBaseAngle = 0.0f;
    pickupReferenceHorizontalPosition = 0.0f;
    preserveArmPoseBetweenMaterials = true;

    bool passed = true;
    for (uint8_t i = 0; i < 3; ++i)
    {
        if (i > 0)
        {
            prepareNextMaterialPickup();
        }

        const uint8_t color = colors[i];
        const uint8_t slot = constrain(storageSlots[i], 1, 3);
        Serial.print("FINAL RAW: color/slot=");
        Serial.print(color);
        Serial.print('/');
        Serial.println(slot);
        const bool keepArrivalGripperAngle = secondRoundArrival && i == 0;
        if (!pickRecognizedMaterial(color, slot, keepArrivalGripperAngle))
        {
            passed = false;
            break;
        }
    }

    preserveArmPoseBetweenMaterials = false;
    // 完整复位由MoveTest在离开原料区时异步启动，与底盘行驶并行。
    Serial.println(passed ? "=== FINAL RAW PICKUP COMPLETE ==="
                          : "=== FINAL RAW PICKUP FAILED ===");
    return passed;
}
