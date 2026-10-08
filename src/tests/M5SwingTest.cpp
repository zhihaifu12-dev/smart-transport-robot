#include <Arduino.h>
#include "Arm.h"
#include "ArmConfig.h"

namespace
{
constexpr float M5_SWING_ANGLE_TENTH_DEG = 300.0f;
constexpr uint32_t M5_ENDPOINT_HOLD_MS = 1000;
}

void setup()
{
    Serial.begin(115200);
    delay(ArmConfig::POWER_STABILIZE_MS);

    // 只初始化M5；上电时的机械位置被定义为本次测试的0度。
    armBaseStepper.init();

    Serial.println();
    Serial.println("=== M5 30 DEGREE SWING TEST ===");
    Serial.println("M5 will move between startup position and +30 degrees.");
}

void loop()
{
    Serial.println("M5: move to +30 degrees");
    armBaseStepper.runToNewPosition(M5_SWING_ANGLE_TENTH_DEG);
    delay(M5_ENDPOINT_HOLD_MS);

    Serial.println("M5: return to startup position");
    armBaseStepper.runToNewPosition(0.0f);
    delay(M5_ENDPOINT_HOLD_MS);
}
