#include <Arduino.h>
#include "RobotConfig.h"
#include "VisionController.h"
#include "LightTestRuntime.h"

namespace Config
{
constexpr uint8_t CAMERA_RX_PIN = PE7;
constexpr uint8_t CAMERA_TX_PIN = PE8;
constexpr uint32_t CAMERA_BAUDRATE = 115200;
constexpr uint32_t COMMAND_REPEAT_INTERVAL_MS = 100;
constexpr uint32_t COMMAND_REPEAT_DURATION_MS = 2000;
} // namespace Config

HardwareSerial cameraSerial(Config::CAMERA_RX_PIN, Config::CAMERA_TX_PIN);

bool rawButtonState = HIGH;
bool stableButtonState = HIGH;
uint32_t buttonChangedAt = 0;
bool lightEnabled = false;
uint32_t lightCommandStartedAt = 0;
uint32_t lastLightCommandAt = 0;

void sendLightCommand(bool enabled)
{
    uint8_t command[VisionProtocol::LIGHT_COMMAND_SIZE];
    VisionProtocol::makeFillLightCommand(enabled, command);
    cameraSerial.write(command, sizeof(command));
    cameraSerial.flush();

    Serial.print("LIGHT TX: A5 ");
    Serial.print(enabled ? "01" : "00");
    Serial.println(" 5A");
}

bool startButtonPressed()
{
    const bool sample = digitalRead(RobotConfig::START_BUTTON_PIN);
    if (sample != rawButtonState)
    {
        rawButtonState = sample;
        buttonChangedAt = millis();
    }

    if (sample != stableButtonState &&
        millis() - buttonChangedAt >= RobotConfig::START_BUTTON_DEBOUNCE_MS)
    {
        stableButtonState = sample;
        return stableButtonState == LOW;
    }
    return false;
}

void LightTest_Setup()
{
    Serial.begin(115200);
    pinMode(RobotConfig::START_BUTTON_PIN, INPUT_PULLUP);
    rawButtonState = stableButtonState = digitalRead(RobotConfig::START_BUTTON_PIN);
    buttonChangedAt = millis();

    cameraSerial.begin(Config::CAMERA_BAUDRATE);
    delay(300);
    sendLightCommand(false);
    delay(20);
    sendLightCommand(false);
    delay(20);
    sendLightCommand(false);

    Serial.println("=== LIGHT TEST READY ===");
    Serial.println("Press PB9 to send A5 01 5A for 2 seconds");
}

void LightTest_Loop()
{
    if (startButtonPressed())
    {
        lightEnabled = true;
        lightCommandStartedAt = millis();
        lastLightCommandAt = 0;
        Serial.println("PB9 PRESSED: turn fill light ON");
    }

    if (lightEnabled &&
        millis() - lightCommandStartedAt <= Config::COMMAND_REPEAT_DURATION_MS &&
        (lastLightCommandAt == 0 ||
         millis() - lastLightCommandAt >= Config::COMMAND_REPEAT_INTERVAL_MS))
    {
        sendLightCommand(true);
        lastLightCommandAt = millis();
    }

    delay(1);
}
