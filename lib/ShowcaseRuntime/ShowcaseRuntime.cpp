#include "ShowcaseRuntime.h"

#include <Arduino.h>

#include "ArmController.h"
#include "Chassis.h"
#include "Diagnostics.h"
#include "GripperController.h"
#include "RobotConfig.h"

namespace
{
enum class ShowcaseState : uint8_t
{
    WaitForStart,
    PrepareDigitArea,
    PlaceAndPickBack,
    StartArmReset,
    WaitArmReset,
    ResetStorageTray,
    Fault
};

// 展示任务固定按1、2、3号储料盘取料，并分别放到数字1、2、3的位置。
// FinalTask_ProcessDigitArea(..., true)会按相同顺序从地面夹回并放回原料盘。
constexpr uint8_t SHOWCASE_SOURCE_SLOTS[3] = {1, 2, 3};
constexpr uint8_t SHOWCASE_TARGET_DIGITS[3] = {1, 2, 3};

ArmController arm;
StorageController storage;
Chassis chassis;
ShowcaseState state = ShowcaseState::Fault;
ShowcaseState lastLoggedState = static_cast<ShowcaseState>(255);
uint32_t completedCycles = 0;

bool rawButtonState = HIGH;
bool stableButtonState = HIGH;
uint32_t buttonChangedAtMs = 0;

const char *stateName(ShowcaseState value)
{
    switch (value)
    {
    case ShowcaseState::WaitForStart:
        return "WAIT_FOR_START";
    case ShowcaseState::PrepareDigitArea:
        return "PREPARE_DIGIT_AREA";
    case ShowcaseState::PlaceAndPickBack:
        return "PLACE_AND_PICK_BACK";
    case ShowcaseState::StartArmReset:
        return "START_ARM_RESET";
    case ShowcaseState::WaitArmReset:
        return "WAIT_ARM_RESET";
    case ShowcaseState::ResetStorageTray:
        return "RESET_STORAGE_TRAY";
    case ShowcaseState::Fault:
        return "FAULT";
    }
    return "UNKNOWN";
}

bool startButtonPressed()
{
    const bool sample = digitalRead(RobotConfig::START_BUTTON_PIN);
    if (sample != rawButtonState)
    {
        rawButtonState = sample;
        buttonChangedAtMs = millis();
    }

    // 单位：ms。只处理去抖后的按下沿，长按不会反复触发。
    if (millis() - buttonChangedAtMs <
            RobotConfig::START_BUTTON_DEBOUNCE_MS ||
        stableButtonState == rawButtonState)
    {
        return false;
    }

    stableButtonState = rawButtonState;
    return stableButtonState == LOW;
}

bool initializeArm()
{
    Serial.println("SHOWCASE INIT: M5 at work zero, M6 retracted, M7 high");
    if (arm.begin())
    {
        return true;
    }

    Diagnostics::logFault(Diagnostics::FaultCode::ArmInitializationFailed,
                          "showcase arm initialization failed");
    return false;
}

void enterFault(Diagnostics::FaultCode code, const char *detail)
{
    Diagnostics::logFault(code, detail);
    state = ShowcaseState::Fault;
    Serial.println("SHOWCASE STOPPED: correct the fault, then press PB9 to retry");
}
} // namespace

void Showcase_Setup()
{
    Serial.begin(RobotConfig::SERIAL_BAUDRATE);
    pinMode(RobotConfig::START_BUTTON_PIN, INPUT_PULLUP);
    rawButtonState = stableButtonState =
        digitalRead(RobotConfig::START_BUTTON_PIN);
    buttonChangedAtMs = millis();

    // 展示环境不移动底盘，但仍初始化并持续服务四轮，使底盘保持停车。
    chassis.begin();

    Serial.println();
    Serial.println("=== SHOWCASE: PLACE ON GROUND AND PICK BACK ===");
    if (!initializeArm())
    {
        state = ShowcaseState::Fault;
        return;
    }

    state = ShowcaseState::WaitForStart;
    Serial.println("READY: load slots 1/2/3, then press PB9 once");
    Serial.println("After starting, successful cycles repeat without delay");
}

void Showcase_Loop()
{
    // 底盘脉冲服务与机械臂异步复位服务必须在每次loop中持续调用。
    chassis.service();
    arm.service();

    if (state != lastLoggedState)
    {
        Diagnostics::logState("SHOWCASE", stateName(state));
        lastLoggedState = state;
    }

    switch (state)
    {
    case ShowcaseState::WaitForStart:
        if (startButtonPressed())
        {
            state = ShowcaseState::PrepareDigitArea;
        }
        break;

    case ShowcaseState::PrepareDigitArea:
        // 每轮先重新绑定机械臂/舵机通信，并令ID5转到88度视觉避让位。
        // 后续正式接口负责等待舵盘到位，不在这里添加额外稳定延时。
        if (arm.prepareDigitAreaOnArrival())
        {
            state = ShowcaseState::PlaceAndPickBack;
        }
        else
        {
            enterFault(Diagnostics::FaultCode::ArmInitializationFailed,
                       "showcase digit-area preparation failed");
        }
        break;

    case ShowcaseState::PlaceAndPickBack:
        // 完整动作顺序由正式粗加工区接口保证：
        // 1. 识别数字位置；2. 从储料盘1/2/3依次取料并放到地面；
        // 3. 按相同顺序从地面夹回；4. 分别放回原储料盘。
        if (arm.processDigitArea(SHOWCASE_SOURCE_SLOTS,
                                 SHOWCASE_TARGET_DIGITS,
                                 true))
        {
            state = ShowcaseState::StartArmReset;
        }
        else
        {
            enterFault(Diagnostics::FaultCode::DigitVisionTimeout,
                       "showcase place/pick-back cycle failed");
        }
        break;

    case ShowcaseState::StartArmReset:
        // 一轮完成后先异步命令M7升至高位、M6收回、M5回工作零位。
        // 必须等机械臂复位完成，才能转动ID5储料盘，避免运动路径干涉。
        arm.beginAreaReset(false);
        state = ShowcaseState::WaitArmReset;
        break;

    case ShowcaseState::WaitArmReset:
        if (arm.resetComplete())
        {
            state = ShowcaseState::ResetStorageTray;
        }
        break;

    case ShowcaseState::ResetStorageTray:
        // ID5复位完成后立即进入下一轮；这里没有轮次结束等待时间。
        if (!storage.moveToInitialAvoidance())
        {
            enterFault(Diagnostics::FaultCode::StorageTrayOffline,
                       "showcase storage tray reset failed");
            break;
        }
        ++completedCycles;
        Serial.print("SHOWCASE CYCLE COMPLETE: ");
        Serial.println(completedCycles);
        state = ShowcaseState::PrepareDigitArea;
        break;

    case ShowcaseState::Fault:
        // 故障后保持停车。PB9只重新尝试初始化，不自动继续危险动作。
        if (startButtonPressed() && initializeArm())
        {
            state = ShowcaseState::PrepareDigitArea;
        }
        break;
    }

    delay(1);
}
