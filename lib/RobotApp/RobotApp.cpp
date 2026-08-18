#include <Arduino.h>
#include "MoveByPosition.h"
#include "Scan.h"
#include "FinalTask.h"
#include <JY901.h>
#include "RobotConfig.h"
#include "MotionConfig.h"
#include "ArmConfig.h"
#include "VisionConfig.h"
#include "RouteConfig.h"
#include "Diagnostics.h"
#include "Chassis.h"
#include "ArmController.h"
#include "GripperController.h"
#include "TaskPlanner.h"

using namespace MotionConfig;
using namespace RouteConfig;
using VisionConfig::ENABLE_ARM_INITIALIZATION;
using VisionConfig::ENABLE_CAMERA;
using VisionConfig::ENABLE_VISION_PICKUP;
using VisionConfig::REQUIRE_QR_RESULT;

constexpr uint8_t START_BTN = RobotConfig::START_BUTTON_PIN;
constexpr float ACCEL_TIME_S = MotionConfig::ACCELERATION_TIME_S;
constexpr uint32_t ARM_POWER_STABILIZE_MS = ArmConfig::POWER_STABILIZE_MS;

HardwareSerial Serial_TJCHMI(RobotConfig::DISPLAY_RX_PIN,
                             RobotConfig::DISPLAY_TX_PIN);
Chassis chassis;
ArmController arm;
GripperController gripperController;
StorageController storageController;

// 整车任务：串口屏选择启停区1/2，再按实体启动键运行一次。

enum MoveState
{
    START_WAIT,
    ARM_INITIALIZE,
    START_DELAY,
    MOVE_COMMAND,
    MOVE_WAIT,
    QR_CENTER_SEARCH_COMMAND,
    QR_CENTER_SEARCH_WAIT,
    QR_CENTER_RETURN_WAIT,
    WAIT_QR_RESULT,
    ROTATE_COMMAND,
    ROTATE_WAIT,
    RAW_PICKUP,
    COARSE_PROCESS,
    TEMPORARY_PROCESS,
    ARM_RESET_WAIT,
    ARM_ERROR,
    POINT_WAIT,
    FINISHED
};

const char *moveStateName(MoveState state)
{
    switch (state)
    {
    case START_WAIT: return "START_WAIT";
    case ARM_INITIALIZE: return "ARM_INITIALIZE";
    case START_DELAY: return "START_DELAY";
    case MOVE_COMMAND: return "MOVE_COMMAND";
    case MOVE_WAIT: return "MOVE_WAIT";
    case QR_CENTER_SEARCH_COMMAND: return "QR_CENTER_SEARCH_COMMAND";
    case QR_CENTER_SEARCH_WAIT: return "QR_CENTER_SEARCH_WAIT";
    case QR_CENTER_RETURN_WAIT: return "QR_CENTER_RETURN_WAIT";
    case WAIT_QR_RESULT: return "WAIT_QR_RESULT";
    case ROTATE_COMMAND: return "ROTATE_COMMAND";
    case ROTATE_WAIT: return "ROTATE_WAIT";
    case RAW_PICKUP: return "RAW_PICKUP";
    case COARSE_PROCESS: return "COARSE_PROCESS";
    case TEMPORARY_PROCESS: return "TEMPORARY_PROCESS";
    case ARM_RESET_WAIT: return "ARM_RESET_WAIT";
    case ARM_ERROR: return "ARM_ERROR";
    case POINT_WAIT: return "POINT_WAIT";
    case FINISHED: return "FINISHED";
    default: return "UNKNOWN";
    }
}

MoveState moveState = START_WAIT;
size_t routeIndex = 0;
uint32_t stateTime = 0;
bool taskCodeReady = false;
bool qrScanEnabled = false;
bool imuReady = false;
uint32_t lastImuDisplayTime = 0;
uint32_t lastProcessedImuAngleFrame = 0;
uint32_t imuSampleSequence = 0;
bool imuYawInitialized = false;
double lastRawImuYawRad = 0.0;
double continuousImuYawRad = 0.0;
uint8_t headingStableSamples = 0;
uint32_t headingStableSince = 0;
uint32_t lastHeadingStableSample = 0;
bool startFlag = false;
bool armHardwareReady = false;
bool armBaseStowed = false;
uint8_t workAreaVisitCount[3] = {0, 0, 0}; // 原料区、粗加工区、暂存区已进入次数
bool finishPageShown = false;
bool startButtonRawState = HIGH;
bool startButtonStableState = HIGH;
uint32_t startButtonChangedTime = 0;
bool startZoneSelected = false;
uint8_t selectedStartZone = 0;
char screenRxBuffer[16] = {0};
uint8_t screenRxLength = 0;
bool routeSegmentActive = false;
bool finalMoveChunk = false;
float moveChunkX = 0.0f;
float moveChunkY = 0.0f;
float rotationTarget = PI / 2;
uint32_t rotationStoppedTime = 0;
uint8_t rotationCommandCount = 0;
float rotationCommandedTravelRad = 0.0f;
float straightTravelDirection = 1.0f;
bool straightTravelPrepared = false;
float qrCenterSearchOffsetMm = 0.0f;
float qrCenterSearchTargetOffsetMm = QR_CENTER_SEARCH_DISTANCE_MM;

enum HeadingCorrectionPurpose
{
    WORK_AREA_ARRIVAL,
    STRAIGHT_TRAVEL_DEPARTURE
};

HeadingCorrectionPurpose headingCorrectionPurpose = WORK_AREA_ARRIVAL;

void beginHeadingCorrection(float target, HeadingCorrectionPurpose purpose)
{
    rotationTarget = target;
    headingCorrectionPurpose = purpose;
    rotationCommandCount = 0;
    rotationCommandedTravelRad = 0.0f;
    rotationStoppedTime = 0;
    moveState = ROTATE_COMMAND;
}

void onStartClick()
{
    const bool readyToStart =
        moveState == START_WAIT && armHardwareReady;
    const bool readyToRetry = moveState == ARM_ERROR;
    if (startZoneSelected && (readyToStart || readyToRetry))
    {
        startFlag = true;
        Serial.println("PB9 start button pressed");
    }
    else
    {
        Serial.println("PB9 ignored: select start zone first");
    }
}

void updateStartButton()
{
    const bool rawState = digitalRead(START_BTN);
    if (rawState != startButtonRawState)
    {
        startButtonRawState = rawState;
        startButtonChangedTime = millis();
    }

    if (millis() - startButtonChangedTime >=
            RobotConfig::START_BUTTON_DEBOUNCE_MS &&
        startButtonStableState != startButtonRawState)
    {
        startButtonStableState = startButtonRawState;
        if (startButtonStableState == LOW) // PB9使用内部上拉，按下接地
        {
            onStartClick();
        }
    }
}

void selectStartZone(uint8_t zone)
{
    if (moveState != START_WAIT || (zone != 1 && zone != 2))
    {
        return;
    }

    selectedStartZone = zone;
    startZoneSelected = true;
    startFlag = false;
    const float startY = zone == 1 ? START_ZONE_1_Y : START_ZONE_2_Y;
    const float returnQrX = zone == 1 ? RETURN_QR_ZONE_1_X : RETURN_QR_ZONE_2_X;
    const float returnQrY = zone == 1 ? RETURN_QR_ZONE_1_Y : RETURN_QR_ZONE_2_Y;
    const float returnX = zone == 1 ? RETURN_ZONE_1_X : RETURN_ZONE_2_X;
    const float returnY = zone == 1 ? RETURN_ZONE_1_Y : RETURN_ZONE_2_Y;

    // 起点、返程二维码点、最终停止点分别赋值；终点禁止从起点坐标派生。
    MoveSquence[POINT_START].x = START_ZONE_X;
    MoveSquence[POINT_START].y = startY;
    MoveSquence[POINT_RETURN_QR].x = returnQrX;
    MoveSquence[POINT_RETURN_QR].y = returnQrY;
    MoveSquence[POINT_FINISH].x = returnX;
    MoveSquence[POINT_FINISH].y = returnY;
    // 启停区与二维码区保持相同X，车头0度时只前进/后退，不产生车体Y横移。
    MoveSquence[POINT_QR].x = START_ZONE_X;
    Current_X = START_ZONE_X;
    Current_Y = startY;
    CurrentRad = PI / 2; // 两个启停区的车头都朝场地+Y

    Serial.print("Selected start zone: ");
    Serial.println(zone);
    Serial_TJCHMI.print("page run\xff\xff\xff");
    char statusCommand[40];
    snprintf(statusCommand, sizeof(statusCommand),
             "t1.txt=\"ZONE%d ARM INIT\"\xff\xff\xff", zone);
    Serial_TJCHMI.print(statusCommand);
    Serial_TJCHMI.print("x0.val=0\xff\xff\xff");
    Serial_TJCHMI.print("t3.txt=\"\"\xff\xff\xff");
    // 选择启停区并确认后立即自检；自检完成后才允许一键启动。
    moveState = ARM_INITIALIZE;
}

void updateScreenInput()
{
    while (Serial_TJCHMI.available())
    {
        const uint8_t incoming = static_cast<uint8_t>(Serial_TJCHMI.read());
        if (incoming == '\n')
        {
            continue;
        }
        if (incoming == '\r')
        {
            screenRxBuffer[screenRxLength] = '\0';
            if (strcmp(screenRxBuffer, "SEL:1") == 0)
            {
                selectStartZone(1);
            }
            else if (strcmp(screenRxBuffer, "SEL:2") == 0)
            {
                selectStartZone(2);
            }
            screenRxLength = 0;
            continue;
        }

        // TJC数值属性可能按4字节小端格式输出：
        // SEL:1 = 53 45 4C 3A 01 00 00 00 0D 0A，需把0x01/0x02转成ASCII。
        if ((incoming == 1 || incoming == 2) &&
            screenRxLength == 4 &&
            memcmp(screenRxBuffer, "SEL:", 4) == 0)
        {
            selectStartZone(incoming);
            screenRxLength = 0;
            continue;
        }

        // 同时兼容屏幕直接发送ASCII的SEL:1/SEL:2，忽略0x00和0xFF。
        if (incoming >= 0x20 && incoming <= 0x7E)
        {
            if (screenRxLength < sizeof(screenRxBuffer) - 1)
            {
                screenRxBuffer[screenRxLength++] = incoming;
                if (screenRxLength == 5 &&
                    memcmp(screenRxBuffer, "SEL:", 4) == 0 &&
                    (screenRxBuffer[4] == '1' || screenRxBuffer[4] == '2'))
                {
                    selectStartZone(screenRxBuffer[4] - '0');
                    screenRxLength = 0;
                }
            }
            else
            {
                screenRxLength = 0;
            }
        }
    }
}

bool motorsStopped()
{
    return chassis.isIdle();
}

float shortestAngle(float angle)
{
    while (angle > PI)
    {
        angle -= 2.0f * PI;
    }
    while (angle < -PI)
    {
        angle += 2.0f * PI;
    }
    return angle;
}

double shortestAngleDouble(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

void updateImu()
{
    while (Serial_WTIMU.available())
    {
        JY901.CopeSerialData(Serial_WTIMU.read());
    }

    // CopeSerialData按字节调用，只有收到新的0x53角度帧后才更新一次航向。
    if (JY901.angleUpdateCount == lastProcessedImuAngleFrame)
    {
        return;
    }
    lastProcessedImuAngleFrame = JY901.angleUpdateCount;

    const double rawYaw = static_cast<double>(JY901.stcAngle.Angle[2]) /
                          32768.0 * M_PI;
    if (!imuYawInitialized)
    {
        lastRawImuYawRad = rawYaw;
        // 以启动后第一帧为相对零点。HWT101执行Z轴复位后可能仍残留数度偏置，
        // 若直接使用该原始值，物理90度会被程序误认为95度并最终反向修到85度。
        continuousImuYawRad = 0.0;
        imuYawInitialized = true;
    }
    else
    {
        // 先消除+180/-180度跳变，再累计连续角度；比例补偿必须在展开后进行。
        continuousImuYawRad += shortestAngleDouble(rawYaw - lastRawImuYawRad);
        lastRawImuYawRad = rawYaw;
    }

    // 陀螺仪相对角以启动朝向为0度，直接提供给串口屏显示。
    // 底盘内部航向再加场地偏置：相对0度对应场地+Y方向（PI/2）。
    const double measuredRelativeYaw = continuousImuYawRad * IMU_ANGLE_SCALE;
    CurrentYaw = static_cast<float>(shortestAngleDouble(measuredRelativeYaw) *
                                    180.0 / M_PI);
    CurrentRad = static_cast<float>(shortestAngleDouble(
        measuredRelativeYaw + static_cast<double>(IMU_FIELD_OFFSET_RAD)));
    imuSampleSequence++;
    imuReady = true;
}

void resetHeadingStability()
{
    headingStableSamples = 0;
    headingStableSince = 0;
    lastHeadingStableSample = imuSampleSequence;
}

bool headingIsStable(float headingError)
{
    if (fabsf(headingError) > FINAL_HEADING_TOLERANCE_RAD)
    {
        resetHeadingStability();
        return false;
    }

    if (imuSampleSequence != lastHeadingStableSample)
    {
        lastHeadingStableSample = imuSampleSequence;
        if (headingStableSamples == 0)
        {
            headingStableSince = millis();
        }
        if (headingStableSamples < UINT8_MAX)
        {
            headingStableSamples++;
        }
    }

    return headingStableSamples >= HEADING_STABLE_SAMPLES &&
           millis() - headingStableSince >= HEADING_STABLE_MS;
}

void updateImuDisplay()
{
    if (!startZoneSelected || !imuReady ||
        millis() - lastImuDisplayTime < IMU_DISPLAY_INTERVAL_MS)
    {
        return;
    }

    lastImuDisplayTime = millis();
    char angleCommand[48];
    const long yawScaled = lroundf(CurrentYaw * IMU_DISPLAY_SCALE);
    snprintf(angleCommand, sizeof(angleCommand),
             "x0.vvs=%u\xff\xff\xffx0.val=%ld\xff\xff\xff",
             IMU_DISPLAY_DECIMALS, yawScaled);
    Serial_TJCHMI.print(angleCommand);
}

uint32_t waitTimeAt(RoutePointId point)
{
    switch (point)
    {
    case POINT_QR:
    case POINT_RETURN_QR:
    case POINT_RAW:
    case POINT_COARSE:
    case POINT_TEMPORARY:
        // 功能区到达后或任务完成后均不增加业务停留时间。
        return 0;
    default:
        return NORMAL_WAIT_MS;
    }
}

bool isWorkAreaPoint(RoutePointId point)
{
    return point == POINT_RAW ||
           point == POINT_COARSE ||
           point == POINT_TEMPORARY;
}

bool isClockwiseOnlyCorner(RoutePointId point)
{
    return point == POINT_COARSE_TEMP_CORNER ||
           point == POINT_TEMP_RAW_CORNER;
}

int8_t workAreaIndex(RoutePointId point)
{
    switch (point)
    {
    case POINT_RAW:
        return 0;
    case POINT_COARSE:
        return 1;
    case POINT_TEMPORARY:
        return 2;
    default:
        return -1;
    }
}

void setArmBaseStowed(bool stowed, bool retractHorizontal = true)
{
    if (!ENABLE_ARM_INITIALIZATION || !armHardwareReady ||
        armBaseStowed == stowed)
    {
        return;
    }

    (void)retractHorizontal;
    arm.setStowed(stowed);
    armBaseStowed = stowed;
}

void resetArmToWorkZero(RoutePointId areaPoint)
{
    if (!ENABLE_ARM_INITIALIZATION || !armHardwareReady)
    {
        return;
    }

    // M5/M6/M7异步回工作零位，底盘可同时驶离功能区。
    arm.beginAreaReset(areaPoint == POINT_RAW);
    armBaseStowed = false;
}

void beginWorkAreaTaskAfterArrival(RoutePointId point);

bool prepareWorkAreaHardwareOnArrival(RoutePointId point)
{
    if ((point != POINT_COARSE && point != POINT_TEMPORARY) ||
        !ENABLE_ARM_INITIALIZATION || !armHardwareReady)
    {
        return true;
    }
    if (arm.prepareDigitAreaOnArrival())
    {
        return true;
    }
    Serial.println("WORK AREA ARRIVAL ERROR: ID5 vision avoidance preparation failed");
    Serial_TJCHMI.print("t1.txt=\"ARMERR\"\xff\xff\xff");
    moveState = ARM_ERROR;
    return false;
}

void finishRoutePointArrival()
{
    if (ROUTE[routeIndex].point == POINT_FINISH)
    {
        // 终点保留实际漂移角度，不执行航向校准。
        moveState = FINISHED;
    }
    else if (isWorkAreaPoint(ROUTE[routeIndex].point))
    {
        if (arm.resetComplete())
        {
            if (!prepareWorkAreaHardwareOnArrival(ROUTE[routeIndex].point))
            {
                return;
            }
            if (ROUTE[routeIndex].point == POINT_TEMPORARY)
            {
                // 暂存区到达后保留实际车身方向，不读取陀螺仪校准作业角。
                beginWorkAreaTaskAfterArrival(POINT_TEMPORARY);
            }
            else
            {
                // 原料区和粗加工区到达后仍校准到各自作业方向。
                beginHeadingCorrection(ROUTE[routeIndex].heading,
                                       WORK_AREA_ARRIVAL);
            }
        }
        else
        {
            Serial.println("WORK AREA ARRIVAL: wait for asynchronous arm reset");
            moveState = ARM_RESET_WAIT;
        }
    }
    else
    {
        // 二维码区、地图中心和终点都不执行航向校准。
        const bool firstCenterWithoutQr =
            REQUIRE_QR_RESULT && routeIndex == 1 &&
            ROUTE[routeIndex].point == POINT_CENTER && !taskCodeReady;
        if (firstCenterWithoutQr)
        {
            qrCenterSearchOffsetMm = 0.0f;
            qrCenterSearchTargetOffsetMm = QR_CENTER_SEARCH_DISTANCE_MM;
            Serial.println("QR CENTER SEARCH: no valid code, start +/-200 mm motion");
            Serial_TJCHMI.print("t1.txt=\"QR SEARCH\"\xff\xff\xff");
            moveState = QR_CENTER_SEARCH_COMMAND;
        }
        else
        {
            stateTime = millis();
            moveState = POINT_WAIT;
        }
    }
}

void moveArmToWorkArea(RoutePointId point)
{
    if (!ENABLE_ARM_INITIALIZATION || !armHardwareReady)
    {
        return;
    }

    const int8_t areaIndex = workAreaIndex(point);
    if (areaIndex < 0)
    {
        return;
    }

    // 每个工作区依次进入两次；超过两次时保持使用第二批参数，避免数组越界。
    const uint8_t batchIndex = min(workAreaVisitCount[areaIndex],
                                   static_cast<uint8_t>(1));
    Serial.print("ARM WORK ZERO: batch/area=");
    Serial.print(batchIndex + 1);
    Serial.print('/');
    Serial.println(areaIndex);

    const bool firstRawArrival =
        point == POINT_RAW && workAreaVisitCount[areaIndex] == 0;
    if (firstRawArrival)
    {
        // 只有首次原料区需要从启停区135度收起位转到M5工作零位。
        Serial.println("ARM WORK ZERO: first raw arrival, release stowed pose");
        arm.setStowed(false);
        armBaseStowed = false;
    }
    else
    {
        // 后续功能区之间已经保持工作姿态，明确不再发送M5工作位命令。
        Serial.println("ARM WORK ZERO: already active, skip M5 command");
    }

    if (workAreaVisitCount[areaIndex] < 2)
    {
        ++workAreaVisitCount[areaIndex];
    }
}

void updateQrScanner()
{
    if (!qrScanEnabled || taskCodeReady)
    {
        return;
    }

    QRcode_scanning();
    if (scanFlag)
    {
        Serial.print("QR raw data: ");
        Serial.println(receivedData);

        char displayCommand[80];
        // 天问/TJC文本框使用\r换行：156+123+\r516+231
        snprintf(displayCommand, sizeof(displayCommand),
                 "t3.txt=\"%.8s\\r%s\"\xff\xff\xff",
                 receivedData, receivedData + 8);
        Serial_TJCHMI.print(displayCommand);

        if (return_qr_int())
        {
            taskCodeReady = true;
            Serial.println("QR task code valid");
            Serial_TJCHMI.print("t1.txt=\"QROK\"\xff\xff\xff");
        }
        else
        {
            Serial.println("QR task code invalid, scan again");
            Serial_TJCHMI.print("t1.txt=\"QRERR\"\xff\xff\xff");
            scanFlag = false;
        }
    }
}

void beginQrDepartureScan()
{
    // 丢弃从启停区驶向二维码区期间可能提前收到的内容。
    while (Serial_QR.available())
    {
        Serial_QR.read();
    }
    scanFlag = false;
    dataIndex = 0;
    receivedData[0] = '\0';
    taskCodeReady = false;
    qrScanEnabled = true;

    Serial.println("QR scan enabled while leaving QR zone");
    Serial_TJCHMI.print("t1.txt=\"SCAN...\"\xff\xff\xff");
    Serial_TJCHMI.print("t3.txt=\"\"\xff\xff\xff");
}

bool isFirstQrDeparture()
{
    // ROUTE[0]是首次到达二维码区，ROUTE[1]驶向中心点并在途中扫码。
    return routeIndex == 1 && ROUTE[routeIndex].point == POINT_CENTER;
}

bool isFirstRawArrival()
{
    return routeIndex == 2 && ROUTE[routeIndex].point == POINT_RAW;
}

const CAR_GOAL_POINT &routeTargetAt(size_t targetRouteIndex)
{
    const RoutePointId point = ROUTE[targetRouteIndex].point;
    const bool useSecondRoundMap =
        targetRouteIndex >= SECOND_ROUND_MAP_BEGIN_ROUTE_INDEX &&
        targetRouteIndex <= SECOND_ROUND_MAP_END_ROUTE_INDEX;
    return useSecondRoundMap ? MoveSquenceRound2[point]
                             : MoveSquence[point];
}

void beginWorkAreaTaskAfterArrival(RoutePointId point)
{
    moveArmToWorkArea(point);
    stateTime = millis();

    if (REQUIRE_QR_RESULT && isFirstRawArrival() && !taskCodeReady)
    {
        moveState = WAIT_QR_RESULT;
    }
    else if (ENABLE_VISION_PICKUP && point == POINT_RAW)
    {
        moveState = RAW_PICKUP;
    }
    else if (ENABLE_VISION_PICKUP && point == POINT_COARSE)
    {
        moveState = COARSE_PROCESS;
    }
    else if (ENABLE_VISION_PICKUP && point == POINT_TEMPORARY)
    {
        moveState = TEMPORARY_PROCESS;
    }
    else
    {
        moveState = POINT_WAIT;
    }
}

void beginStraightDepartureToNextPoint()
{
    const CAR_GOAL_POINT &nextTarget = routeTargetAt(routeIndex + 1);
    const float deltaX = nextTarget.x - Current_X;
    const float deltaY = nextTarget.y - Current_Y;
    const float forwardHeading = atan2f(deltaY, deltaX);
    const float backwardHeading = shortestAngle(forwardHeading + PI);
    const float forwardTurn = fabsf(shortestAngle(forwardHeading - CurrentRad));
    const float backwardTurn = fabsf(shortestAngle(backwardHeading - CurrentRad));

    // 路点8、9离开时固定选择倒车航向：在当前折线路径上，两次主转向均为
    // 俯视顺时针约90度（负角度），同时下一段长直线仍只前进/后退、不横移。
    const bool clockwiseOnlyCorner =
        isClockwiseOnlyCorner(ROUTE[routeIndex].point);
    const bool useBackward =
        clockwiseOnlyCorner || backwardTurn < forwardTurn;
    straightTravelDirection = useBackward ? -1.0f : 1.0f;
    const float travelHeading = useBackward ? backwardHeading : forwardHeading;

    Serial.print("STRAIGHT DEPARTURE: from/to/mode/heading(deg)=");
    Serial.print(static_cast<int>(ROUTE[routeIndex].point));
    Serial.print('/');
    Serial.print(static_cast<int>(ROUTE[routeIndex + 1].point));
    Serial.print('/');
    Serial.print(useBackward ? "BACKWARD/" : "FORWARD/");
    Serial.println(travelHeading * 180.0f / PI, 2);
    if (clockwiseOnlyCorner)
    {
        Serial.println("CORNER TURN POLICY: clockwise-only 90-degree turn");
    }
    beginHeadingCorrection(travelHeading, STRAIGHT_TRAVEL_DEPARTURE);
}

void finishHeadingCorrection()
{
    Rot_PID.PID_Init(HEADING_PID_KP, HEADING_PID_KI,
                     HEADING_PID_KD, HEADING_PID_MAX_I,
                     HEADING_PID_MAX_OUT);

    resetHeadingStability();

    if (headingCorrectionPurpose == STRAIGHT_TRAVEL_DEPARTURE)
    {
        // 当前路点已对准下一条直线；进入下一路点后只允许前进或后退。
        ++routeIndex;
        straightTravelPrepared = true;
        moveState = routeIndex < ROUTE_COUNT ? MOVE_COMMAND : FINISHED;
        return;
    }

    // 原料区和粗加工区航向严格校准完成后，再执行该区域任务。
    if (isWorkAreaPoint(ROUTE[routeIndex].point))
    {
        beginWorkAreaTaskAfterArrival(ROUTE[routeIndex].point);
    }
    else
    {
        stateTime = millis();
        moveState = POINT_WAIT;
    }
}

bool grabCurrentRawBatch()
{
    const uint8_t batch = min<uint8_t>(workAreaVisitCount[0] - 1, 1);
    const TaskPlanner::TaskCode task =
        TaskPlanner::fromLegacyArrays(qr_int_str, qr_position);
    const TaskPlanner::AreaPlan plan = TaskPlanner::makeRawPlan(task, batch);
    Serial.print("RAW PICK ORDER: QR group ");
    Serial.print(batch == TaskPlanner::FIRST_BATCH_INDEX ? 1 : 3);
    Serial.print(", colors=");
    for (uint8_t item = 0; item < TaskPlanner::ITEMS_PER_BATCH; ++item)
    {
        if (item > 0)
        {
            Serial.print('/');
        }
        Serial.print(plan.colors[item]);
    }
    Serial.println(" -> storage slots 1/2/3");
    Serial_TJCHMI.print("t1.txt=\"RAW PICK\"\xff\xff\xff");
    const bool passed = arm.pickRawBatch(plan.colors, plan.sourceSlots,
                                         batch == TaskPlanner::SECOND_BATCH_INDEX);
    if (passed)
    {
        dataIndex = (batch + 1) * TaskPlanner::ITEMS_PER_BATCH;
    }
    return passed;
}

void RobotApp_Setup()
{
    Serial.begin(115200);
    Serial_TJCHMI.begin(RobotConfig::DISPLAY_BAUDRATE);
    Serial_QR.begin(QR_BAUDRATE);
    delay(200);
    Serial_TJCHMI.print("page prepare\xff\xff\xff");
    Serial.println("Waiting for start-zone selection: SEL:1 or SEL:2");

    chassis.begin();
    IMU_Init();
    // 上电只复位储料盘舵机；M5/M6/M7仍等待选定启停区后再初始化。
    if (ENABLE_ARM_INITIALIZATION && !storageController.moveToInitialAvoidance())
    {
        Serial.println("POWER-ON WARNING: storage tray reset failed; retry during arm init");
    }
    Rot_PID.PID_Init(HEADING_PID_KP, HEADING_PID_KI, HEADING_PID_KD,
                     HEADING_PID_MAX_I, HEADING_PID_MAX_OUT);
    Current_X = MoveSquence[POINT_START].x;
    Current_Y = MoveSquence[POINT_START].y;
    CurrentRad = PI / 2; // 启停区1/2均初始面向场地+Y
    pinMode(START_BTN, INPUT_PULLUP);
    startButtonRawState = digitalRead(START_BTN);
    startButtonStableState = startButtonRawState;
    startButtonChangedTime = millis();
    stateTime = millis();
}

void RobotApp_Loop()
{
    // 每个状态只在首次进入时输出一次，便于现场串口追踪完整流程。
    static MoveState lastLoggedState = static_cast<MoveState>(255);
    if (moveState != lastLoggedState)
    {
        Diagnostics::logState("ROBOT", moveStateName(moveState));
        lastLoggedState = moveState;
    }

    // 底盘动作只设置目标，必须持续service()输出脉冲。
    chassis.service();
    arm.service();
    updateQrScanner();
    updateImu();
    updateImuDisplay();
    updateScreenInput();
    updateStartButton();

    switch (moveState)
    {
    case START_WAIT:
        if (startZoneSelected && armHardwareReady && startFlag)
        {
            startFlag = false;
            // 选区后夹爪已闭合并完成M5旋转；按下PB9后才松开夹爪。
            gripperController.openForStart();
            stateTime = millis();
            Serial_TJCHMI.print("t1.txt=\"START\"\xff\xff\xff");
            moveState = START_DELAY;
        }
        break;

    case ARM_INITIALIZE:
        if (ENABLE_ARM_INITIALIZATION && !armHardwareReady)
        {
            // 串口屏确认启停区后只初始化机械臂，不执行启动自检运动。
            delay(ARM_POWER_STABILIZE_MS);
            armHardwareReady = arm.begin();
        }
        // 临时关闭机械臂时仍允许只测试底盘。
        if (!ENABLE_ARM_INITIALIZATION)
        {
            armHardwareReady = true;
        }
        if (ENABLE_ARM_INITIALIZATION && armHardwareReady)
        {
            // 严格按“夹爪闭合 -> M5旋转”的顺序完成启动准备。
            gripperController.closeForStartPreparation();
            setArmBaseStowed(true, false);
        }
        // 机械臂初始化可能造成瞬时压降；出发前重新复位并使能四个底盘驱动器。
        chassis.reenableOutputs();
        Serial_TJCHMI.print(ENABLE_ARM_INITIALIZATION
                                ? "t1.txt=\"ARM OK - READY\"\xff\xff\xff"
                                : "t1.txt=\"READY\"\xff\xff\xff");
        moveState = START_WAIT;
        break;

    case START_DELAY:
        if (imuReady && millis() - stateTime >= START_DELAY_MS)
        {
            Serial_TJCHMI.print("t1.txt=\"RUN\"\xff\xff\xff");
            moveState = MOVE_COMMAND;
        }
        break;

    case MOVE_COMMAND:
    {
        const CAR_GOAL_POINT &target = routeTargetAt(routeIndex);
        if (!routeSegmentActive)
        {
            routeSegmentActive = true;
        }

        const float remainingX = target.x - Current_X;
        const float remainingY = target.y - Current_Y;
        const float remainingDistance = hypotf(remainingX, remainingY);
        const bool initialQrScanTravel = isFirstQrDeparture();

        if (remainingDistance < 0.01f)
        {
            routeSegmentActive = false;
            straightTravelPrepared = false;
            finishRoutePointArrival();
            break;
        }

        // 行驶阶段只按功能区坐标平移，不读取IMU航向做途中校正。
        moveChunkX = remainingX;
        moveChunkY = remainingY;
        finalMoveChunk = true;

        const float moveRpm = isFirstQrDeparture() && !taskCodeReady
                                  ? QR_SCAN_MOVE_RPM
                                  : MOVE_RPM;
        const bool outboundStartToQrStraightOnly = routeIndex == 0;
        if (outboundStartToQrStraightOnly)
        {
            // 去程保持纯前进/后退；最终返程严格使用RETURN_ZONE_X/Y两个坐标。
            chassis.moveLocal(moveChunkY * POSITION_SCALE, 0.0f,
                              moveRpm, ACCEL_TIME_S);
        }
        else if (initialQrScanTravel)
        {
            // 唯一允许横移的路段：首次离开二维码区并在行驶中扫码。
            chassis.moveGlobal(moveChunkX * POSITION_SCALE,
                               moveChunkY * POSITION_SCALE,
                               moveRpm,
                               ACCEL_TIME_S);
        }
        else if (straightTravelPrepared)
        {
            // 其他所有路段只允许前进或后退，横向分量固定为0。
            chassis.moveLocal(straightTravelDirection *
                                  remainingDistance * POSITION_SCALE,
                              0.0f, moveRpm, ACCEL_TIME_S);
            straightTravelPrepared = false;
        }
        else
        {
            Serial.println("ROUTE ERROR: straight travel heading was not prepared");
            moveState = ARM_ERROR;
            break;
        }
        moveState = MOVE_WAIT;
        break;
    }

    case MOVE_WAIT:
        if (motorsStopped())
        {
            // 用已执行的功能区间指令更新开环坐标，不进行途中航向检查。
            Current_X += moveChunkX;
            Current_Y += moveChunkY;

            if (finalMoveChunk)
            {
                const CAR_GOAL_POINT &target = routeTargetAt(routeIndex);
                Current_X = target.x;
                Current_Y = target.y;
                routeSegmentActive = false;
                finishRoutePointArrival();
            }
            else
            {
                moveState = MOVE_COMMAND;
            }
        }
        break;

    case QR_CENTER_SEARCH_COMMAND:
    {
        // 若相机恰好在下一段搜索动作下发前完成识别，小车本来就在中心，直接继续路线。
        if (taskCodeReady && fabsf(qrCenterSearchOffsetMm) < 0.01f)
        {
            Serial.println("QR CENTER SEARCH: code ready at center");
            beginStraightDepartureToNextPoint();
            break;
        }

        const float travelMm =
            qrCenterSearchTargetOffsetMm - qrCenterSearchOffsetMm;
        Serial.print("QR CENTER SEARCH: current/target/travel(mm)=");
        Serial.print(qrCenterSearchOffsetMm);
        Serial.print('/');
        Serial.print(qrCenterSearchTargetOffsetMm);
        Serial.print('/');
        Serial.println(travelMm);
        chassis.moveLocal(travelMm * POSITION_SCALE, 0.0f,
                          QR_CENTER_SEARCH_RPM, ACCEL_TIME_S);
        moveState = QR_CENTER_SEARCH_WAIT;
        break;
    }

    case QR_CENTER_SEARCH_WAIT:
        if (motorsStopped())
        {
            qrCenterSearchOffsetMm = qrCenterSearchTargetOffsetMm;
            if (taskCodeReady)
            {
                // 扫码成功后按当前相对偏移回到地图中心，再恢复正式路线。
                Serial.print("QR CENTER SEARCH: code ready, return(mm)=");
                Serial.println(-qrCenterSearchOffsetMm);
                chassis.moveLocal(-qrCenterSearchOffsetMm * POSITION_SCALE,
                                  0.0f, QR_CENTER_SEARCH_RPM,
                                  ACCEL_TIME_S);
                moveState = QR_CENTER_RETURN_WAIT;
            }
            else
            {
                qrCenterSearchTargetOffsetMm =
                    qrCenterSearchTargetOffsetMm > 0.0f
                        ? -QR_CENTER_SEARCH_DISTANCE_MM
                        : QR_CENTER_SEARCH_DISTANCE_MM;
                moveState = QR_CENTER_SEARCH_COMMAND;
            }
        }
        break;

    case QR_CENTER_RETURN_WAIT:
        if (motorsStopped())
        {
            qrCenterSearchOffsetMm = 0.0f;
            const CAR_GOAL_POINT &center = routeTargetAt(routeIndex);
            Current_X = center.x;
            Current_Y = center.y;
            Serial.println("QR CENTER SEARCH: returned to map center");
            Serial_TJCHMI.print("t1.txt=\"QROK\"\xff\xff\xff");
            beginStraightDepartureToNextPoint();
        }
        break;

    case WAIT_QR_RESULT:
        if (taskCodeReady)
        {
            // 中心等待点已删除；首轮原料区直接承接扫码完成后的抓取任务。
            moveState = ENABLE_VISION_PICKUP ? RAW_PICKUP : POINT_WAIT;
            stateTime = millis();
        }
        break;

    case ROTATE_COMMAND:
    {
        if (!imuReady)
        {
            break;
        }

        // 显式构造最短圆周误差，避免目标与当前角跨越-180/+180时绕远路。
        const float headingError = shortestAngle(rotationTarget - CurrentRad);
        Rot_PID.PID_Calc(CurrentRad + headingError, CurrentRad);
        if (fabsf(headingError) <= FINAL_HEADING_TOLERANCE_RAD)
        {
            if (headingIsStable(headingError))
            {
                finishHeadingCorrection();
            }
        }
        else
        {
            const uint8_t maxRotationCommands =
                1 + MAX_ROTATION_FINE_ADJUSTMENTS;
            if (rotationCommandCount >= maxRotationCommands)
            {
                Serial.print("Rotation fine-adjust limit reached, residual(deg): ");
                Serial.println(headingError * 180.0f / PI, 4);
                Serial_TJCHMI.print("t1.txt=\"ANGLE RETRY\"\xff\xff\xff");
                Rot_PID.PID_Init(HEADING_PID_KP, HEADING_PID_KI,
                                 HEADING_PID_KD, HEADING_PID_MAX_I,
                                 HEADING_PID_MAX_OUT);
                beginHeadingCorrection(rotationTarget,
                                       headingCorrectionPurpose);
                break;
            }

            const float remainingRotationTravel =
                MAX_ROTATION_TRAVEL_RAD - rotationCommandedTravelRad;
            if (remainingRotationTravel <= 0.0001f)
            {
                Serial.print("Rotation one-turn limit reached, residual(deg): ");
                Serial.println(headingError * 180.0f / PI, 4);
                Serial_TJCHMI.print("t1.txt=\"ANGLE RETRY\"\xff\xff\xff");
                Rot_PID.PID_Init(HEADING_PID_KP, HEADING_PID_KI,
                                 HEADING_PID_KD, HEADING_PID_MAX_I,
                                 HEADING_PID_MAX_OUT);
                beginHeadingCorrection(rotationTarget,
                                       headingCorrectionPurpose);
                break;
            }

            resetHeadingStability();
            float correctionRpm = ROTATE_RPM;
            if (fabsf(headingError) < 0.1f)
            {
                correctionRpm = max(8.0f,
                                    ROTATE_RPM * fabsf(headingError) / 0.1f);
            }

            const float theoreticalImuYaw =
                shortestAngle(rotationTarget - IMU_FIELD_OFFSET_RAD);
            Serial.print("IMU target/measured/error(deg): ");
            Serial.print(theoreticalImuYaw * 180.0f / PI, 2);
            Serial.print('/');
            Serial.print(CurrentYaw, IMU_DISPLAY_DECIMALS);
            Serial.print('/');
            Serial.println(headingError * 180.0f / PI, 2);

            // 恢复最短路径正反转：第一次按完整误差到位，之后最多两次PID微调。
            float rotationCommand = rotationCommandCount == 0
                                        ? headingError
                                        : Rot_PID.output;
            if (fabsf(rotationCommand) > remainingRotationTravel)
            {
                rotationCommand = copysignf(remainingRotationTravel,
                                            rotationCommand);
            }
            rotationCommandedTravelRad += fabsf(rotationCommand);
            ++rotationCommandCount;
            chassis.rotate(rotationCommand, correctionRpm, ACCEL_TIME_S);
            rotationStoppedTime = 0;
            moveState = ROTATE_WAIT;
        }
        break;
    }

    case ROTATE_WAIT:
        if (motorsStopped())
        {
            if (rotationStoppedTime == 0)
            {
                rotationStoppedTime = millis();
            }
            else if (millis() - rotationStoppedTime >= ROTATION_SETTLE_MS)
            {
                rotationStoppedTime = 0;
                // 重新读取实际角度，由ROTATE_COMMAND判断完成或继续微调。
                moveState = ROTATE_COMMAND;
            }
        }
        else
        {
            rotationStoppedTime = 0;
        }
        break;

    case RAW_PICKUP:
        // 每次到达原料区抓取当前批次的3个物料。
        // 第一次使用任务码0..2，第二次自动使用3..5。
        if (grabCurrentRawBatch())
        {
            Serial_TJCHMI.print("t1.txt=\"PICK OK\"\xff\xff\xff");
            stateTime = millis();
            moveState = POINT_WAIT;
        }
        else
        {
            Serial_TJCHMI.print("t1.txt=\"ARMERR\"\xff\xff\xff");
            moveState = ARM_ERROR;
        }
        break;

    case COARSE_PROCESS:
    {
        const uint8_t batch = min<uint8_t>(workAreaVisitCount[1] - 1, 1);
        const TaskPlanner::TaskCode task =
            TaskPlanner::fromLegacyArrays(qr_int_str, qr_position);
        const TaskPlanner::AreaPlan plan = TaskPlanner::makeCoarsePlan(task, batch);
        Serial_TJCHMI.print("t1.txt=\"COARSE\"\xff\xff\xff");
        if (arm.processDigitArea(plan.sourceSlots, plan.targetDigits, true))
        {
            stateTime = millis();
            moveState = POINT_WAIT;
        }
        else
        {
            Serial_TJCHMI.print("t1.txt=\"ARMERR\"\xff\xff\xff");
            moveState = ARM_ERROR;
        }
        break;
    }

    case TEMPORARY_PROCESS:
    {
        const uint8_t batch = min<uint8_t>(workAreaVisitCount[2] - 1, 1);
        const TaskPlanner::TaskCode task =
            TaskPlanner::fromLegacyArrays(qr_int_str, qr_position);
        const TaskPlanner::AreaPlan plan =
            TaskPlanner::makeTemporaryPlan(task, batch);
        Serial_TJCHMI.print("t1.txt=\"TEMP PUT\"\xff\xff\xff");
        if (arm.processDigitArea(plan.sourceSlots,
                                 plan.targetDigits,
                                 false,
                                 batch == 1,
                                 batch == 1 ? ArmConfig::TEMPORARY_STACK_HEIGHT : 0.0f,
                                 batch == 1 ? plan.colors : nullptr))
        {
            stateTime = millis();
            moveState = POINT_WAIT;
        }
        else
        {
            Serial_TJCHMI.print("t1.txt=\"ARMERR\"\xff\xff\xff");
            moveState = ARM_ERROR;
        }
        break;
    }

    case ARM_RESET_WAIT:
        if (arm.resetComplete())
        {
            Serial.println("WORK AREA ARRIVAL: arm reset complete");
            if (!prepareWorkAreaHardwareOnArrival(ROUTE[routeIndex].point))
            {
                break;
            }
            if (ROUTE[routeIndex].point == POINT_TEMPORARY)
            {
                // 即使等待过异步机械臂复位，暂存区到达后仍跳过航向校准。
                beginWorkAreaTaskAfterArrival(POINT_TEMPORARY);
            }
            else
            {
                beginHeadingCorrection(ROUTE[routeIndex].heading,
                                       WORK_AREA_ARRIVAL);
            }
        }
        break;

    case ARM_ERROR:
        // 识别超时或对准失败时保持停车；按一次启动键重试当前物料。
        if (startFlag)
        {
            startFlag = false;
            Serial_TJCHMI.print("t1.txt=\"RETRY\"\xff\xff\xff");
            switch (ROUTE[routeIndex].point)
            {
            case POINT_RAW:
                moveState = RAW_PICKUP;
                break;
            case POINT_COARSE:
                moveState = COARSE_PROCESS;
                break;
            case POINT_TEMPORARY:
                moveState = TEMPORARY_PROCESS;
                break;
            default:
                moveState = POINT_WAIT;
                break;
            }
        }
        break;

    case POINT_WAIT:
        if (millis() - stateTime >= waitTimeAt(ROUTE[routeIndex].point))
        {
            // 首次在二维码区停车结束后，离开二维码区时才允许扫码。
            if (REQUIRE_QR_RESULT && routeIndex == 0 &&
                ROUTE[routeIndex].point == POINT_QR)
            {
                beginQrDepartureScan();
            }

            if (routeIndex + 1 >= ROUTE_COUNT)
            {
                moveState = FINISHED;
            }
            else
            {
                const bool leavingWorkArea =
                    isWorkAreaPoint(ROUTE[routeIndex].point);
                if (leavingWorkArea)
                {
                    // 全部物料处理完成后，同时启动机械臂和ID5储料盘复位。
                    // 机械臂复位必须先发出异步命令，储料盘等待期间会持续服务M5。
                    resetArmToWorkZero(ROUTE[routeIndex].point);
                    if (ENABLE_ARM_INITIALIZATION && armHardwareReady)
                    {
                        storageController.moveToInitialAvoidance();
                    }
                }

                const bool initialQrScanDeparture =
                    routeIndex == 0 && ROUTE[routeIndex].point == POINT_QR;
                if (initialQrScanDeparture)
                {
                    // 首次二维码扫描阶段保留横移，直接进入该路段。
                    ++routeIndex;
                    moveState = MOVE_COMMAND;
                }
                else
                {
                    // 功能区任务结束后立即转向；中心点等中转点也在发车前对准下一直线。
                    beginStraightDepartureToNextPoint();
                }
            }
        }
        break;

    case FINISHED:
        // 完成后只切换一次到统计页面，随后保持停止。
        if (!finishPageShown)
        {
            Serial_TJCHMI.print("page count\xff\xff\xff");
            finishPageShown = true;
        }
        break;
    }
}
