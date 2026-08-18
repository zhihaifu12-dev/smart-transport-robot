#ifndef FINAL_TASK_H
#define FINAL_TASK_H

#include <Arduino.h>

// 初始化最终任务机械臂。上电位置必须为：M5工作零位、M6近端、M7高位。
bool FinalTask_InitArm();

// 将储料盘舵机复位到初始避让角（当前为-91度）。可在完整机械臂初始化前调用。
bool FinalTask_ResetStorageTray();

// 到达粗加工区或暂存区时初始化数字区通信，并异步命令ID5转到视觉避让角。
// 后续数字识别只等待剩余停稳时间，不重复发送舵机命令。
bool FinalTask_PrepareDigitAreaOnArrival();

// 启停区准备动作：选区后先闭合夹爪，M5随后旋转；启动键按下后再松开。
void FinalTask_CloseGripperForStart();
void FinalTask_OpenGripperForStart();

// 行驶/作业姿态切换。收起角度单位为0.1度。
void FinalTask_SetArmStowed(bool stowed);

// 功能区任务结束后异步复位机械臂；主循环必须持续调用Service。
// rawArea=true表示刚离开原料区，false表示粗加工区或暂存区。
void FinalTask_BeginAreaArmReset(bool rawArea);
void FinalTask_ServiceArmReset();
bool FinalTask_ArmResetComplete();

// 机械臂轴语义接口。M5/M6/M7分别以0.1 degree、0.1 mm、0.1 mm为单位。
void FinalTask_MoveBaseTo(float angleTenthDegree);
void FinalTask_MoveHorizontalTo(float positionTenthMm);
void FinalTask_MoveVerticalTo(float positionTenthMm);

// 夹爪与ID5储料盘舵机语义接口。
void FinalTask_OpenGripperForGroundPick();
void FinalTask_CloseGripperForHolding();
void FinalTask_OpenGripperFully();
void FinalTask_MoveStorageToSlot(uint8_t slotNumber);

// 按colors顺序从原料转盘抓取，并分别放入storageSlots指定的储料盘。
// 抓取参数和首件固定坐标规则全部沿用ArmTest。
bool FinalTask_PickRawBatch(const uint8_t colors[3],
                            const uint8_t storageSlots[3],
                            bool secondRoundArrival = false);

// sourceSlots指定取料储料盘，targetDigits指定对应的目标数字位置。
// pickBack=true时再夹回原储料盘；reuseTemporaryCoordinates=true时跳过识别，
// 复用第一次暂存区保存的数字坐标。placeDepthReduction单位为0.1mm。
// targetColors仅用于第二次暂存区，在原坐标附近按对应颜色圆心重新校准。
bool FinalTask_ProcessDigitArea(const uint8_t sourceSlots[3],
                                const uint8_t targetDigits[3],
                                bool pickBack,
                                bool reuseTemporaryCoordinates = false,
                                float placeDepthReduction = 0.0f,
                                const uint8_t targetColors[3] = nullptr);

#endif
