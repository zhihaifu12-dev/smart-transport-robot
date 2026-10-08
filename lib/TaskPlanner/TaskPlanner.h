#ifndef REWIND_TASK_PLANNER_H
#define REWIND_TASK_PLANNER_H

#include <Arduino.h>

namespace TaskPlanner
{
constexpr uint8_t ITEM_COUNT = 6;
constexpr uint8_t ENCODED_LENGTH = 15;
constexpr uint8_t ITEMS_PER_BATCH = 3;
constexpr uint8_t FIRST_BATCH_INDEX = 0;
constexpr uint8_t SECOND_BATCH_INDEX = 1;

struct TaskCode
{
    uint8_t colors[ITEM_COUNT] = {0};
    uint8_t positions[ITEM_COUNT] = {0};
};

// 解析格式 CCC+PPP+CCC+PPP。
// 为保持原行为，颜色接受'1'到'6'，位置只接受'1'到'3'。
bool parseTaskCode(const char *encoded, TaskCode &task);

// 返回第二批某颜色在第一批中的暂存位置；找不到时保留第二批粗加工位置。
uint8_t temporaryStackPosition(const TaskCode &task, uint8_t secondBatchIndex);

struct AreaPlan
{
    uint8_t colors[ITEMS_PER_BATCH] = {0};
    uint8_t sourceSlots[ITEMS_PER_BATCH] = {0};
    uint8_t targetDigits[ITEMS_PER_BATCH] = {0};
};

TaskCode fromLegacyArrays(const int colors[ITEM_COUNT],
                          const int positions[ITEM_COUNT]);

// 原料区计划严格按照二维码颜色组排列：
// batchIndex=0读取第1组三位颜色，batchIndex=1读取第3组三位颜色。
// 三件物料按夹取先后依次存入车载1/2/3号槽；二维码第2/4组只作为目标位置。
AreaPlan makeRawPlan(const TaskCode &task, uint8_t batchIndex);
AreaPlan makeCoarsePlan(const TaskCode &task, uint8_t batchIndex);
AreaPlan makeTemporaryPlan(const TaskCode &task, uint8_t batchIndex);
} // namespace TaskPlanner

#endif
