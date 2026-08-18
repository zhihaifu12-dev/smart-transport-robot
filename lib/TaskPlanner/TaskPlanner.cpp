#include "TaskPlanner.h"
#include <cstring>

namespace TaskPlanner
{
bool parseTaskCode(const char *encoded, TaskCode &task)
{
    if (encoded == nullptr || std::strlen(encoded) != ENCODED_LENGTH ||
        encoded[3] != '+' || encoded[7] != '+' || encoded[11] != '+')
    {
        return false;
    }

    constexpr uint8_t COLOR_OFFSETS[ITEM_COUNT] = {0, 1, 2, 8, 9, 10};
    constexpr uint8_t POSITION_OFFSETS[ITEM_COUNT] = {4, 5, 6, 12, 13, 14};
    for (uint8_t index = 0; index < ITEM_COUNT; ++index)
    {
        const char color = encoded[COLOR_OFFSETS[index]];
        const char position = encoded[POSITION_OFFSETS[index]];
        if (color < '1' || color > '6' || position < '1' || position > '3')
        {
            return false;
        }
        task.colors[index] = static_cast<uint8_t>(color - '0');
        task.positions[index] = static_cast<uint8_t>(position - '0');
    }
    return true;
}

uint8_t temporaryStackPosition(const TaskCode &task, uint8_t secondBatchIndex)
{
    secondBatchIndex = constrain(secondBatchIndex, 3, 5);
    for (uint8_t firstBatchIndex = 0; firstBatchIndex < 3; ++firstBatchIndex)
    {
        if (task.colors[firstBatchIndex] == task.colors[secondBatchIndex])
        {
            return task.positions[firstBatchIndex];
        }
    }
    return task.positions[secondBatchIndex];
}

TaskCode fromLegacyArrays(const int colors[ITEM_COUNT],
                          const int positions[ITEM_COUNT])
{
    TaskCode task;
    for (uint8_t index = 0; index < ITEM_COUNT; ++index)
    {
        task.colors[index] = static_cast<uint8_t>(colors[index]);
        task.positions[index] = static_cast<uint8_t>(positions[index]);
    }
    return task;
}

AreaPlan makeRawPlan(const TaskCode &task, uint8_t batchIndex)
{
    AreaPlan plan;
    const uint8_t normalizedBatch = constrain(
        batchIndex, FIRST_BATCH_INDEX, SECOND_BATCH_INDEX);
    const uint8_t colorAndPositionOffset = normalizedBatch * ITEMS_PER_BATCH;
    for (uint8_t item = 0; item < ITEMS_PER_BATCH; ++item)
    {
        // 第一/第三组的第N位颜色就是本批第N个夹取目标。
        plan.colors[item] = task.colors[colorAndPositionOffset + item];

        // 车载槽位按夹取顺序固定为1、2、3，不能与二维码目标位置混用。
        plan.sourceSlots[item] = item + 1;

        // 第二/第四组三位数仅表示粗加工区和暂存区的目标位置。
        plan.targetDigits[item] = task.positions[colorAndPositionOffset + item];
    }
    return plan;
}

AreaPlan makeCoarsePlan(const TaskCode &task, uint8_t batchIndex)
{
    return makeRawPlan(task, batchIndex);
}

AreaPlan makeTemporaryPlan(const TaskCode &task, uint8_t batchIndex)
{
    AreaPlan plan = makeRawPlan(task, batchIndex);
    if (batchIndex == 0)
    {
        return plan;
    }

    for (uint8_t item = 0; item < ITEMS_PER_BATCH; ++item)
    {
        plan.targetDigits[item] =
            temporaryStackPosition(task, item + ITEMS_PER_BATCH);
    }
    return plan;
}
} // namespace TaskPlanner
