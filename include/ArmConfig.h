#ifndef REWIND_ARM_CONFIG_H
#define REWIND_ARM_CONFIG_H

#include <Arduino.h>

namespace ArmConfig
{
// 单位：ms。作用：驱动板上电后等待电源和通信稳定。
// 这是上电等待，不是电机实际运动时间。
constexpr uint32_t POWER_STABILIZE_MS = 1000;

// 单位：0.1 degree。明确指M5机械臂底座步进电机，不是ID5储料盘舵机。
constexpr float M5_WORK_ANGLE = 0.0f;
constexpr float M5_STOW_ANGLE = 1350.0f;

// M6水平轴全流程软件负向限位；单位：0.1 mm。
// -50表示最多向负方向移动5 mm。风险：继续减小可能撞击机械零位。
constexpr float M6_MIN_POSITION_TENTH_MM = -60.0f;

// [实机必调] 单位：0.1 degree(M5)和0.1 mm(M6)。
// 作用：两批物料进入三个工作区时的机械臂工作零位。
// 风险：M5角度或M6伸长过大会碰撞场地装置。
constexpr float WORK_M5_ANGLES[2][3] = {
    {0.0f, -200.0f, -80.0f},
    {0.0f, -360.0f, -180.0f}};
constexpr float WORK_M6_LENGTHS[2][3] = {
    {80.0f, 120.0f, 50.0f},
    {80.0f, 180.0f, 160.0f}};

// [实机必调]
// 作用：第二批暂存区码垛时减少M7下降深度；单位：0.1 mm。
// 增大：第二层放置位置更高；风险：过大可能悬空跌落；已验证值：700。
constexpr float TEMPORARY_STACK_HEIGHT = 700.0f;

// 原料区放入储料盘与粗加工区回存物料共用的唯一参数源。
// 修改这里会同时影响两条入盘路径，禁止在各运行时中再复制独立数值。
namespace StorageReturn
{
constexpr uint8_t SLOT_COUNT = 3;

// [实机必调] ID5储料盘舵机角度；单位：degree。
constexpr float ID5_SLOT_ANGLES_DEG[SLOT_COUNT] = {-79.0f, -5.0f, 78.0f};

// [实机必调] M5机械臂底座入盘角度；单位：0.1 degree。
constexpr float M5_SLOT_ANGLES_TENTH_DEG[SLOT_COUNT] = {
    -1080.0f, -1100.0f, -1090.0f};

// [实机必调] M6入盘伸长位置；单位：0.1 mm。
// 可使用小范围负值：-10表示向负方向移动1 mm，-20表示向负方向移动2 mm。
constexpr float M6_SLOT_POSITIONS_TENTH_MM[SLOT_COUNT] = {
    0.0f, -20.0f, -20.0f};

// [实机必调]
// 作用：物料放入储料盘时M7的下降深度；单位：0.1 mm。
// 增大：夹爪下降更深；风险：过大会碰撞储料盘；粗加工区已用值：380。
constexpr float M7_PLACE_POSITION_TENTH_MM = 345.0f;

// M7向储料盘下降时的TTL运动参数。
constexpr uint16_t M7_PLACE_VELOCITY = 20000;
constexpr uint8_t M7_PLACE_ACCELERATION = 100;

// 到达入盘位置后、松开夹爪前的额外稳定时间；单位：ms。
constexpr uint32_t PLACE_ARRIVAL_SETTLE_MS = 40;

// ID4夹爪释放物料时的角度；单位：degree。
constexpr int GRIPPER_RELEASE_ANGLE_DEG = -50;
} // namespace StorageReturn
} // namespace ArmConfig

#endif
