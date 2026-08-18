#ifndef REWIND_MOTION_CONFIG_H
#define REWIND_MOTION_CONFIG_H

#include <Arduino.h>

namespace MotionConfig
{
// [实机必调]
// 作用：普通路段平移速度；单位：rpm。
// 增大：缩短运行时间；风险：打滑、丢步和停车漂移增加；已验证值：90。
constexpr float MOVE_RPM = 110.0f;

// [实机必调]
// 作用：首次离开二维码区时的扫码平移速度；单位：rpm。
// 增大：扫码窗口缩短；减小：占用比赛时间增加；已验证值：60。
constexpr float QR_SCAN_MOVE_RPM = 40.0f;

// [实机必调]
// 作用：首次到达地图中心仍未取得有效二维码时，沿车头方向前后搜索。
// 单位：mm、rpm。搜索位置固定为中心点+/-200 mm，识别成功后回到中心。
constexpr float QR_CENTER_SEARCH_DISTANCE_MM = 200.0f;
constexpr float QR_CENTER_SEARCH_RPM = 20.0f;

// [实机必调]
// 作用：底盘原地旋转速度；单位：rpm；已验证值：60。
constexpr float ROTATE_RPM = 80.0f;

// [实机必调]
// 作用：底盘速度从零升到目标值的时间；单位：s；已验证值：0.8。
constexpr float ACCELERATION_TIME_S = 0.8f;

// [实机必调]
// 作用：开环坐标距离到轮组指令距离的统一倍率；无量纲；已验证值：1.05。
// 增大：实际行驶更远；风险：越过区域中心或驶出灰色车道。
constexpr float POSITION_SCALE = 1.05f;

// 作用：IMU偏航角比例补偿；无量纲；已验证值：1.00。
constexpr float IMU_ANGLE_SCALE = 1.00f;

// 单位：rad。作用：IMU清零方向到场地+Y方向的坐标变换。
constexpr float IMU_FIELD_OFFSET_RAD = PI / 2.0f;

constexpr uint32_t IMU_DISPLAY_INTERVAL_MS = 200;
constexpr uint8_t IMU_DISPLAY_DECIMALS = 2;
constexpr long IMU_DISPLAY_SCALE = 100L;

// 航向PID参数。单位：输出为rad；改动前必须重新做底盘旋转测试。
constexpr float HEADING_PID_KP = 0.3f;
constexpr float HEADING_PID_KI = 0.0f;
constexpr float HEADING_PID_KD = 0.18f;
constexpr float HEADING_PID_MAX_I = 0.1f;
constexpr float HEADING_PID_MAX_OUT = PI / 12.0f;

// 单位：rad。作用：工作区航向最终允许误差；已验证值：0.3 degree。
constexpr float FINAL_HEADING_TOLERANCE_RAD = 0.3f * PI / 180.0f;
constexpr uint8_t HEADING_STABLE_SAMPLES = 8;
constexpr uint32_t HEADING_STABLE_MS = 150;
constexpr uint8_t MAX_ROTATION_FINE_ADJUSTMENTS = 2;
constexpr float MAX_ROTATION_TRAVEL_RAD = 2.0f * PI;
constexpr uint32_t ROTATION_SETTLE_MS = 80;
} // namespace MotionConfig

#endif
