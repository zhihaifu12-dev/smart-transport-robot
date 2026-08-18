#ifndef REWIND_VISION_CONFIG_H
#define REWIND_VISION_CONFIG_H

#include <Arduino.h>

#ifndef MOVETEST_ENABLE_ARM
#define MOVETEST_ENABLE_ARM 1
#endif
#ifndef MOVETEST_ENABLE_CAMERA
#define MOVETEST_ENABLE_CAMERA 1
#endif
#ifndef MOVETEST_REQUIRE_QR
#define MOVETEST_REQUIRE_QR 1
#endif

namespace VisionConfig
{
constexpr bool ENABLE_ARM_INITIALIZATION = MOVETEST_ENABLE_ARM;
constexpr bool ENABLE_CAMERA = MOVETEST_ENABLE_CAMERA;
constexpr bool REQUIRE_QR_RESULT = MOVETEST_REQUIRE_QR;
constexpr bool ENABLE_VISION_PICKUP =
    ENABLE_ARM_INITIALIZATION && ENABLE_CAMERA;

// [实机必调]
// 作用：数字识别期间连续未识别到目标数字时，触发一次M6径向搜索。
// 单位：ms；增大可减少搜索动作，减小可更快扩大相机搜索范围。
// 风险：过小会造成M6频繁运动，影响相机画面稳定；已验证初始值：1000。
constexpr uint32_t DIGIT_SEARCH_M6_INTERVAL_MS = 1000;

// [实机必调]
// 作用：每次超时后，M6交替向外伸出、向内收回的相对距离。
// 单位：0.1 mm；50表示5 mm。增大会扩大搜索范围，但也增大机械干涉风险。
// 实际目标始终受M6软件正负限位约束；已验证初始值：50。
constexpr float DIGIT_SEARCH_M6_STEP_TENTH_MM = 250.0f;

// 与App_Vision保持一致的补光灯协议。只定义协议，不修改相机工程。
constexpr uint8_t FILL_LIGHT_HEADER = 0xA5;
constexpr uint8_t FILL_LIGHT_ON = 0x01;
constexpr uint8_t FILL_LIGHT_OFF = 0x00;
constexpr uint8_t FILL_LIGHT_TAIL = 0x5A;

// -------------------- 数字1预测坐标补偿 --------------------
// [实机必调]
// 作用：数字1由数字2、3几何预测后，额外修正M5底座角度。
// 单位：0.1 degree；正值增大M5目标角，负值减小M5目标角。
// 风险：绝对值过大会使夹爪偏离数字中心或超过M5软件限位。
// 已验证值：0（保持原预测行为）。
constexpr float DIGIT1_PREDICT_M5_COMPENSATION_TENTH_DEG = -15.0f;

// [实机必调]
// 作用：数字1由数字2、3几何预测后，额外修正M6水平伸长位置。
// 单位：0.1 mm；正值使M6继续伸长，负值使M6向内收回。
// 风险：绝对值过大会偏离数字中心或碰撞M6机械行程端点。
// 已验证值：0（保持原预测行为）。
constexpr float DIGIT1_PREDICT_M6_COMPENSATION_TENTH_MM = 27.0f;
} // namespace VisionConfig

#endif
