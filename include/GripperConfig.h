#ifndef REWIND_GRIPPER_CONFIG_H
#define REWIND_GRIPPER_CONFIG_H

#include <Arduino.h>

namespace GripperConfig
{
// 语义化动作名称由GripperController提供。本文件只保存跨场景公共约束。
// M5表示机械臂底座；ID5专门表示储料盘总线舵机，二者不得混用。
constexpr uint8_t STORAGE_TRAY_SERVO_ID = 5;

// [实机必调]
// 作用：第二轮进入原料区后，第一件物料识别和抓取前的ID4夹爪张开角度。
// 单位：degree；减小会继续张开夹爪，过小可能达到舵机机械极限。
// 已验证初始值：-90。第一轮及第二轮后续物料仍使用原有张开参数。
constexpr int SECOND_ROUND_RAW_ENTRY_OPEN_ANGLE_DEG = -90;
} // namespace GripperConfig

#endif
