# 行为保持型重构记录

本文件记录每个可回退阶段。所有编译命令只使用 `pio run`，不执行上传。

## 基线：d644107

- 来源：`Project2026` 当前工作树的机械复制。
- 原项目保持不变，`App_Vision` 未复制、未修改。
- 七个PlatformIO环境全部编译成功。

## 阶段1：应用入口和目录边界

改动：

- `src/main.cpp` 只调用 `RobotApp_Setup()` 和 `RobotApp_Loop()`。
- 最终任务状态机迁入 `lib/RobotApp`，代码内容和执行顺序不变。
- 测试入口迁入 `src/tests`，环境名称保持不变。
- `FinalTask` 的内部兼容引用仅更新路径。

行为保持证据：

- 引脚、串口、协议、坐标、方向和所有动作参数未修改。
- 七个环境全部编译成功。
- 本阶段只改变文件位置与入口符号名。

风险与实机验证：

- 编译不能验证机械零点、夹爪碰撞边界和相机现场识别。
- 旧任务内部仍包含阻塞式动作，后续阶段以兼容层逐步替换。

## 阶段2：配置、计划和正式控制器

改动：

- 新增六个项目级配置头，所有新代码显式标明距离、角度和时间单位。
- `TaskPlanner`接管任务码校验、两批物料映射和同色码垛位置计算。
- 新增`Chassis`、`ArmController`、`GripperController`、`StorageController`。
- 新增`VisionController`协议编解码，不修改`App_Vision`。
- 状态切换使用统一`Diagnostics`日志。

行为保持证据：

- 配置值从原代码逐值复制，任务码位置仍只接受1..3。
- 控制器当前委托原函数，不改变方向、协议或动作顺序。
- 七个环境全部编译成功。

## 阶段3：测试入口与正式实现解耦

改动：

- `ArmTest/StorageTest/PickStorageTest/PutTest/MoveOnlyTest/LightTest`均为薄入口。
- 已验证动作实现迁入`RawPickupRuntime/StorageRuntime/DigitAreaRuntime`。
- `FinalTask`不再反向包含`src/tests`。
- `moveonlytest`通过环境宏复用正式`RobotApp`，不包含`.cpp`实现。

行为保持证据：

- 迁移采用同一实现文本，保留对象构造、初始化和阻塞等待顺序。
- `pickstoragetest`仍通过编译宏选择原`PICK_STORAGE_TEST`分支。
- 七个环境全部编译成功。

仍需实机验证：

- M5/M6/M7零点和方向；ID5储料盘三个槽位角度。
- 原料转盘颜色追踪、数字圆环定位和第二层同色校准。
- 三分钟完整路线、供电压降和机械碰撞边界。
