# RewindProject架构与运行流程

## 依赖方向

```text
src/main.cpp
  -> RobotApp                 整车有限状态机
     -> Chassis               底盘命名接口
     -> ArmController         M5/M6/M7及区域任务接口
     -> GripperController     夹爪和ID5储料盘接口
     -> TaskPlanner           任务码解析和两批任务映射
     -> Diagnostics           状态与故障日志
     -> RouteConfig           路线和场地坐标

ArmController / GripperController
  -> FinalTask
     -> RawPickupRuntime      原料视觉抓取兼容后端
     -> DigitAreaRuntime      数字区放置、取回和码垛兼容后端
        -> StorageRuntime     储料盘取放公共实现
```

`App_Vision`是独立工程。本项目只在`VisionController`和`VisionConfig`中定义既有
协议字节，不包含、修改或烧录相机端源码。

## 轴与编号

- M5：机械臂底座步进电机，位置单位`0.1 degree`。
- M6：机械臂水平伸缩轴，位置单位`0.1 mm`。
- M7：机械臂垂直升降轴，位置单位`0.1 mm`。
- ID5储料盘：总线舵机，不是M5。由`StorageController`管理。

业务层使用：

```cpp
arm.moveBaseTo(angleTenthDegree);
arm.moveHorizontalTo(positionTenthMm);
arm.moveVerticalTo(positionTenthMm);
gripperController.openForGroundPick();
storageController.moveToSlot(slotNumber);
```

## 整车状态流程

1. `START_WAIT`：等待串口屏选择启停区和PB9实体启动键。
2. `ARM_INITIALIZE`：初始化机械臂，夹爪闭合，M5收纳，重新使能底盘。
3. `START_DELAY`：等待IMU有效并完成出发前稳定等待。
4. `MOVE_COMMAND/MOVE_WAIT`：按`RouteConfig::ROUTE`下发并等待底盘线段。
5. `WAIT_QR_RESULT`：首次离开二维码区时等待合法任务码。
6. `ROTATE_COMMAND/ROTATE_WAIT`：IMU闭环航向校准，最多两次细调。
7. `RAW_PICKUP`：按当前批次颜色顺序抓三件并放入车载储料盘。
8. `COARSE_PROCESS`：按数字位置放置三件，再逆序取回储料盘。
9. `TEMPORARY_PROCESS`：第一批平放；第二批按第一批同色位置码垛。
10. `ARM_RESET_WAIT`：必要时等待异步M5/M6/M7复位完成。
11. `ARM_ERROR`：保持停车，PB9只重试当前区域任务。
12. `FINISHED`：回到抽签启停区并切换统计页面。

每次进入状态时统一打印`[STATE][ROBOT] <STATE_NAME>`。

## 等待语义

- `*_MOVE_*`或轴驱动等待：实际电机运动时间。
- `*_SETTLE_MS`：电机到位后的机械振动稳定时间。
- `*_EXTRA_WAIT_MS`：针对特定实机动作增加的安全余量。
- `RouteConfig::*_WAIT_MS`：工作区任务结束后的路线业务停留时间。

这些时间不可互相合并。兼容运行库保留了原有阻塞顺序，只有原代码已经证明安全
的M5/M6/M7离场复位继续采用异步服务。

## 行为保持边界

- 保持所有引脚、串口波特率、协议帧、方向位、路线坐标和动作顺序。
- 高层任务是可观察有限状态机；复杂单件抓放仍使用已验证兼容后端。
- 在没有实机轨迹和电流数据前，不把兼容后端中的阻塞动作强行改为并行动作。
- 后续替换单个兼容后端时，必须先保留旧后端、全环境编译并做实机A/B验证。
