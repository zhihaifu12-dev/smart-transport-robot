# 参数调整索引

| 文件 | 负责内容 | 主要单位 |
|---|---|---|
| `include/RobotConfig.h` | 公共串口、显示屏、启动键 | bit/s、ms |
| `include/MotionConfig.h` | 平移、旋转、IMU和航向PID | rpm、s、rad |
| `include/ArmConfig.h` | M5/M6工作零位、码垛高度 | 0.1 degree、0.1 mm |
| `include/GripperConfig.h` | 夹爪与ID5储料盘公共约束 | ID、degree、ms |
| `include/VisionConfig.h` | 相机启用项和协议常量 | byte、px、ms |
| `include/RouteConfig.h` | 启停区、工作区路线和停留 | mm、rad、ms |

调整原则：

1. 先确认场景和单位，不用裸数字替换另一个场景的参数。
2. M5角度增大表示沿驱动器既有正方向旋转，不代表物理顺时针。
3. M6/M7位置增大效果以既有方向位为准；方向位不能用于补偿坐标误差。
4. 实际运动、到位稳定和额外安全等待分别调整并分别记录。
5. 每次只改一组参数，编译七环境，再进行低速单模块实机验证。
6. 储料盘`ID5`参数不得放入M5底座参数组。

兼容运行库中的场景专用已验证参数暂时原位保留。迁移这些参数前必须先建立实机
动作快照；仅靠编译无法证明碰撞边界，因此不得为了目录整齐一次性改名或合并。
