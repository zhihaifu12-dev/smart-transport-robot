# RewindProject

智能搬运机器人固件的行为保持型重构项目。

- 原始行为基线来自同级目录 `Project2026` 的当前已编译工作树。
- `src/main.cpp` 仅保留Arduino入口和应用调度。
- 正式模块位于 `lib/`，测试入口位于 `src/tests/`。
- 每个阶段的行为证据和回退点见 `docs/REFACTOR_STAGES.md`。
- `App_Vision` 是独立相机工程，本项目只维护通信协议定义，不修改相机源码。
