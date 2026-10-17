# PI Reading Index

这套文档只解释源码，不修改源码。源码行号以本 clean release 为准。

1. `00_PI_MASTER_RUNBOOK.md`：从零接手、环境、运行、demo、输出和排错。
2. `01_RUNTIME_LINE_BY_LINE.md`：model、USD、terrain/arena、gait、controller、env 逐段逐行。
3. `02_RL_CONFIG_AND_TOOLS_LINE_BY_LINE.md`：PPO、训练、评估、sweep、自检及所有配置。
4. `03_OBSERVATION_REWARD_ACTION_REFERENCE.md`：34D observation、纯 forward reward、1D action 的公式和证据边界。
5. `04_RESULTS_AND_DEMO_GUIDE.md`：现有结果、图表阅读、demo 生成和科研结论纪律。
6. `05_VIDEO_RENDERING_PLAYBOOK.md`：内部核验离线视频与正式 Omniverse 导出的严格分工、审计和排错。

建议第一次按 `00 -> 01 -> 03 -> 02 -> 04 -> 05` 阅读。第一次只想跑起来时，读 `00` 即可。

PI 最短路径：先打开根目录 `PI_HANDOFF.md` 指定的 canonical 三地形
高保真 panel；随后运行 `python scripts\check_project.py` 和 `pytest -q`。只想理解
模型与实验结论时，到这里即可；准备重跑仿真再进入 `00`，准备解释
observation/reward 再读 `03`，准备制作正式视频才读 `05`。

当前范围、phase 修正溯源与 Clean 独立性边界：`07_ORGANIZATION_AND_INTEGRITY.md`。
