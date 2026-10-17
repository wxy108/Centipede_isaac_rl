# 统一 PD：平地与岩面 rough terrain 新仿真

已完成 6 个波数 × 2 种地形，每个条件完整 12 s、24,000 个物理步。使用原 common.usda 身体模型；沿用之前六组纯运动学优化幅度，本次没有重搜幅度、收敛性或最大并行容量。

[平地视频](../demos/flat_demo/flat_unified_pd.mp4) · [rough terrain 视频](../demos/rough_demo/rough_unified_pd.mp4)

## 与选定 wave 4 demo 的更新节奏

与 9 月 17 日完整 wave-4 CPU rough-terrain demo 相同：物理每 0.5 ms 一步，CPG 位置和速度指令每 1 ms 更新并保持两个物理步，front5 world-pitch 反馈每 0.5 ms 重算。1 ms 不表示 PD 只每 1 ms 执行一次。原入口 `cpu_demo_12s/worker.py` 第 155–171 行显示这一更新顺序；新执行入口见下面的代码快照。

## 统一参数与检查

全部条件的 173 个关节使用同一完整 wave-4 PD 表，包括 front5 taper 和 yaw gains；原生 getter 读回逐位一致。CPU PhysX、TGS 64/16、contact offset 20 µm、rest 0、摩擦 0.5；其余场景设置依统一 profile。4 个 worker，各 2 个 Kit/PhysX 线程、3 个真实条件，共 12 条件的部分批次。数值库线程固定为 1；PXR 启动限制 2，运行时读回 28，与既有策略一致。PXR、Kit 和操作系统线程数不是同一概念。

12 条轨迹逐物理步检查位置及速度的有限性，全部完成；原生日志错误列表为空；环境间机器人和私有地形碰撞隔离通过。每 10 ms 保存所有 251 个刚体的位姿、速度，以及关节状态。

## 地形与视频

rough 使用此前 Libo-9-3-1-3 seed1501、512×512、2 mm 网格的岩面统计生成样本（由实测岩面的统计特征生成，并非原始实测扫描直接导入）。高度范围 −6.05 至 +10.49 mm，标准差约 3.19 mm。六波数使用同一文件与起点。视频采用与物理碰撞完全相同的高度及三角网格，未放大高度。

视频从本次新模拟轨迹离线 OpenGL 渲染原 USD 外观；每段 12 s、20 fps、1 倍速、240 帧，两组共用相机比例。相机跟随身体，平地网格间隔 10 mm。渲染没有再次积分物理；全部帧的身体绑定和画面边界检查通过，并完成全片解码检查。

## 运动记录（4–12 s）

下表记录沿世界 −X 方向的质心位移；只是本次单地形/单起点观察，不是动力学最优幅度排名。侧向运动另存于机器可读记录。

| 波数 | Body 幅度 ° | Leg yaw 幅度 ° | 平地 −Δx / mm | rough −Δx / mm |
|---|---:|---:|---:|---:|
| 1.5 | 32.0083 | 24.7026 | 359.20 | 48.50 |
| 2 | 27.8208 | 29.0004 | 350.61 | 204.51 |
| 2.5 | 22.2417 | 34.9760 | 297.40 | 233.73 |
| 3 | 17.7042 | 33.3329 | 253.56 | 196.18 |
| 3.5 | 7.5833 | 43.2880 | 215.49 | 190.84 |
| 4 | 2.6990 | 41.8512 | 192.51 | 166.82 |

## 输入、代码与原始数据

[仿真核验记录](simulation_verification.json) · [视频核验记录](video_verification.json) · [实际执行的仿真源码](executed_source/unified_trial/simulation.py) · [启动入口](executed_source/launch.py) · [任务与输入表](executed_source/campaign.json) · [统一 PD](executed_source/pd_wave4_firm20.json) · [反馈公式](executed_source/unified_trial/world_pitch.py)

`executed_source` 保存本次执行源码、四份 plan、完整 rows、配置、参数读回及日志。幅度来自 plan 的 `joint_cells` / `body_factors` / `leg_cluster_factors`，在 `_scale_commands` 同时缩放原 CPG 位置和速度目标；统一 PD 在 `_prepare_plant` 检查完整名称映射后写入。没有旧 PD 回退。

原始轨迹及接触数据目录：`C:\Users\wxy22\Documents\Codex\2026-09-28\read-conversation-centipede-catchup\work\unified_demo_2026-09-28\runs\v4`。重复执行入口为 `C:\Users\wxy22\Documents\Codex\2026-09-28\read-conversation-centipede-catchup\work\unified_demo_2026-09-28\launch.py`，每次须指定新 `--attempt` 名称。

启动故障 v1（Windows 日志句柄）、v2（固定依赖的沙箱读取权限）、v3（USD 原点整数类型）均在物理积分前停止并保留记录；修正后 v4 四个 worker 全部成功。源快照里一处继承的反馈注释误写 “1 ms here”，实际代码和读回均为 0.5 ms；可重用源码已只修正该注释，AST 不变，见 SOURCE_NOTE.json。历史结果及 Clean 未改写。
