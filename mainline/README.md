# Centipede 当前主线

**最新交付（2026-10-02）：**Libo 2/4/8 mm已尝试至高度×2.25。本轮新增3150条件，2676完整、474失败；合并4055计划、3553完整、502失败、3418统计观察。九类图、48张静态图及21段新固定spawn12视频已核验；本轮自动跟进已关闭。×2.25 wave3.5完整仅19/18/21，不足24，进一步加高暂停等待讨论；没有证明物理stall。主动调度估算17h16，历时37h02。无物理队列；random-erasure/Newton/Clean及关机均不启动。 [最终图、全部demo与证据](terrain_roughness/libo_cutoff_height_expansion_2026-10-01/README.md#final-delivery)。

**短波旁支更新：**30共享spawn结果呈混合/非单调，不能把原单次短波恢复推广为普遍规律，锚点机制仍未验证。完整病例均值要结合失败与配对结果解读。

## 历史结果与暂停工作（以下按日期保留，执行以顶部最新状态为准）

**此前完成（2026-10-01）：**现有4 mm六档Libo已补到每组30个计划spawn：新增600条件全部尝试，580完成6 s、18数值发散、2边界保护停止；合并原300rough和5flat后，共905个计划条件、877完整、28未完成，按每个rough组一次剔除后保留842rough＋5flat＝847个统计观察。 七张图、六段spawn12视频和证据统一在[30-spawn主线](terrain_roughness/README.md#libo-spawn30-extension)。全部结束，无物理队列或关机安排；其他任务以最新启动条目为准；random-erasure继续暂停。

**暂停任务（2026-09-30）：**[轻量contact erasure与两种等效率](terrain_roughness/README.md#contact-erasure-100)随机队列暂停，等待用户选择是否接受延长：280唯一计划条件中140完成、65数值发散、75仍未计分（含10唯一确定性端点）。两个明确批准的同十环境端点对照已完成20物理实例，不增加随机样本。发散已有只读复核，PD/A/B/物理未改，暂无原生worker。完整4060方案已归档暂停；高度/截断rough扩展继续只记录。

**后续higher-height/cutoff仅记录，等待用户启动；现有六档30-spawn已单独授权完成。**[统一研究记录](terrain_roughness/README.md#side-finding-shortwave)收录2.5/3视频与统计的核对、每组总计30个共享spawn、×1.75/×2.0后按×0.25继续到wave3.5走不动，以及不同2/4/8 mm截断地形的同类试验。停跑阈值与截断比较口径待讨论；剩余higher-height/cutoff扩展没有待跑campaign、队列或自动化。

历史十spawn结果：[Libo六档高度、统一点型/淡色STD及每组一次剔除](terrain_roughness/README.md#libo-six-heights)。六档 Libo 的300个条件及五个平地对照已尝试，297个完成6秒、8个提前停止；按用户规则另剔除30个完整观察用于派生统计，保留267个。

已完成流程阶段：[真实地形 roughness 的 fixed/per-gait 双套流程](terrain_roughness/README.md)。9/17 原版／恢复至 4 mm／恢复至 2 mm 的 Libo、Narvik 六张地形与配套图已完成；六波数各 6 s 实际 Isaac Lab 预览已全部完成（36 条件），20 张仿真预览图、六段 6 秒 demo 视频及完整核验均已保存。平地 cluster 幅度与全173关节 wave-4 PD 保持固定；已完成六表面的固定足点 roughness 与三张前向位移对比图（位置及全部朝向平均），per-gait sweep 尚未开始。

旁支发现与后续计划统一记录：[恢复短波细节后低波数前进能力回升及“有效锚点”假说](terrain_roughness/README.md#side-finding-shortwave)，附数值、图、配置来源和三组六波数完整视频。

前一轮高度扫描：[固定 4 mm 截断、七档高度的实际试跑结果与全部视频](terrain_roughness/README.md#fixed4mm-height-results)。七档高度 × 两站点 × 六波数的 84 个条件均已尝试；78 个完成 6 秒，2 个状态发散，4 个被同组保护提前中断。三张位移图与 14 段 demo 已完成，其中 12 段含六波数，Libo 1.5/2 倍的两段只含完成的三个高波数。 机器人 A/B 和全部 PD 固定；失败记录与有效结果分开保存，未新增调参或收敛测试。此前短波恢复对照作为旁支保留。

以下为已完成的统一 PD 基线。

六波数 × 平地/rough，每条件 12 s，已完成。使用原 common.usda 模型与之前六组运动学幅度；没有重做优化或收敛测试。

- [平地 demo](demos/flat_demo/flat_unified_pd.mp4) · [rough demo](demos/rough_demo/rough_unified_pd.mp4)
- [统一 trial 标准](TRIAL_STANDARD.md) · [固定全部 173 关节 PD](pd_wave4_firm20.json) · [运行 profile](trial_profile.json)
- [运行清单](campaign.json) · [启动代码](launch.py) · [物理与控制代码](unified_trial/simulation.py) · [反馈公式](unified_trial/world_pitch.py)
- [完整仿真检查](evidence/simulation_verification.json) · [视频检查](evidence/video_verification.json) · [实际执行源码快照](evidence/executed_source/unified_trial/simulation.py)

原始轨迹、关节状态和接触记录在 `runs/v4/`。`evidence/executed_source` 是物理运行时的原样快照；当前入口只迁移路径和修正继承注释，没有重新模拟，也没有改写旧结果。

使用既有 `isaaclab_50` Python 运行 `launch.py flat_low --check` 可只检查输入。其他任务名是 rough_low、flat_high、rough_high。真正新运行必须显式选择任务及未使用过的 `--attempt` 名称；四任务并行入口为 `supervise.py`。旧结果永不覆盖。

时序与选定的历史 wave 4 一致：物理和 front5 反馈 0.5 ms，CPG 位置/速度指令每 1 ms 更新并保持两步。每 10 ms 保存轨迹；视频 20 fps、1倍速。

rough 是基于实测岩面统计特征生成的 Libo seed1501 高度场，并非原扫描直接导入；视频使用与碰撞相同的三角网格。只观察本次固定场景表现，不据此宣称动力学最优幅度。

运行所需历史模型/命令/辅助模块在 `X:\Projects\_archive\Codex` 中按原始路径归档，原生环境在现有 Anaconda 中。profile 内历史来源路径是证据字段；当前 launcher 明确加载本目录固定 PD。旧 C 项目路径已切换为指向 X 盘的兼容入口；未核验历史依赖及当前会话残留明确保留，历史 receipts 不批量改写。
