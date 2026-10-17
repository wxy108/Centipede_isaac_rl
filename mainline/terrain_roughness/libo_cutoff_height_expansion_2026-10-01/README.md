# Libo 高度与截断扩展

<a id="contact-loss-update"></a>
## 2026-10-02 追加：rough terrain等效contact loss图

复用完整2/4/8 mm×0.25–2.25数据，新增A/B/C三张loss–roughness比例图、位移/loss随roughness的并列对照和原始missing count版；已有三张forward displacement–loss比例图保留并统一索引。五wave配色、统一圆点和淡色样本STD，双方0–6 s且保留集合相同。仅后处理，无新仿真、无PD/amplitude/物理改动；进一步加高及random-erasure仍暂停，自动跟进保持关闭。

[全部八类图及定义](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/README.md) · [逐组数值和来源核验](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/REPORT.json)

<a id="final-delivery"></a>
## 2026-10-02 本轮交付；进一步加高等待讨论

本轮自动跟进已关闭：libo已从应用删除，避免重复工作。后续物理须先讨论样本不足问题，当前不排队。

新增3150个条件全部计入：2676完整、474失败。合并旧900 rough和5 flat，共4055计划、3553完整、502失败；135组各一次剔除后保留3418。失败为492状态发散、8边界停止、2原生启动错误，不赋零、不重试数值offender。30共享spawn、既有平地cluster A/B、全173关节9/17 wave4 PD、0.5 ms物理/1 ms CPG/TGS64/16和既定资源布局保持固定。

最后×2.25的wave3.5在2/4/8 mm分别只有19/18/21条原始完整轨迹，其中末两秒≤2 mm分别1/0/1条；均不足24，按协议暂停。尚未确定物理“走不动”的高度，未启动×2.5。主动调度估算17 h16 min，总历时37 h02 min，中断间隔另记；最近续跑约50 min。无物理队列、random-erasure重启或关机。

九类结果图、48张静态图和21段新五wave视频已审阅；保留此前六段4 mm视频。所有新视频固定spawn12：90个完成画格、15个原失败占位；120帧/20 fps/6 s全解码通过，每段首中末帧和代表完整图已实际查看。保存Isaac轨迹＋原USD几何OpenGL回放，没有新增物理步。六张地形预览/输入曲线也已实审。

扩大样本没有普遍重现“低波数随短波恢复改善”；原单次发现和锚点假说保留，但不推广为规律。高高度均值受完成病例选择影响，须结合失败图及共同完整spawn配对结果。A/B/C仅为相对平地的接触差异描述。

[最终报告](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/FINAL_REPORT.md) · [完整统计/失败/原始停止判断](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/RESULTS.json) · [图视频及独立审计清单](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/FINAL_DELIVERY.json)

### 九类结果图

- [0–6 s位移 vs共同roughness](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/mean_std_vs_roughness.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/mean_std_vs_roughness.svg)
- [位移 vs高度倍率](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/mean_std_vs_height.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/mean_std_vs_height.svg)
- [4–6 s位移 vs共同roughness](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/mean_std_4_6_vs_roughness.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/mean_std_4_6_vs_roughness.svg)
- [位移 vs wave number](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/mean_std_vs_wave_number.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/mean_std_vs_wave_number.svg)
- [未完成条件数量](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/incomplete_condition_counts.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/incomplete_condition_counts.svg)
- [A：逐10 ms缺失比例](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/displacement_vs_instantaneous_loss_portion.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/displacement_vs_instantaneous_loss_portion.svg)
- [B：逐周期缺失比例](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/displacement_vs_cycle_loss_portion.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/displacement_vs_cycle_loss_portion.svg)
- [C：全窗口缺失比例](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/displacement_vs_pooled_loss_portion.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/displacement_vs_pooled_loss_portion.svg)
- [原始wave3.5停止判据](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/raw_wave3p5_stopping_criterion.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/raw_wave3p5_stopping_criterion.svg)

### 固定spawn12五wave视频索引

每格均为同一预选spawn；本轮新21段，旧4 mm六段复用保留。单段不代表组均值。

| 高度倍率 | 2 mm | 4 mm | 8 mm |
|---|---|---|---|
| ×0.25 | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff2mm_height_x0p25_five_waves_spawn12.mp4) | [视频（已保留）](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_spawn_repeats/extension_spawn30_2026-09-30/demos/libo_height_x0p25_five_waves_spawn12.mp4) | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff8mm_height_x0p25_five_waves_spawn12.mp4) |
| ×0.5 | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff2mm_height_x0p5_five_waves_spawn12.mp4) | [视频（已保留）](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_spawn_repeats/extension_spawn30_2026-09-30/demos/libo_height_x0p50_five_waves_spawn12.mp4) | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff8mm_height_x0p5_five_waves_spawn12.mp4) |
| ×0.75 | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff2mm_height_x0p75_five_waves_spawn12.mp4) | [视频（已保留）](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_spawn_repeats/extension_spawn30_2026-09-30/demos/libo_height_x0p75_five_waves_spawn12.mp4) | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff8mm_height_x0p75_five_waves_spawn12.mp4) |
| ×1 | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff2mm_height_x1_five_waves_spawn12.mp4) | [视频（已保留）](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_spawn_repeats/extension_spawn30_2026-09-30/demos/libo_height_x1p00_five_waves_spawn12.mp4) | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff8mm_height_x1_five_waves_spawn12.mp4) |
| ×1.25 | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff2mm_height_x1p25_five_waves_spawn12.mp4) | [视频（已保留）](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_spawn_repeats/extension_spawn30_2026-09-30/demos/libo_height_x1p25_five_waves_spawn12.mp4) | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff8mm_height_x1p25_five_waves_spawn12.mp4) |
| ×1.5 | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff2mm_height_x1p5_five_waves_spawn12.mp4) | [视频（已保留）](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_spawn_repeats/extension_spawn30_2026-09-30/demos/libo_height_x1p50_five_waves_spawn12.mp4) | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff8mm_height_x1p5_five_waves_spawn12.mp4) |
| ×1.75 | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff2mm_height_x1p75_five_waves_spawn12.mp4) | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/initial_4mm_h1p75_2p00/demos/libo_cutoff4mm_height_x1p75_five_waves_spawn12.mp4) | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff8mm_height_x1p75_five_waves_spawn12.mp4) |
| ×2 | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff2mm_height_x2_five_waves_spawn12.mp4) | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/initial_4mm_h1p75_2p00/demos/libo_cutoff4mm_height_x2_five_waves_spawn12.mp4) | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos/libo_cutoff8mm_height_x2_five_waves_spawn12.mp4) |
| ×2.25 | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/higher_h2p25_c2_4_8/demos/libo_cutoff2mm_height_x2p25_five_waves_spawn12.mp4) | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/higher_h2p25_c2_4_8/demos/libo_cutoff4mm_height_x2p25_five_waves_spawn12.mp4) | [视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/higher_h2p25_c2_4_8/demos/libo_cutoff8mm_height_x2p25_five_waves_spawn12.mp4) |

### 静态姿态与地形

- [libo_cutoff4mm_all_terrains_t6](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/initial_4mm_h1p75_2p00/simulation_previews/libo_cutoff4mm_all_terrains_t6.png)
- [libo_cutoff2mm_all_terrains_t6](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/simulation_previews/libo_cutoff2mm_all_terrains_t6.png)
- [libo_cutoff8mm_all_terrains_t6](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/simulation_previews/libo_cutoff8mm_all_terrains_t6.png)
- [libo_cutoff2mm_height_x2p25_five_waves_spawn12_oblique_t6](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/higher_h2p25_c2_4_8/simulation_previews/libo_cutoff2mm_height_x2p25_five_waves_spawn12_oblique_t6.png)
- [libo_cutoff4mm_height_x2p25_five_waves_spawn12_oblique_t6](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/higher_h2p25_c2_4_8/simulation_previews/libo_cutoff4mm_height_x2p25_five_waves_spawn12_oblique_t6.png)
- [libo_cutoff8mm_height_x2p25_five_waves_spawn12_oblique_t6](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/higher_h2p25_c2_4_8/simulation_previews/libo_cutoff8mm_height_x2p25_five_waves_spawn12_oblique_t6.png)
- [initial_4mm_h1p75_2p00地形预览](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/terrains/initial_4mm_h1p75_2p00/libo_cutoff_height_preview_layout_v2.png) · [roughness输入曲线](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/terrains/initial_4mm_h1p75_2p00/roughness_vs_height_cutoff.png)
- [matched_2_8mm_h0p25_2p00地形预览](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/terrains/matched_2_8mm_h0p25_2p00/libo_cutoff_height_preview.png) · [roughness输入曲线](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/terrains/matched_2_8mm_h0p25_2p00/roughness_vs_height_cutoff.png)
- [higher_h2p25_c2_4_8地形预览](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/terrains/higher_h2p25_c2_4_8/libo_cutoff_height_preview.png) · [roughness输入曲线](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/terrains/higher_h2p25_c2_4_8/roughness_vs_height_cutoff.png)

图内比例一致；×2.25地形预览的Z范围±24.5 mm，与前两阶段±21.5 mm不同。宽幅图请打开原图。静态各单图/源码/哈希在对应simulation_previews/RENDER_REPORT.json。视频生成时的manually_reviewed=false字段保留原记录，随后人工验收以FINAL_DELIVERY.json为准。

## 历史协议、运行与中途报告（仅作来源，当前状态以上方交付为准）

# Libo 高度与截断扩展（2026-10-01）

用户已启动更高地形和2/8 mm比较。已启动首批物理批次，四worker实际3/2/3/2环境的173关节PD、幅度、物理及线程原生读回通过（[启动核对](FIRST_NATIVE_STARTUP_CHECK.json)）；实际运行状态以 [STATUS.json](STATUS.json) 为准。

先运行4 mm的×1.75/×2.0（300条件），再完成2/8 mm的×0.25–×2.0（2400条件）。原4 mm六档900次与5个flat控制全部复用。此后每次增加×0.25，各截断分别依据wave3.5的原始完整轨迹停止；每个“截断×高度×wave”使用同一30套位置/朝向，尝试次数不等于完整样本数。

停止判据已由用户确认：至少24/30次完整6秒，其中至少80%在4–6秒的初始朝向前进位移≤2 mm。停止判断不应用异常值剔除，数值发散和边界停止不算卡住。完整数不足24时该截断暂停向上，标为无法判定并等待讨论；其他有完整判据的截断可继续。

平地cluster A/B、9/17 wave4全173关节PD、模型、CPU TGS64/16、0.5 ms物理及front5反馈、1 ms CPG指令保持、contact20 µm/rest0均冻结。沿用四worker×两线程，正式批次每worker3/2/3/2真实条件；native worker与原控制/物理源码相同。未优化amplitude，未做收敛或资源容量测试，未恢复random-erasure或安排关机。

2/4 mm复用已恢复短波的母表面；8 mm将历史控制的连续三角表面精确细分到0.5 mm。三档同网格、区域和底层来源，高度按mean+s×centered缩放，不统一归一RMS，不重抽相位。8 mm细分在理想连续插值下保持原表面；float32顶点及网格烹制有差别，不声称与旧粗网格轨迹逐位相同。“截断mm”是名义恢复频带，横轴roughness仍为共同38足、703对、均匀位置及全朝向平均的坡度角RMS。

- [冻结协议、停止规则与来源](PROTOCOL.json) · [夜间运行状态](STATUS.json)
- [首批300条件](initial_4mm_h1p75_2p00/simulation/campaign.json) · [准备核验](initial_4mm_h1p75_2p00/simulation/PREPARATION_CHECK.json)
- [首批地形预览](terrains/initial_4mm_h1p75_2p00/libo_cutoff_height_preview_layout_v2.png) · [地形生成记录](terrains/initial_4mm_h1p75_2p00/REPORT.json)
- 原始轨迹/所有失败分别保存在各阶段simulation/runs；完成后各阶段results与combined/vNN保存位移、A/B/C、样本数和停止判断。原有报告/图/视频不覆盖。

统计仍按每组剔除一个距0–6秒中位数最远的完整观察，再算mean/样本STD，所有波数统一点型、连线和淡色误差条。真实卡住/反向是有效行为；失败没有六秒分数，不赋零。全程保留原始样本供核对，停止判据与统计剔除分开。

补至×2.0预计约10小时主动调度；之后三档每加×0.25约需1.5–2小时，另有生成、分析和视频时间。无法预先保证所有未知停止高度均在一夜内完成。

用户于2026-10-01明确回复“授权自动跟进”后，已建立仅本轮的30分钟心跳（id libo，当前聊天）。原拒绝的授权条件已满足。自动跟进核对当前控制器，外部中断仅按原条件核对恢复，完成图/视频与QA后最终交付并关闭；运行无实质变化时安静。实际调度记录见[FOLLOWUP_AUTOMATION.json](FOLLOWUP_AUTOMATION.json)。预选spawn12渲染器已准备，图/视频与人工QA目前尚未完成，不冒充完整交付。


### 2026-10-01 external interruption and recovery

The matched 2/8 mm dispatcher stopped externally at 12:53 UTC; its exact initiating cause is unknown. The system subsequently booted at 14:24 UTC. Preserve all completed observations and native failures; the interrupted spawn12 low group (source IDs60/61/62) is unscored and restores the same three-condition layout. The two completed high groups remain scored; source66 in the other low group remains a state-divergence failure, with only its unchanged unscored peers eligible for recovery.

The first deliberate controller resume stopped before native dispatch because protected non-Python processes had reused nine historical PIDs. A narrow reviewed identity guard now accepts readable non-Python executable metadata while inaccessible Python or unknown identities still block. Seven identity cases and a live418-PID audit passed. Only the resumer orchestration source/hash changed; native worker, terrain inputs, PD, amplitudes, physics and layouts are unchanged. Original source and protocol, revision receipt, controller interruption/recovery receipts and logs are preserved in `orchestration_revision_pid_reuse_2026-10-01` and `controller_resume_history`. The first attempted resume added no physics conditions.

The current controller uses the existing `--resume` path and delegates the original stage reconciliation. Consult latest `STATUS.json`, `CONTROLLER_DISPATCH.json` and stage `DISPATCH.json` for current identities. Historical errors and timestamps are retained; the current status/receipts govern. An observed interruption gap is excluded from active dispatch time at final analysis. Other study holds and the no-shutdown rule remain.


### 2026-10-01 阶段性审阅：后台继续运行

用户明确要求后台继续跑、同时查看结果并适当生成demo，授权用已提交完成数据做串行低优先级后处理。此处仅汇总完整4mm阶段，不对仍运行的2/8mm阶段作完整结论，也不增加物理步或调整PD/amplitude/物理资源配置。

旧900加新300 rough尝试共1200，1098完整、102未完成；40组按既定规则各剔除一项，保留1058rough及5个原flat。新×1.75/×2共300次中226完整、74状态发散。×2的wave3.5与2.5保留组均值分别71.66±17.37与56.42±17.64mm；共同完成的原始17个spawn配对差12.00±27.67mm，12/17为正。×1.75配对差仍为−1.97mm，不把组均值交叉称为稳定优势；wave3在两档配对均仍弱于2.5。均值以数值完成为条件，未完成不赋零、不当物理卡住。wave3.5×2仍只有2/25达到末两秒≤2mm，未达到80%停止规则。

- [五wave位移与共同roughness](interim_2026-10-01/displacement_vs_roughness_4mm.png)
- [共享spawn配对差与未完成比例](interim_2026-10-01/paired_differences_and_incomplete_4mm.png)
- [×1.75五wave保存轨迹demo](interim_2026-10-01/demos/libo_cutoff4mm_height_x1p75_five_waves_spawn12.mp4) · [×2 demo](interim_2026-10-01/demos/libo_cutoff4mm_height_x2_five_waves_spawn12.mp4)
- [静态姿态/地形对照](interim_2026-10-01/simulation_previews/libo_cutoff4mm_all_terrains_t6.png)
- [统计与输入哈希](interim_2026-10-01/INTERIM_REPORT.json) · [图与视频审阅](interim_2026-10-01/VISUAL_REVIEW.json)

两视频沿用预选spawn12，×1.75缺wave2.5，×2缺wave1.5/3，均原数值失败、不更换spawn。120帧/20fps/6s全部解码通过，首中末帧和统计图已查看；原USD视觉模型的保存姿态回放，不是新增物理或native RTX。后处理44.65s，前后同一控制器/dispatcher持续存活；该观察不能证明吞吐完全无影响。此为阶段性交付，整轮扩展、最终图和视频仍待完成，心跳保持有效。


### 2026-10-01 evening Continue: second external interruption

The latest human Continue prompted a fresh identity check. All owned controller/dispatcher/native processes were absent; the last supervisor remained at `peer_recovery_resume01_1_13`, with1013 finished worker attempts. A subsequent system boot was observed at00:15UTC Oct2 (20:15 local Oct1). The exact exit time and initiating cause are unknown; do not use the unobserved gap as measured active dispatch time.

Two independent metadata audits confirmed six unscored conditions with no complete result, native quality-failure receipt or traceback, and no overlap with completed/offender condition keys. Preserve and restore only their exact four layouts:8mm×1.75 s15 low[75,76], high[79];8mm×2 s14 low[71,72], s15 high[78]. Prior offenders77/78/70/79 remain recorded and are not retried. The existing unchanged controller `--resume` was relaunched; current STATUS/CONTROLLER_DISPATCH/stage DISPATCH govern identity and progress. Initial preparation is still checked before any native launch. Evidence and original logs: [resume03 audit](controller_resume_history/resume_03/INTERRUPTION_AUDIT.json), [launch](controller_resume_history/resume_03/RESUME_LAUNCH.json). Scientific sources, amplitudes, PD, terrain and resource settings were not changed. Other queues remain paused, with no shutdown.

恢复首批四个native worker已实际开始运行，原2/1/2/1选中条件布局、PD/profile哈希及matched-plant检查通过；[启动核验](controller_resume_history/resume_03/NATIVE_START_VERIFICATION.json)。该记录仅证明实际启动，不将尚未完成的轨迹记为六秒成功。


### 2026-10-01 late local follow-up: ×2 completed; classified native gate fault

The initial 300 new 4 mm conditions and 2,400 new 2/8 mm conditions are accounted for: 2,433 complete and 267 failures. Preserving the old 900 rough and 5 flat observations gives combined v02 totals of 3,605 planned, 3,310 complete and 295 failed; 3,190 observations remain after 120 prescribed one-point omissions. Combined figures exist, but final manual figure/video QA remains pending. At ×2 the raw wave 3.5 stalled fractions are 2/27 (2 mm), 2/25 (4 mm) and 1/26 (8 mm); none meets 80%, so the controller entered the authorized 450-condition ×2.25 stage.

The ×2.25 stop was diagnosed as a native startup-plugin gate failure in 4 mm s02 high: source IDs 13/14 saved finite 6 s trajectories, then failed the unchanged zero-native-error assertion. Two 411 ms CPU-info/gpu-foundation dependency errors were recorded. Scientific impact is unknown, so both are terminal `native_startup_error` failures, never scored, replaced by zero, or retried. This is separate from numerical divergence and physical stalling. Original 14 artifact hashes and all 36 scientific pins were preserved.

Only resumer and analyzer failure bookkeeping changed. The explicit exclusion validator requires the exact reviewed error pair, active source IDs and original hashes; unclassified future faults still stop. The resumer's 17 rejection/accounting checks, analyzer exclusion checks and independent review passed. Original source/protocol and new revision evidence: [revision](orchestration_revision_native_failure_2026-10-01/REVISION.json). The original native worker, stage runner, PD, amplitudes, physics, terrain, layout and resource settings are unchanged. Completed earlier reports remain byte-identical.

After proving all owned processes absent, the existing controller `--resume` was relaunched. The excluded 4 mm IDs 13/14 are settled without retry; ordinary quality offenders remain failed and only unscored peers are eligible for existing recovery. New 8 mm IDs 13/14 refer to a different job. The exact first formal 3/2/3/2 layout and all 173 PD/profile/amplitude/native readbacks passed: [startup/accounting verification](controller_resume_history/resume_04/NATIVE_START_VERIFICATION.json), [fault audit](controller_resume_history/resume_04/FAULT_RECONCILIATION_AUDIT.json), [launch](controller_resume_history/resume_04/RESUME_LAUNCH.json). Startup verification does not claim complete trajectories or numerical stability. Fault-handling downtime is [recorded separately](controller_resume_history/resume_04/FAULT_PAUSE_TIMING.json); it is not another external reboot. Latest STATUS/CONTROLLER_DISPATCH/stage DISPATCH govern current identity; historical error fields may remain while current status is running. Campaign, final QA/delivery and heartbeat continue; other queues remain held, with no shutdown.


### 2026-10-02 human Continue: verified external restoration and current ETA

The latest human Continue found stale running state with all actual owned processes absent. The last ×2.25 supervisor was peer_recovery_resume01_1_16, 244 finished worker attempts. System boot was observed at 17:15:51 UTC Oct2 (13:15:51 America/New_York); exact prior exit time and cause remain unknown. Do not count the unobserved interruption gap as native runtime.

Two metadata audits support restoration of exactly seven external-unscored conditions with no results, quality receipts, tracebacks or completed/offender conflicts: 2mm s14low[71,72], s15low[76,77], s15high[79], 8mm s16low[80,81]. Original offenders70/75/78/82 and prior native-invalid4mm13/14 remain failed, never retried. Existing unchanged controller --resume was relaunched; actual native startup, exact selected2/2/1/2 layouts, all173PD and frozen profile/amplitudes passed. Evidence: [resume05](controller_resume_history/resume_05/INTERRUPTION_AUDIT.json), [startup](controller_resume_history/resume_05/NATIVE_START_VERIFICATION.json).

At the interrupted snapshot, 332/450 unique conditions were terminal:179 complete,150 state divergences,1 boundary guard,2 native startup errors.118 conditions in82 original job groups remained. Recent peer dispatch timing gives about35–60 minutes of continuous runtime for the current stage; subsequent failures and external stops can extend this estimate. It does not include final figure/video QA.

The raw wave3.5 groups already cannot reach24 complete trajectories: 2/4/8mm have18/16/19 complete,9/10/7 failed and3/4/4 pending, so their maximum possible complete counts are21/20/23. Finish the remaining authorized current-stage conditions, then the unchanged protocol should pause all three cutoffs for insufficient complete observations. This is not proof of physical stalling; no automatic×2.5 campaign or offender retry is authorized. Final21 terrain videos and nine plot categories still require review after physics exits. [Counts, bounds and ETA](controller_resume_history/resume_05/PROGRESS_AND_ETA.json). Current receipts/PID commands govern identity; historical launchPIDs and stale error fields do not. Other queues remain held; no scientific settings change or shutdown.
