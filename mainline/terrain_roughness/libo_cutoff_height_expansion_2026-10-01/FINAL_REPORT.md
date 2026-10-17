# Libo 截断与高度扩展：本轮交付（2026-10-02）

本轮3150个新增条件已全部计入，九类科学图、48张静态预览及21段新视频已完成核验。物理队列为空；更高高度因完整样本不足暂停，物理stall阈值仍未确定。统计与来源均经过独立审计，旧报告和原始失败保留。

本轮新增3150个条件：2676个完成6 s，474个失败。保留旧900个rough条件和5个flat对照后，共4055个计划条件，3553个完整、502个失败；各rough组按原规则剔除一个距0–6 s位移中位数最远的完整观察后，保留3418个统计观察。失败不记为零位移，恢复同伴的重复执行不增加独立样本数。两例原生启动错误保留为未验证失败，其已有6 s数据不计分、不重试。[合并数据与失败记录](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/RESULTS.json)

全部波数1.5、2、2.5、3、3.5复用同一套30个spawn位置和朝向。各波数沿用既有平地cluster优化的A/B，未重新调幅；全部173关节跨trial、波数和地形使用同一套9/17 wave-4 PD。CPU PhysX TGS64/16，物理及front5反馈0.5 ms，CPG指令保持1 ms，contact offset20 µm/rest0；四worker各两线程，正式批次3/2/3/2个实际环境，其他模型、控制及接触设置固定。[固定配置](X:/Projects/Centipede_IsaacLab_RL/mainline/trial_profile.json) · [PD表](X:/Projects/Centipede_IsaacLab_RL/mainline/pd_wave4_firm20.json) · [本轮协议](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/PROTOCOL.json)

2/4/8 mm指名义短波截止档，不是roughness数值。各档固定频带后改变地形高度；共同0.5 mm网格保留来源相位，8 mm通过细分历史连续三角面构建。有限精度及mesh cooking差异仍需保留说明。roughness横轴是固定中立38足、703对足点的坡度角RMS，在共同场地区域均匀采样位置并平均全部朝向，所有波数共用。前向位移为整机质量加权COM沿各自初始前进方向的有符号投影。

本轮在×2.25结束后，三个截止档均因完整样本不足暂停继续加高；不能判定wave3.5物理性“走不动”，也未自动启动×2.5。

| 截止档 | 原始完整wave3.5条件 | 4–6 s位移≤2 mm | 停止判断 |
|---|---:|---:|---|
| 2 mm | 19/30 | 1/19 | 完整数不足24，暂缓讨论 |
| 4 mm | 18/30 | 0/18 | 完整数不足24，暂缓讨论 |
| 8 mm | 21/30 | 1/21 | 完整数不足24，暂缓讨论 |

已确认的stall判据要求至少24个完整条件，且其中至少80%的末两秒有符号前向位移≤2 mm。判据使用原始完整样本，不应用统计剔除；数值发散、边界保护和启动错误不算stall。[当前状态与判断](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/STATUS.json)

主要比较显示，较高波数在部分高地形档具有优势，但并非所有波数或截止档都按同一顺序排列：

- 4 mm×2下，wave3.5相对2.5的原始共享spawn配对差为+12.00±27.67 mm，17对中12对为正；wave3相对2.5则为−7.99±26.38 mm。这里的±为配对差的样本STD。
- ×1下，wave2在2/4/8 mm的组均值分别为122.88/123.61/135.21 mm；2 mm相对8 mm的原始共同完整spawn配对差为−12.83±21.53 mm，n=28。这不支持“恢复更多短波必然提高低波数性能”的普遍结论。
- ×1.75下，wave2的2 mm组均值比8 mm约高14.3 mm，但共同完整的15个spawn配对差仅−0.70±19.03 mm。组间完成集合不同，直接比较均值可能掺入完成病例选择偏差；配对后也仍只代表双方都完成的条件。

原短波恢复旁支发现保留：旧单次Libo wave1.5在原版约8 mm、恢复4 mm、恢复2 mm的0–6 s位移分别为14.54、2.43、113.79 mm，说明特定位置上存在明显恢复，且不单调。Narvik没有同样的持续趋势。扩大到30个共享spawn后，恢复现象并非普遍成立；“短波增加有效锚点”仍是假说，尚未由锚点、滑移或接触持续时间证实。旧8 mm网格为2 mm，也不能与本轮共同0.5 mm网格混为同一控制。[原旁支记录和六段视频](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/README.md#side-finding-shortwave) · [原位移数据](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/shortwave/displacement_roughness/RESULTS.json)

contact-loss图沿用0–6 s窗口，按同wave平地、同CPG时刻的38足接触作为expected support；missing指平地该足有接触、rough该足无接触。三种横轴均转换为百分比：

- A：每个有效10 ms时刻计算missing足数/expected足数，再等权平均；expected为0的时刻跳过。
- B：每个名义1 s周期内先汇总missing/expected，再对六个周期比例等权平均。
- C：整个0–6 s窗口的总missing样本/总expected样本，按expected足时样本数量加权。

先计算每个trial的比例，再计算组mean/STD；位移和A/B/C使用同一保留集合。它们描述相对平地的接触差异，包含相位变化，不能直接当作独立随机擦除概率，也不能仅凭曲线交点证明因果机制。

调度的主动运行时间估算为17 h16 min，总历时37 h02 min；最近一次恢复约50 min。观察到的checkpoint至resume间隔合计约19 h33 min，确切退出时刻未知，因此不能把总历时当成连续仿真耗时。这些数值是调度层时间，不是各环境物理步CPU时间之和；最终图形、视频及人工QA耗时另记。

以下材料已验收，完整链接见[主线交付索引](README.md#final-delivery)。

| 材料 | 目录或内容 | 验收状态 |
|---|---|---|
| 九类科学图 | [combined/v03](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03)：0–6 s与4–6 s位移–roughness、位移–高度、位移–wave、A/B/C三图、失败数量、原始wave3.5停止判据 | 数据与九类图已审阅 |
| 第一阶段视频 | [4 mm×1.75/×2 demos](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/initial_4mm_h1p75_2p00/demos)，2段 | 全解码及首/中/末帧实审通过 |
| 匹配截止档视频 | [2/8 mm×0.25–×2 demos](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/matched_2_8mm_h0p25_2p00/demos)，16段 | 全解码及首/中/末帧实审通过 |
| ×2.25视频 | [2/4/8 mm demos](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/higher_h2p25_c2_4_8/demos)，3段 | 全解码及首/中/末帧实审通过 |

21段视频固定使用预选spawn12，105个画格中90个有验证完整轨迹、15个显示原失败，不换成其他成功spawn、不填零位移。视频仅回放保存的Isaac轨迹，使用原USD视觉几何的VTK图形，不增加物理步；20 fps、1倍速，保存姿态覆盖0–5.95 s，静态图使用6.00 s。各阶段内相机尺度统一，阶段间不保证完全相同。21段视频的120帧均完整解码通过；每段首/中/末帧和代表完整画面已实际目视审阅。所有48张PNG均通过读取/哈希检查，六张汇总图及代表双视角实审通过。人工检查代表帧，不声称逐帧目视所有2520帧。

随机擦除、Newton/soft-leg、Clean采用仍暂停；无关机安排。当前后续是讨论完整病例不足的处理；不能把本轮质量暂停解释为已经达到物理stall。


验收和完整来源清单：[FINAL_DELIVERY.json](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/FINAL_DELIVERY.json)。最终后处理未新增物理步，未改冻结的科学输入；九类图、全视频及地形图审阅记录保存在该清单，旧阶段/合并报告哈希未变。×2.25的三张窄幅汇总原稿标题有裁剪，保留生成来源；交付索引使用已生成并实审的完整斜视图，不新增图或更改渲染源码。

本轮自动跟进libo已关闭；没有继续排队或自动加高。
