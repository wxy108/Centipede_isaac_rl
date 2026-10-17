# 真实地形 roughness：当前主线

**阶段性图与demo（2026-10-01）：**4mm高度×0.25–2的五wave曲线、配对差/未完成率和×1.75/×2预选spawn12视频已审阅；2/8mm后台继续运行。见[阶段性结果](libo_cutoff_height_expansion_2026-10-01/README.md#2026-10-01-阶段性审阅后台继续运行)。此为中途审阅，尚非整轮交付。

**本轮正在运行（2026-10-01）：**Libo×1.75以上高度与2/8 mm截断扩展，30共享spawn、固定A/B/PD/物理。wave3.5停跑标准已确认：至少24完整、80%末两秒≤2 mm；数值失败不当卡住。先共同补到×2，再按×0.25继续。实际状态和新证据见[高度/截断扩展](libo_cutoff_height_expansion_2026-10-01/README.md)。random-erasure继续暂停，无关机。

**历史暂停任务（2026-09-30）：**[contact erasure轻量版](#contact-erasure-100)随机队列暂停，等待用户选择延长耗时；280唯一计划条件，两个已批准的端点对照已完成。完整4060方案已归档暂停。更高地形及其他截断rough扩展仍只记录，统一在[研究入口](#side-finding-shortwave)；本次现有六档的30-spawn补齐已完成。

当前主线：现有4 mm六档Libo已补到每组30个计划spawn：新增600条件全部尝试，580完成6 s、18数值发散、2边界保护停止；合并原300rough和5flat后，共905个计划条件、877完整、28未完成，按每个rough组一次剔除后保留842rough＋5flat＝847个统计观察。

<a id="libo-spawn30-extension"></a>
## 现有4 mm六档补到30个spawn（已完成）

现有4 mm六档Libo已补到每组30个计划spawn：新增600条件全部尝试，580完成6 s、18数值发散、2边界保护停止；合并原300rough和5flat后，共905个计划条件、877完整、28未完成，按每个rough组一次剔除后保留842rough＋5flat＝847个统计观察。

固定高度×0.25/0.5/0.75/1/1.25/1.5，wave1.5/2/2.5/3/3.5；旧spawn0–9和原8失败保留，新增共享spawn10–29。既有平地cluster A/B、9/17 wave-4全173关节PD、CPU TGS64/16、0.5 ms物理/front5反馈、1 ms CPG保持、contact20 µm/rest0及模型均未改变。原始native worker和物理/控制源码逐字相同；四worker×两线程和正式批次3/2环境布局沿用，无收敛、资源搜索或新优化。

- [0–6 s位移 vs共同roughness](libo_spawn_repeats/extension_spawn30_2026-09-30/results/mean_std_vs_roughness.png) · [4–6 s位移](libo_spawn_repeats/extension_spawn30_2026-09-30/results/mean_std_4_6_vs_roughness.png) · [位移 vs波数](libo_spawn_repeats/extension_spawn30_2026-09-30/results/mean_std_vs_wave_number.png)
- [A/B/C contact-loss比例并列图](libo_spawn_repeats/extension_spawn30_2026-09-30/results/loss_portion_metrics_comparison.png) · [A逐10 ms比例平均](libo_spawn_repeats/extension_spawn30_2026-09-30/results/displacement_vs_instantaneous_loss_portion.png) · [B逐周期比例平均](libo_spawn_repeats/extension_spawn30_2026-09-30/results/displacement_vs_cycle_loss_portion.png) · [C全窗口总比例](libo_spawn_repeats/extension_spawn30_2026-09-30/results/displacement_vs_pooled_loss_portion.png)
- [完整原始值、mean/STD、剔除项、失败、配对差和源哈希](libo_spawn_repeats/extension_spawn30_2026-09-30/results/RESULTS.json) · [逐时刻前向位移](libo_spawn_repeats/extension_spawn30_2026-09-30/results/forward_timeseries.npz)
- [锁定输入/30共享spawn/授权](libo_spawn_repeats/extension_spawn30_2026-09-30/simulation/campaign.json) · [准备核验](libo_spawn_repeats/extension_spawn30_2026-09-30/simulation/PREPARATION_CHECK.json) · [完整运行记录](libo_spawn_repeats/extension_spawn30_2026-09-30/simulation/supervisor.json) · [出生位置与朝向](libo_spawn_repeats/extension_spawn30_2026-09-30/spawn_preview.png)
- [视频来源、完整解码和目视核验](libo_spawn_repeats/extension_spawn30_2026-09-30/demos/VIDEO_REPORT.json) · [外部中断前记录](libo_spawn_repeats/extension_spawn30_2026-09-30/simulation/dispatch_resume_history/supervisor_before_01.json)

每组30指计划尝试，完整但卡住或反向的观察保留。数值/边界失败没有六秒分数，不赋零。每个合并后的rough高度×wave组仅剔除一个距0–6 s位移中位数最远的完整观察，距离并列取低spawn编号；旧轮的剔除项不沿用。0–6/2–6/4–6窗口和A/B/C共用保留集合，STD为样本STD(ddof=1)。flat每wave单一对照，不估计STD，图仍用统一圆点/连线及淡色误差条。原十spawn报告和图全部保留。

前进位移沿每个试次初始朝向投影整机质量加权COM，四元数旋转和方向投影独立核对；不是统一取−x。每个原生启动重新核对完整PD、幅度、步长、接触和线程；完整轨迹通过有限状态和原生日志审计，失败不冒充完成核验。新spawn最小预测边界余量34.15 mm，每物理步仍检查15 mm保护；两次正常速度下的边界停止分别是×1.0/k1.5/spawn25的4.904 s、×0.5/k1.5/spawn28的5.9745 s，独立保留，不当卡住。

A为每个有效10 ms时刻的missing/expected-flat脚数比例等权平均；B在每个名义1 s CPG周期内先汇总missing/expected再平均周期；C为整个窗口的总missing/总expected。逐trial计算后再求组mean/STD，足编号和指令CPG时间与相应flat参考一致，阈值1e−8 N；expected=0时A跳过。图用0–6 s，其他窗口数值也已保留。它们是phase-matched接触差异描述，phase shift也会计入loss；不当成已校准的随机擦除概率，loss轴交叉不等同于共同R上的同地形交叉。

**0–6 s mean ±样本STD，mm；括号为剔除后n。**

| 高度 | R (°) | k1.5 | k2 | k2.5 | k3 | k3.5 |
|---|---:|---:|---:|---:|---:|---:|
| 0 | 0 | 224.15 (1) | 212.54 (1) | 180.17 (1) | 153.37 (1) | 131.69 (1) |
| 0.25 | 2.256 | 213.05 ± 8.37 (28) | 204.18 ± 3.69 (29) | 175.25 ± 2.68 (29) | 147.85 ± 2.86 (29) | 127.87 ± 2.68 (29) |
| 0.5 | 4.474 | 177.76 ± 20.71 (28) | 186.35 ± 8.11 (29) | 162.84 ± 5.49 (29) | 141.46 ± 4.46 (29) | 123.49 ± 4.14 (29) |
| 0.75 | 6.625 | 144.15 ± 29.19 (29) | 160.75 ± 12.41 (29) | 151.16 ± 9.86 (28) | 131.21 ± 11.12 (29) | 120.04 ± 6.39 (29) |
| 1 | 8.692 | 89.21 ± 38.52 (27) | 123.61 ± 22.82 (29) | 128.96 ± 16.62 (29) | 116.09 ± 9.77 (29) | 110.84 ± 7.33 (29) |
| 1.25 | 10.666 | 44.12 ± 32.26 (25) | 97.27 ± 30.43 (29) | 114.20 ± 20.21 (26) | 99.79 ± 17.16 (29) | 101.62 ± 14.90 (29) |
| 1.5 | 12.546 | 30.98 ± 34.31 (24) | 75.24 ± 26.53 (27) | 99.08 ± 18.34 (23) | 81.46 ± 17.10 (27) | 89.84 ± 19.83 (28) |

**30-spawn的wave2.5/3复核：**当前数据仍不支持在这些高度下k3总体超过k2.5，单spawn优势确实存在。下面只比较双方共同完成/保留的spawn，配对差为k3−k2.5；独立组均值不能替代此配对。

| 高度 | 原始配对差 mean±STD (mm), n | k3胜出 | 剔除后配对差 mean±STD (mm), n |
|---|---:|---:|---:|
| ×1 | -14.78 ± 15.65, 30 | 4/30 | -13.45 ± 15.03, 28 |
| ×1.25 | -13.47 ± 21.24, 27 | 6/27 | -12.55 ± 15.25, 25 |
| ×1.5 | -12.76 ± 20.53, 22 | 7/22 | -15.42 ± 19.59, 20 |

这里只是同一张Libo表面的30个位置/朝向观察，不是30个独立地形。完成组均值对数值/边界缺失是条件统计；每组一次剔除也可能移除真实卡住或高表现的观察，完整原始值和失败均保留。机制与更高height下的趋势仍未确认。

调度运行约131.96 min，另有约157.32 min的中断前后无进度记录间隔；起止总墙钟289.28 min，不把间隔算作物理积分耗时。停止瞬间未知。共262个worker尝试、642次条件执行，其中36次原条件同伴补跑、6次外部中断后未计分条件续跑；不是642个独立样本。中断的原目录、日志、输入和两份未计分receipt保留，没有重跑数值触发条件。

六段demo均用运行前预选spawn12/yaw90.784°，各含五个wave；真实保存Isaac轨迹与原USD几何OpenGL回放，不是native RTX截屏，不增加物理步。每段6 s、20 fps、1倍速，120帧全解码通过，首/中/末帧已目视检查；源姿态0–5.95 s，统计终值6.00 s。回放/解码89.1 s，最小画框余量7.43 mm。单次demo不代表组均值。

- [Libo height_x0p25 · 五wave · spawn12](libo_spawn_repeats/extension_spawn30_2026-09-30/demos/libo_height_x0p25_five_waves_spawn12.mp4)
- [Libo height_x0p50 · 五wave · spawn12](libo_spawn_repeats/extension_spawn30_2026-09-30/demos/libo_height_x0p50_five_waves_spawn12.mp4)
- [Libo height_x0p75 · 五wave · spawn12](libo_spawn_repeats/extension_spawn30_2026-09-30/demos/libo_height_x0p75_five_waves_spawn12.mp4)
- [Libo height_x1p00 · 五wave · spawn12](libo_spawn_repeats/extension_spawn30_2026-09-30/demos/libo_height_x1p00_five_waves_spawn12.mp4)
- [Libo height_x1p25 · 五wave · spawn12](libo_spawn_repeats/extension_spawn30_2026-09-30/demos/libo_height_x1p25_five_waves_spawn12.mp4)
- [Libo height_x1p50 · 五wave · spawn12](libo_spawn_repeats/extension_spawn30_2026-09-30/demos/libo_height_x1p50_five_waves_spawn12.mp4)

全部仿真、合并分析、七张图和六视频已完成，无后续物理队列或关机安排。更高height1.75+、2/8 mm cutoff和两套random-erasure队列继续暂停；后续研究与短波恢复发现仍在[统一研究记录](#side-finding-shortwave)。

<a id="observed-contact-displacement"></a>
## 历史十spawn：Displacement vs missing expected support（2026-09-30）

**历史十spawn比例图（新版见上方30-spawn入口）：**用户确认每个10 ms时刻，先算缺失足数/同一时刻flat接触足数，再对有效时刻等权平均，乘100转成百分比。flat接触足数为0的时刻跳过，不记为0；各wave在0–6 s均有589个有效时刻、11个跳过时刻。公式为 `L = 100/N_valid × sum_t (missing_feet(t)/flat_contact_feet(t))`。然后按原保留spawn计算组均值与样本STD，两轴均用0–6 s，位移数值及267个保留试次不变。

- [历史：displacement vs loss portion（逐时刻比例平均）](observed_contact_loss_preview/displacement_vs_instantaneous_loss_portion.png) · [SVG](observed_contact_loss_preview/displacement_vs_instantaneous_loss_portion.svg) · [逐trial/组数值与定义核验](observed_contact_loss_preview/displacement_vs_instantaneous_loss_portion.json)

**用户追加要求：三种比例口径都画图比较。**

- [三指标并列对照](observed_contact_loss_preview/loss_portion_metrics_comparison.png) · [SVG](observed_contact_loss_preview/loss_portion_metrics_comparison.svg)
- 单图：[A 逐10 ms时刻比例平均](observed_contact_loss_preview/displacement_vs_instantaneous_loss_portion.png) · [B 逐1 s CPG周期比例平均](observed_contact_loss_preview/displacement_vs_cycle_loss_portion.png) · [C 整个0–6 s总缺失/总预期接触](observed_contact_loss_preview/displacement_vs_pooled_loss_portion.png)
- [三指标逐trial/组数值、完整定义、源哈希及检查](observed_contact_loss_preview/loss_portion_metrics_comparison.json)

B先分别在六个名义1 s周期内汇总missing/expected，再对六个比例等权平均；边界按(j,j+1]的100个10 ms记录划分。C在全0–6 s内汇总missing/expected，等价于按预期接触样本数量加权。三图使用同一267个保留试次、同一位移mean/STD、同一坐标范围、配色及淡色误差条；flat每wave单一参考不估计STD。A/B曲线较接近，C向更高loss横坐标移动，特别是高波数。该比较不自动替换先前选定的A定义；不按交叉形状挑选指标，不恢复暂停仿真。

每个有效时刻权重相同；先前PREVIEW.json里的总缺失样本/总预期样本是按预期接触数量加权，口径不同，保留作对照。仍为已有数据后处理，没有新增仿真或恢复暂停任务。下方为前一版样本数量图的定义和入口。


按用户要求，横轴使用 missing expected support samples 的**数量**，纵轴为 forward COM displacement（mm），五条曲线分别为 wave 1.5/2/2.5/3/3.5；图上不使用 terrain roughness 轴或高度标签。

- [图：displacement vs missing expected support](observed_contact_loss_preview/displacement_vs_missing_expected_support.png) · [SVG 矢量版](observed_contact_loss_preview/displacement_vs_missing_expected_support.svg) · [逐组数值、统计口径与源核验](observed_contact_loss_preview/displacement_vs_missing_expected_support.json)
- [逐脚 contact pattern 对照](observed_contact_loss_preview/contact_pattern_examples.png) · [原位移曲线与 contact loss 并列预览](observed_contact_loss_preview/original_displacement_and_observed_loss.png)

一个样本是一只脚在一次10 ms记录中的状态；同 wave、同足编号、同指令CPG时刻，flat接触且rough不接触则计一次缺失。两轴均使用0–6 s窗口、原图相同267个保留试次，组均值与样本STD（ddof=1）；flat每wave单个参考、无STD，统一圆点与实线连接。仅用已完成轨迹重绘，幅度/PD/物理未变，新增物理步为0，暂停队列未启动。

每wave的flat预期支撑样本总数依次为8360/7728/7366/7708/7421，因此当前横轴数量不等于缺失百分比。曲线依原地形条件顺序连接组均值，不拟合曲线；这里的交叉是在同一缺失数量上的曲线比较，不能直接视为旧共同roughness横轴上的同地形交叉。新增接触、相位变化、载荷和滑移仍需分别解释；接触差异本身不构成随机擦除或因果验证。

<a id="libo-six-heights"></a>
## 历史十spawn：Libo 六档高度、淡色STD与每组一次剔除

**六档 Libo 的300个条件及五个平地对照已尝试，297个完成6秒、8个提前停止；按用户规则另剔除30个完整观察用于派生统计，保留267个。**

- [0–6 s displacement vs roughness](libo_spawn_repeats/trimmed_results/mean_std_vs_roughness.png)
- [4–6 s displacement vs roughness](libo_spawn_repeats/trimmed_results/mean_std_4_6_vs_roughness.png)
- [displacement vs wave number](libo_spawn_repeats/trimmed_results/mean_std_vs_wave_number.png)
- [剔除清单、剔除前后数值、有效数与失败数](libo_spawn_repeats/trimmed_results/SUMMARY.json) · [新增100条件锁定输入与源哈希](libo_spawn_repeats/extension_1p25_1p5/simulation/campaign.json) · [新增运行记录](libo_spawn_repeats/extension_1p25_1p5/simulation/supervisor.json)

按用户确认的规则，每个“粗糙地形高度×波数”组，从完成六秒的0–6 s前向位移中剔除距该组中位数最远的一次，恰好一个；若距离并列取spawn编号较小者。0–6 s、4–6 s及派生配对差共用这套保留条件，再计算mean和样本STD(ddof=1)。这是一项明确的描述统计选择，不证明被剔除试次物理错误；真实卡住也可能被规则选中。数值/边界失败本来没有六秒值，单独记录，不算这一个统计剔除项，不赋零。原始未剔除结果和失败证据全部保留。

平地每个波数只有一个对照，保留该点，不剔除或估计STD。绘图全部用相同圆点、相同实线；R=0的平地点与各地形均值连成同一条曲线，误差条只用同色淡色。没有新增平地重复。

新增地形直接引用已生成的固定4 mm母表面高度×1.25和×1.5文件，固定38足/703对坡度角RMS分别10.66625°、12.54634°；不改变最短波长、0.5 mm网格、XY相位。十套出生位置/朝向与原四档逐项相同，CPGphase0、初始化最高点间隙规则及固定平地cluster A/B、完整173PD均不变。CPU TGS64/16、0.5 ms物理/front5反馈、1 ms CPG、contact20 µm/rest0、四worker×两线程及3+2真实环境分组均沿用。新执行的adapted_functions.py与前一轮逐字相同；无幅度优化、收敛或资源搜索。

**0–6 s mean ± STD，mm；n为统计保留的完整观察数。下表为已完成六档高度结果。**

| 高度 | R (°) | k=1.5 | k=2 | k=2.5 | k=3 | k=3.5 |
|---|---:|---:|---:|---:|---:|---:|
| 0 | 0 | 224.15 (n=1) | 212.54 (n=1) | 180.17 (n=1) | 153.37 (n=1) | 131.69 (n=1) |
| 0.25 | 2.256 | 212.26 ± 7.34 (n=8) | 204.44 ± 4.76 (n=9) | 175.15 ± 1.85 (n=9) | 149.91 ± 2.43 (n=9) | 128.13 ± 1.63 (n=9) |
| 0.5 | 4.474 | 183.91 ± 11.84 (n=9) | 192.25 ± 6.44 (n=9) | 162.85 ± 5.65 (n=9) | 141.96 ± 2.62 (n=9) | 123.35 ± 2.51 (n=9) |
| 0.75 | 6.625 | 163.58 ± 12.71 (n=9) | 158.30 ± 13.62 (n=9) | 147.79 ± 6.28 (n=8) | 136.35 ± 9.00 (n=9) | 117.57 ± 4.36 (n=9) |
| 1 | 8.692 | 96.98 ± 32.90 (n=8) | 127.44 ± 14.87 (n=9) | 133.27 ± 14.87 (n=9) | 114.64 ± 10.60 (n=9) | 111.79 ± 3.61 (n=9) |
| 1.25 | 10.666 | 25.48 ± 28.55 (n=8) | 109.92 ± 23.90 (n=9) | 130.67 ± 16.60 (n=8) | 102.89 ± 11.92 (n=9) | 103.49 ± 9.97 (n=9) |
| 1.5 | 12.546 | 21.57 ± 23.32 (n=9) | 64.49 ± 27.09 (n=9) | 106.54 ± 15.14 (n=8) | 78.75 ± 10.90 (n=7) | 90.32 ± 19.13 (n=9) |

每组剔除项、距中位数差、原始mean/STD与保留mean/STD均在SUMMARY.json；原始四档结果见下方首次试验，新增原始记录另存extension_1p25_1p5/results。不同高度各自剔除一个可能造成保留的spawn集合不同，因此配对差只使用双方均保留的交集，并报告交集数。位移仍按各试次初始头向投影整机质量加权质心；位置/朝向联合变化、同一张地形及短时窗口的证据限制仍适用。

**新增两档原始运行：**95/100条件完成6秒，失败分类{'state_divergence': 5}；108次条件执行中含8次受牵连同伴补跑，未算新增统计样本。调度21.51分钟，回放/解码30.8秒。所有启动的完整PD、contact、求解器、步长、线程及源哈希重新核对；完成组全步有限状态和最终原生日志通过；失败组不冒充通过最终审计。

- [新增两档全部原始数值与失败记录](libo_spawn_repeats/extension_1p25_1p5/results/RESULTS.json)
- [新增视频的来源与解码/目视核验](libo_spawn_repeats/extension_1p25_1p5/demos/VIDEO_REPORT.json)

- [Libo height_x1p25 · 完成波数1.5,2,2.5,3,3.5 · spawn2](libo_spawn_repeats/extension_1p25_1p5/demos/libo_height_x1p25_five_waves_spawn02.mp4)
- [Libo height_x1p50 · 完成波数1.5,2,2.5,3,3.5 · spawn2](libo_spawn_repeats/extension_1p25_1p5/demos/libo_height_x1p50_five_waves_spawn02.mp4)

两段demo预先固定spawn2/yaw90°，按实际已保存的Isaac轨迹、原USD几何OpenGL回放，未新增物理；如果预选条件没有完整6秒记录，会注明缺失波数。视频不随统计剔除选择最佳试次，显示的单次轨迹不等于组均值。三张派生图及两视频首/中/末帧已检查。全部物理和后处理结束，无后续队列或关机安排。

以下为首次四档的原始未剔除结果、前一轮高度扫描和短波旁支。

<a id="libo-spawn-repeats"></a>
## Libo 配对 spawn 重复试验（2026-09-29）

**200 次 Libo 条件与五次平地对照均已尝试；197/200 次粗糙地形、5/5 次平地完成 6 秒，3 个条件未完成（2 个 非物理状态发散、1 个 边界保护截断）。三张统计图和五段配套 demo 已生成。**

- [0–6 s mean ± std vs roughness](libo_spawn_repeats/results/mean_std_vs_roughness.png)
- [4–6 s mean ± std vs roughness](libo_spawn_repeats/results/mean_std_4_6_vs_roughness.png)
- [mean ± std vs wave number](libo_spawn_repeats/results/mean_std_vs_wave_number.png)
- [完整数值、配对高度差、有效数、失败数和来源核验](libo_spawn_repeats/results/RESULTS.json) · [逐时刻前向位移](libo_spawn_repeats/results/forward_timeseries.npz)
- [出生位置与朝向预览](libo_spawn_repeats/spawn_preview.png) · [锁定输入与源哈希](libo_spawn_repeats/simulation/campaign.json) · [运行及补跑记录](libo_spawn_repeats/simulation/supervisor.json) · [视频来源、解码与目视检查](libo_spawn_repeats/demos/VIDEO_REPORT.json)

仅 Libo，固定名义最短波长 4 mm、0.5 mm 空间网格和 XY 相位，地形高度 ×0.25/0.5/0.75/1，wave 1.5/2/2.5/3/3.5。每个组合十套共享的初始位置/朝向，CPG 初相位保持原 sample0；初始 Z 按原局部最高点间隙规则变化。幅度用既有平地 cluster A/B，完整173关节 PD用9/17 wave-4表；CPU TGS64/16、0.5 ms物理/front5反馈、1 ms CPG、contact20 µm/rest0和摩擦0.5均固定。没有新幅度优化或收敛/容量搜索。

主批次为四worker×两Kit/PhysX线程；每套spawn按三个低波数及两个高波数分组，实际每四worker批次十条件，不补造样本。数值库线程1；PXR请求2、原生读回28。五次新flat各波数一次，统一用预选spawn2、yaw90°，是两worker部分批次。受牵连同伴用原spawn、命令和参数在部分批次补跑，环境在该剩余条件批次中重新编号，未重抽样、未重跑触发异常的条件。

位移为含局部 COM 偏移的整机质量加权质心，在 XY 平面的净平移投影到各试次初始尾→头方向；同时保存横向位移。姿态四元数旋转与投影均用独立实现核对。roughness 使用共同区域位置和全朝向平均的固定中立38足/703对坡度角RMS，所有wave共用，未使用per-gait指标。

**0–6 s 前向位移，单位 mm。粗糙地形为 mean ± 样本 std（ddof=1）；括号仅标出不足十次的有效数。平地每个波数单次，不估计std。**

| Wave | 平地 | Libo ×0.25 | ×0.5 | ×0.75 | ×1 |
|---|---:|---:|---:|---:|---:|
| 1.5 | 224.15 | 210.07 ± 9.51 (n=9) | 176.81 ± 25.08 | 158.60 ± 19.77 | 89.27 ± 38.49 (n=9) |
| 2 | 212.54 | 203.13 ± 6.10 | 190.00 ± 9.35 | 160.89 ± 15.22 | 131.71 ± 19.48 |
| 2.5 | 180.17 | 174.79 ± 2.09 | 161.72 ± 6.42 | 150.00 ± 8.88 (n=9) | 131.01 ± 15.73 |
| 3 | 153.37 | 149.20 ± 3.20 | 142.70 ± 3.40 | 134.11 ± 11.06 | 116.57 ± 11.70 |
| 3.5 | 131.69 | 127.73 ± 1.99 | 124.07 ± 3.29 | 118.78 ± 5.62 | 110.69 ± 4.86 |

**本轮观察：**这套固定A/B下，五个波数的完成条件均值都随四档地形高度增加而下降。wave1.5从×0.25的 210.07±9.51 mm降到×1的 89.27±38.49 mm；×1原始值范围 27.65–143.01 mm，位置/朝向依赖明显。wave3.5在×1为 110.69±4.86 mm，观察到的离散程度更小。旧固定spawn的wave1.5、×1单次2.43 mm不代表这组新位置/朝向的平均表现；也不能据此单独归因到位置或朝向，更不能验证锚点机制。

五个新平地对照与原yaw0基准的最大差 0.124 mm；位置与朝向同时改变，不能归因于单独的yaw效应。平地是参考点，不能据此估计十次配对rough/flat差的方差。

**失败与边界：**正常卡住、零位移、低位移和后退全部保留，未按性能剔除。异常条件没有六秒数值，不赋零；均值只描述完成条件，各组失败比例在完整数值中另列。实际碰撞包络从Mesh点、Sphere半径和Capsule半径/高度读取，未使用失真的Capsule extent元数据；不修改碰撞形状，不增加墙、夹紧或重置。每0.5 ms检查15 mm保护，全部完成试次的最小外形边距 34.63 mm。

- 高度 ×0.75、wave 2.5、spawn 1，t=2.6285 s：state_divergence。[原始异常与已采样轨迹](libo_spawn_repeats/simulation/runs/v1/height_x0p75_s01_low/BOUNDARY_FAILURE.json)。
- 高度 ×0.25、wave 1.5、spawn 5，t=5.9845 s：boundary_guard。[原始异常与已采样轨迹](libo_spawn_repeats/simulation/runs/v1/height_x0p25_s05_low/BOUNDARY_FAILURE.json)。
- 高度 ×1、wave 1.5、spawn 9，t=4.3 s：state_divergence。[原始异常与已采样轨迹](libo_spawn_repeats/simulation/runs/v1/height_x1p00_s09_low/BOUNDARY_FAILURE.json)。

所有实际尝试均重新核对完整PD、原生contact offset/rest、求解器迭代、步长、线程、源哈希和幅度输入；失败组只有启动核验，不冒充通过最终原生错误审计。完成组通过全步有限状态和最终原生日志检查。AST比较确认场景创建函数、命令装载函数及物理/控制/反馈循环前缀与上一轮实际代码一致；其余声明变化见RESULTS.json。

这是同一个固定地形上的十组位置/朝向联合条件，并非十个独立地形实例。位置受每个朝向的安全走廊约束，不是全场均匀抽样；CPG相位固定，不分离位置与朝向各自的方差。0–6 s含落地，4–6 s为补充窗口而非已验证的稳态。STD是空间/朝向变化的描述量，不等于均值置信区间；数值失败会使均值有条件选择。

**配套demo：**每段并列五个波数，均选预先固定的spawn2/yaw90°。使用实际Isaac轨迹和原USD几何OpenGL回放，无新增物理步；不是原生RTX截屏。统一相机比例和真实Z比例，每段6秒、20fps、1倍速，姿态取t=0–5.95 s；图表终值取t=6.00 s。视频全120帧解码及首/中/末帧、三张图已检查。

- [平地 · 五波数](libo_spawn_repeats/demos/libo_flat_five_waves_spawn02.mp4)
- [Libo 高度 ×0.25 · 五波数](libo_spawn_repeats/demos/libo_height_x0p25_five_waves_spawn02.mp4)
- [Libo 高度 ×0.50 · 五波数](libo_spawn_repeats/demos/libo_height_x0p50_five_waves_spawn02.mp4)
- [Libo 高度 ×0.75 · 五波数](libo_spawn_repeats/demos/libo_height_x0p75_five_waves_spawn02.mp4)
- [Libo 高度 ×1.00 · 五波数](libo_spawn_repeats/demos/libo_height_x1p00_five_waves_spawn02.mp4)

205 个预定条件，共 211 次条件执行、85 次worker尝试，其中 6 次为受牵连条件补跑，不计作新增统计样本。实际仿真调度用时 40.68 分钟（含启动、记录、组间收尾和同伴补跑）；视频生成/完整解码 73.6 秒。源码快照在simulation/executed_source、results/analysis_source.py和demos/rendering_source.py。全部运行与后处理结束，无后续队列或关机安排。

以下为已完成的前一轮高度扫描及短波旁支。
2026-09-29：**主线是固定最短波长、调整高度构造 roughness 序列。本轮固定 4 mm 截断的七档高度试跑已结束：七档高度 × 两站点 × 六波数的 84 个条件均已尝试；78 个完成 6 秒，2 个状态发散，4 个被同组保护提前中断。三张位移图与 14 段 demo 已完成，其中 12 段含六波数，Libo 1.5/2 倍的两段只含完成的三个高波数。 per-gait sweep 尚未开始；短波频带试验作为旁支发现保留。**

<a id="fixed4mm-height-preview"></a>
## 4 mm 固定频带：七档地形高度与试跑

**运行更新：**七档高度 × 两站点 × 六波数的 84 个条件均已尝试；78 个完成 6 秒，2 个状态发散，4 个被同组保护提前中断。三张位移图与 14 段 demo 已完成，其中 12 段含六波数，Libo 1.5/2 倍的两段只含完成的三个高波数。 详情及全部视频见[实际仿真结果](#fixed4mm-height-results)。

2026-09-29：按用户“多做几组”的要求，将两组提议范围合并为 **0.25、0.5、0.75、1、1.25、1.5、2 倍**，Libo、Narvik 各七档，共 14 个地形条件。地形生成阶段只改变地形高度 amplitude，保持机器人平地 cluster A/B、完整 PD；随后用户授权的实际试跑结果列于下方。

- [Libo 七档三维表面与截面](height_amplitude_4mm/libo_height_amplitude_preview.png)
- [Narvik 七档三维表面与截面](height_amplitude_4mm/narvik_height_amplitude_preview.png)
- [固定足点 roughness / 高度 RMS 随倍率变化](height_amplitude_4mm/roughness_vs_height_amplitude.png)
- [全部地形路径、哈希、参数及检查](height_amplitude_4mm/REPORT.json)

母表面分别为已保存的 `shortwave/libo_restore4.npz`、`shortwave/narvik_restore4.npz`。采用 `h_s = mean(h_0) + s * (h_0 - mean(h_0))`，保持平均高度、名义 4 mm 截断、0.5 mm 网格、1.022 m 顶点跨度、XY 布局、相位及归一化谱形；非 DC 谱功率随 s² 变化。沿用母表面既有的三角网格表示尾部，不另做滤波。1 倍直接引用原文件，其他倍率保存为 12 张新的完整 2045×2045 NPZ。

R 仍使用同一中立 38 足、703 对；共同仿真 ROI 内的相同位置与全部朝向平均，复用 65,536 个 Sobol 样本，未做收敛搜索。两个 1 倍 R 均与先前 4 mm 结果一致；没有计算 per-gait roughness。

| 地形高度倍率 | Libo 固定足点 R | Narvik 固定足点 R | Libo 高度 Sq (mm) | Narvik 高度 Sq (mm) |
|---|---:|---:|---:|---:|
| 0.25 | 2.2564° | 0.5417° | 0.7966 | 0.1955 |
| 0.5 | 4.4744° | 1.0828° | 1.5932 | 0.3910 |
| 0.75 | 6.6253° | 1.6228° | 2.3897 | 0.5865 |
| 1 | 8.6919° | 2.1612° | 3.1863 | 0.7821 |
| 1.25 | 10.6662° | 2.6975° | 3.9829 | 0.9776 |
| 1.5 | 12.5463° | 3.2313° | 4.7795 | 1.1731 |
| 2 | 16.0315° | 4.2896° | 6.3726 | 1.5641 |

预览显示同一中心 (170, −120) mm 的 100×100 mm 局部窗口，各站点七档共享相机、Z 范围和色标，并附同一中心截面。为看清形状，Z 显示统一夸张 3 倍，坐标刻度仍为真实毫米；Libo 与 Narvik 的色标范围分别注明。图中 R 是共同 ROI 的描述量，并非只取这个显示窗口。保存数据未经视觉夸张。

地形读回、平均高度、Sq 倍率、基准 R、源哈希和图片解码均通过；三张图已目视检查。入口为 `height_amplitude_preview.py`，实际生成源码快照在 `height_amplitude_4mm/generation_source.py`。所有预览已完成，无后台仿真队列。

<a id="fixed4mm-height-results"></a>
### 七档高度实际仿真结果（2026-09-29）

**七档高度 × 两站点 × 六波数的 84 个条件均已尝试；78 个完成 6 秒，2 个状态发散，4 个被同组保护提前中断。三张位移图与 14 段 demo 已完成，其中 12 段含六波数，Libo 1.5/2 倍的两段只含完成的三个高波数。**

- [0–6 s displacement vs roughness](height_amplitude_4mm/results/displacement_vs_roughness.png)
- [4–6 s displacement vs roughness](height_amplitude_4mm/results/displacement_4_6_vs_roughness.png)
- [displacement vs wave number](height_amplitude_4mm/results/displacement_vs_wave_number.png)
- [全部数值、运行选择、失败分类和核验](height_amplitude_4mm/results/RESULTS.json) · [逐时刻前向位移](height_amplitude_4mm/results/forward_timeseries.npz) · [视频来源及完整解码](height_amplitude_4mm/demos/VIDEO_REPORT.json)

唯一地形干预是相对均值的高度倍率，名义最短波长 4 mm、0.5 mm 空间网格和相位固定。平地 cluster A/B、9/17 wave-4 全173关节 PD、CPU TGS64/16、0.5 ms 物理及反馈、1 ms CPG 指令保持均未改变。主批次固定四进程 × 两线程 × 三个实际条件，低波数/高波数组槽位及站点顺序不变；同一初始 XY、朝向和落地间隙规则，初始 Z 随表面高度按原规则变化。不是幅度优化、收敛测试或容量搜索。

前向位移仍为整机质心沿初始尾到头 XY 方向（约 −X）的净平移；包含各刚体质量和局部 COM 偏移。R 是同一中立 38 足／703 对、共同区域位置及全部朝向的平均，各 wave 共用，未使用 per-gait 指标。0–6 s 包含落地启动；4–6 s 是补充窗口，尚未证明稳态。每个条件只有一个初始位置/朝向/地形相位，不能由连线推断统计显著性或单一机制。

**主要观察：**Libo wave 1.5 在高度 0.75 → 1 → 1.25 倍时，前向位移为 143.04 → 2.43 → 114.13 mm；对应 4–6 s 为 62.40 → 0.59 → 41.86 mm，非单调表现也出现在后两秒。wave 2 在 1.25 倍降到 1.69 mm（4–6 s 为 −0.86 mm）。Narvik 本轮全部完成，wave 1.5 各高度均保持约 199–227 mm；Libo 的现象不能直接推广到另一地形。固定截止波长下改变高度也出现低波数性能谷值，不能把此现象直接归为之前旁支中的“恢复短波使锚点增多”。

**0–6 s 前向净位移，单位 mm；× 为本环境状态发散，— 为同组保护导致未完成，均不是零位移。**

| 站点 | 高度倍率 | R (°) | k=1.5 | k=2 | k=2.5 | k=3 | k=3.5 | k=4 | Demo |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| Libo | 0.25 | 2.256 | 213.55 | 207.03 | 171.94 | 145.28 | 128.69 | 112.14 | [六波数](height_amplitude_4mm/demos/libo_height_x0p25_six_waves.mp4) |
| Libo | 0.5 | 4.474 | 164.26 | 191.99 | 164.59 | 141.79 | 124.06 | 108.29 | [六波数](height_amplitude_4mm/demos/libo_height_x0p50_six_waves.mp4) |
| Libo | 0.75 | 6.625 | 143.04 | 146.80 | 143.22 | 127.67 | 116.72 | 104.12 | [六波数](height_amplitude_4mm/demos/libo_height_x0p75_six_waves.mp4) |
| Libo | 1 | 8.692 | 2.43 | 109.07 | 147.34 | 117.15 | 116.12 | 99.34 | [六波数](height_amplitude_4mm/demos/libo_height_x1p00_six_waves.mp4) |
| Libo | 1.25 | 10.666 | 114.13 | 1.69 | 124.87 | 121.86 | 111.45 | 98.61 | [六波数](height_amplitude_4mm/demos/libo_height_x1p25_six_waves.mp4) |
| Libo | 1.5 | 12.546 | — | — | × | 116.62 | 111.19 | 75.39 | [仅 k=3/3.5/4](height_amplitude_4mm/demos/libo_height_x1p50_high_waves.mp4) |
| Libo | 2 | 16.031 | × | — | — | 72.84 | 73.80 | 51.55 | [仅 k=3/3.5/4](height_amplitude_4mm/demos/libo_height_x2p00_high_waves.mp4) |
| Narvik | 0.25 | 0.542 | 226.89 | 210.82 | 180.16 | 152.70 | 131.62 | 117.54 | [六波数](height_amplitude_4mm/demos/narvik_height_x0p25_six_waves.mp4) |
| Narvik | 0.5 | 1.083 | 219.84 | 209.75 | 179.65 | 151.77 | 131.30 | 116.97 | [六波数](height_amplitude_4mm/demos/narvik_height_x0p50_six_waves.mp4) |
| Narvik | 0.75 | 1.623 | 215.94 | 206.41 | 178.70 | 150.39 | 130.06 | 115.06 | [六波数](height_amplitude_4mm/demos/narvik_height_x0p75_six_waves.mp4) |
| Narvik | 1 | 2.161 | 219.05 | 205.31 | 177.36 | 139.93 | 128.47 | 113.40 | [六波数](height_amplitude_4mm/demos/narvik_height_x1p00_six_waves.mp4) |
| Narvik | 1.25 | 2.698 | 199.48 | 189.40 | 175.97 | 139.20 | 129.59 | 112.23 | [六波数](height_amplitude_4mm/demos/narvik_height_x1p25_six_waves.mp4) |
| Narvik | 1.5 | 3.231 | 202.49 | 185.19 | 172.32 | 137.23 | 129.15 | 108.72 | [六波数](height_amplitude_4mm/demos/narvik_height_x1p50_six_waves.mp4) |
| Narvik | 2 | 4.290 | 200.46 | 179.38 | 149.16 | 135.92 | 128.61 | 107.63 | [六波数](height_amplitude_4mm/demos/narvik_height_x2p00_six_waves.mp4) |

**失败保留与边界：**Libo 1.5 倍的 wave 2.5 在 t=1.5555 s、2 倍的 wave 1.5 在 t=3.867 s 发生非物理状态发散；瞬时最大 body 线速度分别约 3.16×10⁹、2.66×10¹¹ m/s，触发原 15 mm 边界保护。这不是可信的巨大前进量，也不能按一般越界扩大地形后当作通过。各自同 worker 的另两个条件被一起停止，未给它们赋予 6 秒位移或“步态失败”结论；2 倍组的同伴也有较大瞬时速度，未获完整质量验证。

原 `runs/v1/libo_height_x1p50_low` 在异常抛出前未保存轨迹。只给原保护分支添加失败状态/已采样轨迹落盘，在 `runs/v2_capture` 完成尚未启动的 2 倍四进程批次，并对 1.5 倍低波数组做一次单进程、同三槽位、同两线程的有界复现。复现的触发时刻和越界幅度完全相同；它是诊断，不能当独立统计样本或新的并行方案。AST 核对确认执行代码仅改变原失败分支的记录，异常、步进、控制、PD、初始化、地形和保护条件均未改变。根因尚未区分，未继续扩展诊断或调参。

- [1.5 倍失败环境与精确状态摘要](height_amplitude_4mm/simulation/runs/v2_capture/libo_height_x1p50_low/BOUNDARY_FAILURE.json)
- [2 倍失败环境与精确状态摘要](height_amplitude_4mm/simulation/runs/v2_capture/libo_height_x2p00_low/BOUNDARY_FAILURE.json)
- 同目录保存 `failure_state.npz`、`trajectory_partial.npz` 和原始日志；原失败记录也保留。

全部 28 组已选择运行的启动读回均核对完整 PD、幅度、步长、迭代数和线程。26 组成功运行通过逐步有限状态、原生日志、隔离与裁剪保护检查，保存帧最小 body 中心边距为 96.49 mm；失败组无最终原生错误审计，不冒充通过。高度 1 倍的 12 条件与旧 4 mm 基准的 t/pose/q/v 数组逐位相同，位移差为 0；这证明复现一致，不证明物理真实性。

视频读取已保存的实际 Isaac 轨迹，用原 USD 几何 OpenGL 回放；没有新增物理步，不是原生 RTX 截屏。每段 6 秒、20 fps、1 倍速，保存姿态 t=0–5.95 s；图表终值取 t=6.00 s。所有视频 120 帧完整解码通过，首/中/末帧与三张图已检查。三个高波数的两段明确标记低波数未完成。源码入口：`run_height_amplitude.py`、`recover_height_amplitude.py`、`analyze_height_amplitude.py`、`render_height_amplitude.py`；物理源码及失败记录源码在 `simulation/executed_source`，后处理快照在 `results/analysis_source.py` 与 `demos/rendering_source.py`。

本轮试跑、结果整理及回放已结束。六个条件仍无完整 6 秒结果；无新物理任务或关机安排，per-gait sweep 仍未开始。

<a id="side-finding-shortwave"></a>
## 旁支发现与后续计划：地形频带、roughness 和步态表现

**2026-09-30 · 最初仅记录；后续只启动其中的现有六档30-spawn补齐，已完成，见[30-spawn结果](#libo-spawn30-extension)。其余计划仍暂停。**用户明确“先别跑，我让你记录，我们后面抽时间统一执行”。本次只核对既有轨迹并更新记录：没有启动新 simulation、没有生成待跑 campaign、没有队列、自动化或关机安排。下列计划不构成执行授权。

### 后续统一执行的研究清单

1. **核对 wave 2.5 与 3 在高度 >×1.0 的关系。**用户从 demo 观察到 wave 3 的优势，与当前组均值不一致。补测时保留各 spawn 的原始位移、同一 spawn 的波数配对差、未剔除与剔除后的 mean/STD、失败数和对应视频，区分单次表现与总体倾向；不预设哪一波数一定更好。下面保留原十spawn核对；最新30-spawn的配对统计见[完成结果](#libo-spawn30-extension)。
2. **每组总计 30 个共享 spawn（用户已选定）。**现有每组十个条件保留，再增加二十个新的位置/朝向；新的“地形高度 × 波数”组使用同一套三十个条件。波数仍为 1.5、2、2.5、3、3.5。三十是计划试次总数，完成数与数值/边界失败数分别报告；被同伴中断后补跑的同一条件不增加样本数。同一张表面的空间重复不冒充独立地形 realization。
3. **向更粗糙地形延伸。**固定最短波长后，继续高度 ×1.75、×2.0，随后按 ×0.25 递增，直到 wave 3.5 也无法持续前进。每个新高度仍比较上述五个波数与共享 spawn。**“走不动”的定量阈值、重复比例及最低有效完成数尚未确认**，须在将来统一执行前讨论；先前询问中的“末两秒 <2 mm、80%”只是提案，不能作为已采用标准。数值发散与场地边界截断不能算动力学卡住。
4. **不同截断地形做同类试验。**按讨论的名义最短波长 2/4/8 mm 分别固定频带、扫描高度 roughness、比较五个波数，使用共同 spawn 和固定足点 roughness 指标。不能把最短波长标签当作 roughness；每个表面都要计算实际 R。历史约8 mm地形的网格为2 mm，而4/2 mm档为0.5 mm；因此后续要先明确网格、母场/短波相位、幅值基准和比较口径，不能将频带、离散精度与高度变化合并后作单一因果解释。
5. **统一图和证据入口。**沿用淡色STD errorbar、全部统一圆点及连线到平地；各rough组按已确认规则剔除一个距0–6 s中位数最远的完整观察，两个时间窗口使用同一保留集合。必须同时保留未剔除统计与原始点，避免把真实成功/卡住误称为错误数据；flat维持适量既有对照，不默认补成三十次。地形预览、实际轨迹demo、剔除清单、失败记录、源代码/输入哈希和耗时与本入口的短波恢复发现放在一起。

**固定约束：**既有平地 cluster 优化 A/B、原身体模型/配置、9/17 wave-4 全173关节 PD、0.5 ms物理/front5反馈、1 ms CPG保持、CPU TGS64/16、contact20 µm/rest0、原摩擦/初始化与边界保护均保持。沿用四worker×两Kit/PhysX线程、数值库线程1和既有实际环境分组；不进行幅度优化、PD重调、收敛或容量搜索。spawn 的CPG初相位仍用原sample0。

规模参考，尚未排队：仅4 mm的六个已有高度补二十个spawn，加上×1.75/×2.0各三十个spawn，即600+300=900个新增粗糙条件；若2/8 mm也各覆盖相同八个高度，则另各1200个，合计3300个新增条件，之后每个截止档每加一高度再加150个。最终范围由“走不动”标准和未来实际结果决定，不把这张清单当作已运行或已锁定总量。

### 既有数据核对：视频里的优势为何未出现在均值里

本次从原始保存姿态重新计算251个body含局部COM偏移的质量加权质心，沿各试次初始头向作有符号投影；同时核对wave/env绑定和视频使用的位移序列。四个被核对的demo条件，重新计算结果与视频HUD源序列逐点一致。相机各自跟随脊柱中心，视频只显示预选spawn2；画面运动不等于组均值。视频末帧为5.95 s，下面数值为轨迹的精确6.00 s。[核对数值、原始轨迹和源哈希](libo_spawn_repeats/wave2p5_3_audit_2026-09-30/AUDIT.json) · [轨迹及共同spawn简图](libo_spawn_repeats/wave2p5_3_audit_2026-09-30/video_vs_shared_spawns.png)

| 高度 | demo spawn2，k=2.5 / k=3 | 共同完整spawn中k=3胜出 | 未剔除配对差 mean（k=3−k=2.5） |
|---|---:|---:|---:|
| ×1.25 | 125.29 / 115.16 mm | 0/9 | −28.01 mm |
| ×1.5 | 108.90 / 122.12 mm | 1/7 | −25.50 mm |

**×1.5的demo中k=3领先13.22 mm是真的。**这也是k=3完整组按用户约定规则被剔除的spawn2：该组原始均值84.17 mm，剔除后78.75 mm；k=2.5则为102.80→106.54 mm。剔除放大了组间差异，但未剔除、同spawn配对的比较仍没有显示k=3平均领先；末两秒配对差也为−13.67 mm。不同波数的数值失败造成完成集合不同（×1.5：k=2.5失败1个、k=3失败2个），须一并保留，不能按零位移合并。本次核对未发现投影或视频绑定不一致；这并不解释位置依赖的动力学机制，也不替代之后扩大样本的试验。

<a id="contact-erasure-100"></a>
### 当前轻量研究：38腿独立 contact erasure 与时间粒度

用户先明确允许启动固定p、两种等效率的完整研究，随后又要求“先别跑4060完整的，把这个也归档，先做一些轻量的，完整的，1h左右耗时的”。最新状态是轻量版随机队列因约一小时目标的实质超时而暂停，剩余75个原条件等待用户范围选择；两个明确批准的十环境端点对照已完成。完整4060和higher-height/cutoff rough扩展均暂停；现有六档30spawn已按后续授权完成，不能据此自动恢复其他队列或安排关机。

轻量版保留五wave 1.5/2/2.5/3/3.5、固定p=0..1每0.1一档、10/20/50ms三档擦除状态保持时间，只将随机档重复缩为2个mask seed。共280唯一条件：270新随机条件+10个已经完成、跨步长共用的p0/1端点。每次6s，0–2s不擦除、2–6s擦除和计分。38条腿左右独立，每时间片重新抽Bernoulli(p)，相邻擦除片连续保持off；每个条件p固定。跨wave/p/步长复用同一seed的时间索引uniform bank，步长不同的mask不同。每个随机worker十个真实条件、四worker×两线程；不补造样本。物理0.5ms/CPG1ms、完整173PD、既有cluster A/B及身体模型保持固定。

- [锁定输入/源码/布局/mask与范围](contact_erasure_light_2026-09-30/simulation_v2/campaign.json) · [当前暂停状态](contact_erasure_light_2026-09-30/simulation_v2/RUNTIME_SCOPE_PAUSE.json) · [已完成匹配端点](contact_erasure_light_2026-09-30/layout_controls/supervisor.json)
- [原生通道核验](contact_erasure_light_2026-09-30/simulation_v2/CHANNEL_GATE.json) · [最初随机诊断的发散状态](contact_erasure_light_2026-09-30/simulation/runs/v1/pilot_p050_bin010_w3/BOUNDARY_FAILURE.json)
- [已归档完整4060状态/位置映射](_archive/contact_erasure_full_2026-09-30/ARCHIVE_STATUS.json) · [既有297完成rough/flat输入、8失败单列](contact_erasure_light_2026-09-30/rough_inputs/ROUGH_INPUTS.json)

真实擦除关闭每条腿整个subtree的6个leg/foot colliders，共228个；21个chassis collider保留。原生零擦除五wave轨迹与原flat逐位一致；全擦除后leg原生normalforce为0。p0.5/10ms/w3/seed0随机诊断在2.9s数值发散，发散前5800completed steps擦除腿原生接触力均为0，初次擦除后mass/inertia/173PD不变。首次恢复后1ms内所有机器人接触的最大native penetration约1.161mm，峰normalforce约0.411N；不是专属恢复足的度量，也不据此证明发散原因。失败与代码/计划保留；不会当卡住、零位移或稳定性通过。最初诊断gate误要求随机改动轨迹保持6s稳定，已将native通道检查与数值稳定性结果分开，PD/物理未改。主批次增加初次恢复和结束时mass/inertia/PD核验。

输出包括每步长五wave displacement–p曲线、原始seed点、浅色样本SD及配对wave差；所有完成的静止/反向结果保留，不套用rough历史一次剔除规则。数值/边界失败没有6s值，受牵连同伴仅按原输入补跑，不增加seed数。每组2seed只能看探索性趋势和候选crossing；不把插值交点当已确认临界p，也不自动启动新seed确认。

两种rough等效率都做：接触法按同wave无擦除flat的CPG相位预期支撑脚来排除自然swing，再用flat erasure的support-missing曲线校准；性能法从同wave/步长的2–6s位移曲线反求p。均保留多解、越界和缺失档不能插值的情况。已有rough力记录只有10ms，比较采用共同400个2–6s足点采样；新flat同时记录0.5ms力。相位偏移、滑移、身体接触及碰撞恢复使它们仅是描述性等效量，不证明rough造成独立Bernoulli擦除。

旧contact-erasure参考仍保存于[Round9 IID_PROTOCOL.md](X:/Projects/_archive/Codex/2026-09-11/con/round9_wave_amplitude_2026-09-14/trend_revision_2026-09-15/IID_PROTOCOL.md)：旧MuJoCo31.25µs、19左右腿对、0.5s外部随机窗口和burst时长，并非每个physics步独立；旧wave3/4发现集交叉没有通过新seed确认，恢复penetration最大约6.37mm。本轮不采用旧PD/amplitude/solver参数。

发散只读复核：当前130个随机条件完成6s，65个原条件数值发散，75个仍未计分；未计分与数值失败分列，不置零。65次都由边界guard截获，但停止时最小body峰速度已约1.16×10⁶m/s，全部是数值爆炸而非正常走出边界，也不是先观察到NaN。41次停止前最后一帧的最快关节是tarsus。最初w3/p0.5/10ms诊断在2.86s采样时tarsus已1425rad/s，2.87–2.9s无原生地面接触力仍继续增长，2.9s爆到6.09×10¹⁴m/s；这不支持把最后一次恢复冲击直接认定为原因。当前低惯量弱阻尼跗节与关节/接触求解耦合是待验证解释，USD更新副作用、离散积分及其他实现问题尚未排除。删除力为零与参数不变仅核验通道的一部分，完成6s/有限状态亦不单独证明动力学可信。旧Round9 MuJoCo228/228完成、数值警告0，Newton/discrete、31.25µs、19双腿对长burst；该轮armature0。更早August MuJoCo用0.5ms/RK4且armature10⁻⁴kg·m²，属于另一模型/控制设置。两套旧MuJoCo都不能作为当前PhysX仅换引擎的匹配对照。没有新增诊断物理或调整PD/A/B/物理。

[发散复核数据/来源/解释边界](contact_erasure_light_2026-09-30/divergence_audit/DIVERGENCE_AUDIT.json) · [已有失败时间线](contact_erasure_light_2026-09-30/divergence_audit/pilot_divergence_timeline.png) · 三段已生成demo：[10ms](contact_erasure_light_2026-09-30/demos/flat_p010_bin010_seed00.mp4)、[20ms](contact_erasure_light_2026-09-30/demos/flat_p010_bin020_seed00.mp4)、[50ms](contact_erasure_light_2026-09-30/demos/flat_p010_bin050_seed00.mp4)。

已生成四个代表失败视频（实际保存姿态回放，无新物理）：[wave3／p0.5／10ms](contact_erasure_light_2026-09-30/failure_demos/01_wave3_p050_hold10.mp4)，2.9000s停止；[wave2／p0.6／20ms](contact_erasure_light_2026-09-30/failure_demos/02_wave2_p060_hold20.mp4)，4.4070s停止；[wave2／p0.6／50ms](contact_erasure_light_2026-09-30/failure_demos/03_wave2_p060_hold50.mp4)，2.2845s停止；[wave2.5／p0.5／20ms](contact_erasure_light_2026-09-30/failure_demos/04_wave2p5_p050_hold20.mp4)，3.0905s停止。每段斜视＋水平侧视、常速＋末尾0.1倍慢放；粉色半透明表示腿/足碰撞关闭。姿态原始10ms采样，无插值，片尾定格最后可显示帧并单列终止异常数值，不补造瞬间轨迹。wave2的20/50ms使用相同seed均匀数bank但mask不同，不能当单因素因果证明。四段均完整解码并查看片尾和慢放样本，来源和核验见[VIDEO_REPORT](contact_erasure_light_2026-09-30/failure_demos/VIDEO_REPORT.json)。渲染不改变随机队列暂停或任何实验输入。

### 先前旁支发现：恢复地形短波细节后低波数前进能力回升


2026-09-29 · 初步观察，独立于“固定最短波长、调整高度”的主线 roughness 扫描。

[Libo 与 Narvik 的 displacement vs wave number 合图](shortwave/displacement_roughness/displacement_vs_wave_number_combined.png)：两张图已按相同横纵轴并排整理，三条曲线及原始数值不变。

**观察：**Libo 地形恢复中高空间频率（短波长）成分后，低步态波数 k=1.5、2 在恢复至 2 mm 档的前向净位移明显回升。比较中，每个波数沿用其已有平地 cluster A/B，模型、全173关节 9/17 wave-4 PD 与物理／控制设置保持一致。[位移数据与定义](shortwave/displacement_roughness/RESULTS.json) · [Libo 图](shortwave/displacement_roughness/libo_displacement_vs_wave_number.png)

| Libo wave number | 原版（约 8 mm 截止） | 恢复至 4 mm | 恢复至 2 mm |
|---|---:|---:|---:|
| 1.5 | 14.54 mm | 2.43 mm | 113.79 mm |
| 2 | 102.86 mm | 109.07 mm | 141.37 mm |

数值为 0–6 s 整机质心沿初始前进方向的净位移。这里的 8/4/2 mm 是名义最短波长，网格分别为 2/0.5/0.5 mm。k=1.5 并非单调改善；Narvik 未呈现相同的持续恢复，每条件也只有一次 6 秒运行，因此记录为当前条件下的恢复迹象。[Narvik 对照图](shortwave/displacement_roughness/narvik_displacement_vs_wave_number.png) · [地形构造](shortwave/REPORT.json) · [仿真配置及核验](shortwave/SIMULATION_VERIFICATION.json)

**待验证机制（用户提出）：**短波细节可能增加足端可利用的有效锚点（anchoring points），改善支撑、抗滑和推进。这是解释假说；尚未通过有效锚点数量、接触持续时间或足端滑移分析证实，不能仅由前向位移归因。

**三组 × 六波数视频已保存到 X 盘，以下链接指向完整原文件：**每段包含 k=1.5、2、2.5、3、3.5、4，同屏 6 秒、20 fps。Libo 为本旁支的主要观察，Narvik 为保留的对照；全部六个文件已按原记录重新核对 SHA256，内容一致。[视频来源及完整解码记录](shortwave/demos/VIDEO_REPORT.json)

| 地形组 | Libo 六波数 | Narvik 六波数对照 |
|---|---|---|
| 原版，约 8 mm 截止 | [完整视频](shortwave/demos/libo_original_six_waves.mp4) | [完整视频](shortwave/demos/narvik_original_six_waves.mp4) |
| 恢复至 4 mm | [完整视频](shortwave/demos/libo_restore4_six_waves.mp4) | [完整视频](shortwave/demos/narvik_restore4_six_waves.mp4) |
| 恢复至 2 mm | [完整视频](shortwave/demos/libo_restore2_six_waves.mp4) | [完整视频](shortwave/demos/narvik_restore2_six_waves.mp4) |

先前旁支材料全部保留；2026-09-30的后续计划及既有轨迹核对见本节开头，仍未启动新物理运行。

## 当前方向纠正：固定最短波长，再调 roughness

用户在 2026-09-29 明确：所需 roughness 序列应先锚定一个最短波长，再调整高度等变量。 本轮进一步指定采用现有 **4 mm 截断样本**调整地形高度 amplitude，并先生成预览；机器人 A/B 和 PD 不变。此前原版／恢复至 4 mm／恢复至 2 mm 的系列是**短波频带变化对照**，虽然可计算各自的固定足点 R，但不作为目标 roughness 扫描设计。本轮七档高度地形已生成，随后授权的 84 条件试跑已结束；完整结果和失败边界见上方。

本轮第一组保持最短波长、网格、相位及 XY 布局相同，以同一母表面 `h_s = mean(h_0) + s * (h_0 - mean(h_0))` 调整高度。由既定 38 足全连接指标计算实际 R，所有 wave 共用；per-gait 是同一表面的另一套统计。高度缩放保持谱的频带与归一化形状，功率随 s² 变化；角度 RMS 随 s 非线性变化。本轮已按固定 4 mm 的七档高度生成预览并实际试跑；未开始 per-gait 扩展。

**命名必须区分：**旧版表面是 grid=2 mm、最短波长约 8 mm；新的恢复至 2 mm 表面是 grid=0.5 mm、最短波长约 2 mm。Libo wave 1.5 在这两者的 0–6 s 前向位移分别为 14.54 mm、113.79 mm。已核对新视频、env 0 的 wave/地形绑定、原始轨迹哈希与位移结果一致；4–6 s 仍前进 37.96 mm。差异的动力学原因尚未解释，净位移也不等同于整体运动质量。

[Libo / wave 1.5 / 最短波长 2 mm 单独视频](shortwave/demos/libo_lambda2mm_wave1p5.mp4)：从已完成六波数视频准确提取第一格，显示对应保存轨迹的实时前向位移，6 秒、20 fps、相机随身体移动。末帧对应 5.95 s（111.96 mm），轨迹的精确 6.00 s 值为 113.79 mm；未增加物理步。[提取与绑定核验](shortwave/demos/libo_lambda2mm_wave1p5.json)。

补充核对（2026-09-29）：Libo wave 1.5 的三段单独视频已齐全，显示同一口径的实时前向位移：

| 表面 | 单独视频 | 0–6 s 前向净位移 |
|---|---|---:|
| 原版：最短波长约 8 mm，grid 2 mm | [播放](shortwave/demos/libo_original_wave1p5.mp4) | 14.54 mm |
| 恢复至最短波长 4 mm，grid 0.5 mm | [播放](shortwave/demos/libo_lambda4mm_wave1p5.mp4) | 2.43 mm |
| 恢复至最短波长 2 mm，grid 0.5 mm | [播放](shortwave/demos/libo_lambda2mm_wave1p5.mp4) | 113.79 mm |

三段均为 6 秒、20 fps、720×720，从原六波数视频的正确画格提取，没有新物理步。视频末帧是 5.95 s，原版／4 mm 档当时分别为 15.63／6.08 mm；上表和所有位移图使用保存轨迹的精确 6.00 s 终值，摆动造成瞬时值不同。

另已按用户要求单独生成 [Libo displacement vs wave number](shortwave/displacement_roughness/libo_displacement_vs_wave_number.png) 和 [Narvik displacement vs wave number](shortwave/displacement_roughness/narvik_displacement_vs_wave_number.png)。横轴为六个 wave number，三条曲线对应原版／4 mm／2 mm 表面；均采用 0–6 s 质心沿初始前进方向的净位移，两图共用 0–240 mm 纵轴。图例分别标记最短波长与网格间距，仍属于已完成的短波频带对照。两张图和两段新增视频均已完整解码并目视检查，来源及全部作图数值见 [WAVE_NUMBER_PLOTS.json](shortwave/displacement_roughness/WAVE_NUMBER_PLOTS.json)，入口为 `compare_wave_number.py`。

## 前向位移 vs 固定足点 roughness（2026-09-29）

已从最终成功的 `simulation_v2/runs/v2` 提取 36 条件的保存轨迹，无新增物理运行。三张简图的颜色均代表 wave number，每一来源的六波数共用横轴数值：

- [0–6 s 前向净位移](shortwave/displacement_roughness/forward_0_6.png)
- [相对原版地形的位移变化](shortwave/displacement_roughness/change_from_original.png)
- [4–6 s 前向净位移](shortwave/displacement_roughness/forward_4_6.png)
- [完整数值、定义、输入哈希和检查](shortwave/displacement_roughness/RESULTS.json) · [每 10 ms 的前向位移](shortwave/displacement_roughness/forward_timeseries.npz)

前向位移定义为整机质心平移在**初始尾到头方向（约 −X）**上的投影，不是 XY 路径长度，也不随瞬时朝向改变投影轴。使用原生读回的 251 个 body 质量，并将实际执行 USD 的局部 `physics:centerOfMass` 用保存的 link pose 转到世界系后加权；局部 COM 来自已保存的 authored 参数，未另存原生 COM tensor。所有条件的质量、局部质心和 body 映射相同。过去 mass-weighted link origins 近似的前向轨迹与这里最多相差约 0.410 mm。

用户本轮选择：在共同仿真区域内均匀采样位置，并平均全部朝向。固定中立 38 足 XY、703 个不重复足点对；先对配对角度平方和位置／朝向求平均，再开根号。采用同一组 65,536 个三维 scrambled Sobol 样本（x、y、heading；seed 918）；没有收敛搜索。中心采样域由共同物理区域按模板最大半径向内缩，使所有方向和全部足点均在地形内。原三角形插值及 float32 碰撞顶点，无额外平滑、去平面或高度归一；足端自身 Z、gait 相位和实际触地状态不进入这个横轴。

| 来源 | 原版 R (°) | 4 mm 档 R (°) | 2 mm 档 R (°) |
|---|---:|---:|---:|
| Libo | 8.571256 | 8.691882 | 8.705109 |
| Narvik | 2.127328 | 2.161210 | 2.169650 |

4 mm / 2 mm 是恢复的短波频带标签，不是 roughness 数值。以下是 **0–6 s 前向净位移，单位 mm**：

| Wave | Libo 原版 | Libo 4 mm | Libo 2 mm | Narvik 原版 | Narvik 4 mm | Narvik 2 mm |
|---|---:|---:|---:|---:|---:|---:|
| 1.5 | 14.54 | 2.43 | 113.79 | 198.03 | 219.05 | 200.64 |
| 2 | 102.86 | 109.07 | 141.37 | 205.67 | 205.31 | 205.22 |
| 2.5 | 141.96 | 147.34 | 131.45 | 176.79 | 177.36 | 176.29 |
| 3 | 122.96 | 117.15 | 121.38 | 149.19 | 139.93 | 139.81 |
| 3.5 | 113.97 | 116.12 | 112.14 | 128.79 | 128.47 | 129.37 |
| 4 | 94.82 | 99.34 | 102.60 | 114.92 | 113.40 | 112.40 |

2 mm 档相对原版的差值（mm，正值表示走得更远）：

| Wave | Libo Δ | Narvik Δ |
|---|---:|---:|
| 1.5 | +99.25 | +2.61 |
| 2 | +38.51 | -0.46 |
| 2.5 | -10.51 | -0.49 |
| 3 | -1.59 | -9.38 |
| 3.5 | -1.83 | +0.58 |
| 4 | +7.78 | -2.52 |

Libo wave 1.5 的变化最大，14.54 → 2.43 → 113.79 mm，不单调；最后两秒位移仍为 0.35 → 0.59 → 37.96 mm。Narvik wave 1.5 在 4 mm 档比原版增加 21.02 mm，到 2 mm 档只增加 2.61 mm。每条件仅一个 seed/start 的 6 秒 demo，空间采样不是独立动力学重复；连线仅串联观察点，不能把变化完全归因为单个 roughness 标量。0–6 s 包含落地启动，且初始化按同一规则为不同地形选初始高度；4–6 s 为补充窗口，不表示已验证稳态。

计算入口：`displacement_roughness_analysis.py`；`--render` 只读取本次结果重画图。原计算源码保存在 `analysis_source.py`，最终图形源码在 `render_source.py`；首版图形的共享纵轴裁掉了右侧部分曲线，已按全部 36 点统一纵轴并修复相近横轴标签，数值没有变化。三张最终 PNG 均已解码及目视检查。

本次完成的是六张已跑表面的 fixed 指标和位移后处理；per-gait stance/swing/全足统计及后续扩大地形参数的实验仍未开始。幅度、PD、物理参数和已有轨迹保持原样。

## 当前短波恢复与仿真预览

**Demo 视频已完成：**每段六波数同屏，6 秒、1 倍速、20 fps、1440×1760。直接读取已保存的实际 Isaac 轨迹，原 USD 几何 OpenGL 回放，没有新增物理步，也没有改动幅度、PD 或地形。每段 120 帧均完整解码通过，六段的首／中／末帧已检查。[视频来源与核验](shortwave/demos/VIDEO_REPORT.json)。

三组 × 六波数的完整视频已统一列在[旁支发现的视频入口](#side-finding-shortwave)，Libo 与 Narvik 对照均保留原文件。

视频覆盖 t=0 至 5.95 s 的保存姿态，末帧显示到播放时刻 6 s；t=6.00 s 的终态另在静态预览图中。相机跟随身体中心、方向及比例固定，物理几何不作垂直夸张。静态图对应的旧渲染源码已保存在 `simulation_previews/rendering_source.py`，视频源码在 `demos/rendering_source.py`；首段完成后遇到读者元数据帧数 Infinity 无法写入严格 JSON 的记录错误，修复后复用该已通过完整解码的 MP4，失败与首版源码保留在 demos 中。

**本轮完成结果：**全部 36 条件的实际轨迹已保存，六组平地 cluster A/B 原样复用、全173关节 PD 原生读回完全一致，0.5 ms / 1 ms 时序、TGS64/16、四进程两线程、有限状态、隔离和边界检查全部通过，原生错误为零。[完整核验](shortwave/SIMULATION_VERIFICATION.json)。仿真和出图均已结束，无后台任务或关机安排。

先看六波数 × 三档地形的实际姿态总图：[Libo](shortwave/simulation_previews/libo_all_waves_all_terrains.png) · [Narvik](shortwave/simulation_previews/narvik_all_waves_all_terrains.png)。以下每张图包含全部六个波数：

| 来源 / 档位 | 斜视 4.5 s | 斜视 6 s | 俯视 6 s |
|---|---|---|---|
| Libo 原版 | [图](shortwave/simulation_previews/libo_original_oblique_t4.5.png) | [图](shortwave/simulation_previews/libo_original_oblique_t6.png) | [图](shortwave/simulation_previews/libo_original_top_t6.png) |
| Libo 4 mm | [图](shortwave/simulation_previews/libo_restore4_oblique_t4.5.png) | [图](shortwave/simulation_previews/libo_restore4_oblique_t6.png) | [图](shortwave/simulation_previews/libo_restore4_top_t6.png) |
| Libo 2 mm | [图](shortwave/simulation_previews/libo_restore2_oblique_t4.5.png) | [图](shortwave/simulation_previews/libo_restore2_oblique_t6.png) | [图](shortwave/simulation_previews/libo_restore2_top_t6.png) |
| Narvik 原版 | [图](shortwave/simulation_previews/narvik_original_oblique_t4.5.png) | [图](shortwave/simulation_previews/narvik_original_oblique_t6.png) | [图](shortwave/simulation_previews/narvik_original_top_t6.png) |
| Narvik 4 mm | [图](shortwave/simulation_previews/narvik_restore4_oblique_t4.5.png) | [图](shortwave/simulation_previews/narvik_restore4_oblique_t6.png) | [图](shortwave/simulation_previews/narvik_restore4_top_t6.png) |
| Narvik 2 mm | [图](shortwave/simulation_previews/narvik_restore2_oblique_t4.5.png) | [图](shortwave/simulation_previews/narvik_restore2_oblique_t6.png) | [图](shortwave/simulation_previews/narvik_restore2_top_t6.png) |

20 张 PNG 已逐文件解码检查。两个总图覆盖全部 36 条件；另检查了 Libo 2 mm 斜视、Narvik 2 mm 俯视。全部视角的机器人边界均在画框内，最小几何余量约 7.40 mm。[出图来源与相机记录](shortwave/simulation_previews/RENDER_REPORT.json)。

- 地形近景：[Libo](shortwave/libo_comparison.png) · [Narvik](shortwave/narvik_comparison.png)
- 三维总览与细节：[Libo](shortwave/libo_surfaces.png) · [Narvik](shortwave/narvik_surfaces.png)
- 频谱／高度差／局部坡度：[Libo](shortwave/libo_statistics.png) · [Narvik](shortwave/narvik_statistics.png)
- [地形构造和检查](shortwave/REPORT.json) · [共同网格统计](shortwave/COMMON_GRID_STATISTICS.json) · [本轮物理输入](shortwave/simulation_v2/campaign.json)
- 当前入口：`shortwave_terrain.py` 生成／从已保存表面出图；`run_shortwave.py` 准备／检查／启动固定短时批次；`render_shortwave.py` 从实际姿态出图；`verify_shortwave.py` 核验实际读回。已完成输入和结果禁止覆盖。

**用户再次明确：amplitude 原样沿用平地 cluster 优化结果；全部 173 关节 PD 固定为 9/17 wave-4 完整表。** 表中只显示六位小数，运行仍使用原 JSON 中的完整数值及原命令缩放流程。

| Wave number | Body A (deg) | Leg yaw B (deg) |
|---|---:|---:|
| 1.5 | 32.008333 | 24.702581 |
| 2 | 27.820833 | 29.000403 |
| 2.5 | 22.241667 | 34.976043 |
| 3 | 17.704167 | 33.332860 |
| 3.5 | 7.583333 | 43.288041 |
| 4 | 2.698958 | 41.851248 |

来源为主线 `campaign.json` 的 flat_low / flat_high cells，及对应 flat plans；row 级既有幅度缩放后只按原规则继续缩放 body yaw 和 leg yaw，leg pitch 不二次缩放。CPG 位置和速度均复用原缩放逻辑，没有优化或调 PD。

短波构造保留原来的 9/17 NPZ 为精确对照。先按原三角形将控制表面细分至 0.5 mm，以免加密改变 f(x,y)；原网格节点逐点相同，随机位置插值误差小于 7e-17 m。随后在 1.024 m 周期构造域中固定 <=62.5 cycles/m（>=16 mm）的复 Fourier 系数，16–8 mm 平滑过渡，短波功率来自相同原始扫描的二维方向性 PSD，两档共享相位；4 mm 档在 200–250 cycles/m 滚降，2 mm 档在 400–500 cycles/m 滚降。未重新归一总高度 RMS，也未用高度 CDF 投影改写长波。边界裁成原版相同的 1.022 m 顶点跨度；裁剪后的窗口频谱不声称与周期域逐点相同。

恢复档位描述的是新增／替换成分的频带；旧分片平面几何本身的微弱高频尾部保留。原始 0.5 mm 网格不是测量精度证明；这是实测频谱约束的合成细节，尚未验证噪声下限或自然形貌拓扑。预览中的局部 RMS 梯度不是全连接足点坡度角 RMS。

已完成仿真：2 来源 × 3 档地形 × 6 波数，每条件 6 s。沿用 CPU PhysX TGS64/16、0.5 ms 物理/front5 反馈、1 ms CPG、20 µm 接触偏移、原模型及四进程 × 两线程。按三个地形档位依次执行，每批四个 worker，各含三个实际条件；同一 wave 固定槽位。每 10 ms 保存轨迹，全部物理步做有限性和边界保护。使用相同的物理地形范围 X=[-223,321] mm、Y=[-353,81] mm，保留原顶点与三角形；所有 body 中心须离裁剪边界超过 15 mm。初始高度继续用原始 common.initial 规则按各完整地形的局部最大高度计算，因此高度可随地形变化。

图像使用真实 Isaac CPU PhysX 轨迹及原 USD 几何，通过已用过的 VTK OpenGL 后端回放，明确标注，并非原生 RTX 截屏。相机改变不重新推进物理。短时预览不构成完整性能验证、幅度优化或新的 roughness sweep。

已保留两类未完成尝试：`shortwave/simulation` 中 Narvik 低波数组在 5.089 s 进入最初较小裁剪域的 15 mm 保护区；随后统一扩大区域，未修改内部地形。`shortwave/simulation_v2/runs/v1` 在环境切换后进程退出，仅记录启动后的 0.1–0.3 s；中断证据在 `INTERRUPTION_v1.json`。最终完成结果使用相同配置的 `simulation_v2/runs/v2`，不覆盖旧尝试；三个批次端到端约 504 s。

以下是先前独立 PSD 初版及历史来源说明，保留作方法对照。


## 先看结果

- [Libo 原始／基准／增高／变密／变疏](preview/libo_comparison.png) · [Narvik 同类对照](preview/narvik_comparison.png)
- [Libo 完整三维表面](preview/libo_full.png) · [Narvik 完整三维表面](preview/narvik_full.png)
- [Libo 统计核对](preview/libo_statistics.png) · [Narvik 统计核对](preview/narvik_statistics.png)
- [六波数历史名义 stance/swing 窗口](preview/phase_windows.png)：供下一轮明确窗口，不等于实测触地。

近景统一为 100×100 mm、同一族共用颜色范围，黑点为相同的 38 足中立 XY 模板。三维总览为约 0.512 m 表面，垂直比例夸大以便观察；图示抽样 4 mm，保存的数据仍为 0.5 mm，细节看近景。

## 本次生成了什么

两类来源分别生成基准、高度 ×1.5、水平间距 ×0.5、水平间距 ×2，共 8 张表面。倍率是展示变化的例子，尚未作为完整实验的固定档位。水平变化组分别归一到该来源的同一高度 RMS；增高组严格为基准高度的 1.5 倍。

| 来源 | 基准 / 变密 / 变疏高度 RMS | 增高高度 RMS | 保存网格与范围 |
|---|---:|---:|---|
| Libo | 3.151799 mm | 4.727698 mm | 1024×1024，0.5 mm，顶点跨度 511.5 mm |
| Narvik | 0.775040 mm | 1.162560 mm | 同上 |

所有数值有限、谱能量归一、生成器频带、幅度比例、水平变化组等高度 RMS，以及保存后读回检查均通过。这些是构造正确性检查，**不代表已通过自然地形高保真验证**。

## 真实成分与处理方法

原始数据为 Libo-9-3-1-3.ply、Narvik-4-2-5-3.ply，各 201×201 点、0.5 mm 网格、100 mm 顶点跨度。来自原始岩石节理数据库，两个来源不混合；不是泛化到所有土壤的地形模型。原始文件位于 `X:/Data/Centipede_Terrain_Surface_2026-09-15/rock_joint/database`，来源及哈希可沿 `PREPARATION.json` 追溯。

1. 对原始表面去拟合平面，平面系数保存在报告中。
2. 用 Hann 窗提取二维方向性 PSD，平滑频谱并插值到更大空间域；不先压成径向谱。图中的径向曲线只用于展示。
3. 在 2048×2048、0.5 mm 的约 1.024 m 周期母场上生成新相位，用固定 40 次二维 IAAFT 类投影约束高度分布。最终投影严格满足已设定的 Fourier 幅值，高度分布允许小误差。新布局不是原扫描逐点重建。
4. 从同一个母场取基准、压缩和展开的区域。变疏用 Fourier 重采样，变密在满足频带限制后取样；各自去均值并归一到相同高度 RMS。高度 ×1.5 直接由基准相乘得到。
5. 保存 8 张表面并读回核验；没有改变机器人、PD 或动力学设置。

首轮共同母场频带约为 2.5–100.5 mm 波长；300–400 cycles/m 平滑滚降。这个上限用于让间距 ×0.5 后的高频仍低于输出 Nyquist，避免只有变密组额外损失短波。100.5 mm 是源栅格的 DFT 周期，实际源顶点跨度为 100 mm。间距 ×2 后最长可到约 201 mm，这是明确的尺度变换，不是新测得的长波；没有另行添加无实测依据的独立长波成分。

## 当前保真程度与限制

以 9 个同样大小的 100 mm 窗口比较，双方都先去平面。9 窗口来自同一合成表面，是相关描述性窗口，不是独立地点验证。

| 检查 | Libo 原始 → 合成窗口均值 | Narvik 原始 → 合成窗口均值 |
|---|---:|---:|
| 高度 RMS | 3.1518 → 3.1284 mm | 0.7750 → 0.7712 mm |
| 局部 RMS 梯度 | 0.5768 → 0.7628（+32.3%） | 0.1733 → 0.1625（−6.2%） |
| 梯度各向异性比 | 1.953 → 3.306 | 1.679 → 1.483 |

保存的完整基准表面相对于源高度分布，归一化 Wasserstein 距离约为 Libo 0.86%、Narvik 1.25%（除以源高度 RMS）。母场最终谱对构造目标的相对误差小于 1e-12；**这个目标是经过窗口、平滑、插值、频带限制与归一后的谱，不等于未经处理的实测谱**。

共同高频限制省略的源高度谱功率为 Libo 0.284%、Narvik 0.537%；按频率平方加权的谱梯度功率分别约为 54.5%、77.7%。谱梯度与表中的离散有限差分梯度并非同一口径。此结果说明“高度功率保留很多”不能推出“坡度细节也保留很多”。当前 Libo 的同尺寸窗口局部梯度和方向性仍有明显差异；Narvik 更接近这两项，但也未完成全面自然形貌验证。下一步应讨论这些差异的调整，不以高度 CDF 或算法目标谱匹配替代保真判断。

PSD 不包含相位，无法单独保证尖峰位置、沟槽连通性和局部拓扑。0.5 mm 是网格间距，也不代表测量精度。完整原始图、对照图、PSD、5/10/20/40/80 mm 间距的高度差统计都已保留供判断。

## 9 月 17 日 wave-4 criterion 的实际地形（已核对）

所选 PD/运行基准是 `cpu_demo_12s_n6_2026_09_17_v1` 的环境 5，原表第 28 行，`w4__historical_baseline__f1_s1501_p0`。运行 manifest 绑定的地形为 `X:/Data/Centipede_Terrain_Surface_2026-09-15/open_loop_review_2026-09-16/optimization_surfaces_v1/Libo-9-3-1-3_seed1501_512.npz`，SHA256 `caeede88cd6e4b2681090ae1d3112f75f81157646754642773b0ebd77d09e321`。同一 demo 的 wave-4 mild 环境 4 是 Narvik，不与所选 rough 条件混淆；wave 2、3、4 使用同一对地形文件，地形不随波数生成。

构建链：原始 Libo-9-3-1-3 PLY（201×201、0.5 mm、100 mm）→ Gaussian sigma=1 个原始像素平滑 → 每 4 点取样得到 51×51、2 mm → 去平面 → 沿 X/Y 镜像为 102×102 → 从镜像场计算无 Hann 窗的二维 PSD → 频谱 Gaussian sigma=0.8 平滑和插值 → 新随机相位、80 次谱/高度分布交替投影 → 最后一次谱投影 → 512×512、2 mm 的合成地形。

频谱把大于镜像 DFT 周期 204 mm 的波长移除，16–8 mm 波长之间滚降，8 mm 及更短波长移除。204 mm 来自数值镜像扩展，不是新增的 204 mm 实测范围。名义 seed=1501 经 SeedSequence([20260916,1,1,1501]) 派生实际 seed=10550352636565198984；因此不能只给 RNG 传入 1501 就重现它。生成高度 RMS 归一到 2 mm 参考的 3.188006 mm，没有额外手动 ×1.5 的运行时高度变换。

最终 NPZ 高度范围约 −6.046 至 +10.488 mm；FFT 周期 1.024 m，实际顶点跨度 1.022 m，文件后缀 `_512` 是网格点数而非 512 mm。worker 直接把高度作为顶点 Z，XY 居中于 ±0.511 m，每个格子两个三角形，共 522242 面，subdivision/碰撞近似均 none。2 mm 是空间网格，0.5 ms 是当时物理时间步，二者不同。

确切代码在 `X:/Projects/_archive/Codex/2026-09-11/con/terrain_surface_library_2026-09-15/build_library.py` 的 `generate`（33 行）和降采样（58 行）；大场生成调用在 `open_loop_review_2026-09-16/optimization/prepare_large_terrain.py`（42 行）；实际加载在 `X:/Projects/_archive/Codex/2026-09-15/ni/work/runtime_speed_2026-09-17/cpu_demo_12s/worker.py`（66–73 行）。本次核对 generator/preparer/reference/PLY/surface/worker/plan 的哈希和 manifest 绑定均一致，未重新生成或运行仿真。

与本次初版的区别：旧版是 2 mm 网格、镜像 PSD、约 8–204 mm 的频带；本次基准为 0.5 mm 网格、Hann 窗 PSD、约 2.5–100.5 mm 的频带。原始 Libo 扫描相同，但布局、频带及估计方法都不同。本次初版不冒充旧 criterion 的加密版本；精确旧地形文件保留作为已知基准。

## 两套 roughness 定义保持明确

指标来自旧对话 **Terrain Roughness**，采用全连接、不重复足点对的坡度角 RMS：

`h_i = f(x_i,y_i)`；`beta_ij = atan2(abs(h_i-h_j), ||xy_i-xy_j||)`。

每帧先对所选足点集的全部配对取 beta² 均值，再对位置/朝向/相位平均，最后开方并转为度。足端自身 Z 不进入指标；不把 pitch up/down 作为独立扫描参数。

- Fixed：一套中立位 38 足 XY，703 对，所有 wave number 共用。固定模板保存在 `preview/terrains.npz` 的 `fixed_foot_xy_m`，来自现有 common 模型零关节位的 XY 投影。
- Per-gait：最新六波数的名义 XY 随相位变化，全部足点、stance、swing 分别计算。历史窗口为 pitch>=0 的 19 足 nominal stance，其余 19 足 nominal swing；已绘图供讨论，还未开展新扫描。Swing 指标只描述投影下方的地形，不表示实际接触或净空。
- 共用同一张表面、同一采样坐标及朝向。本轮 fixed 已按用户选择在仿真区域内取位置并平均全部朝向；per-gait 扩展和相位窗口仍待后续确定，不做收敛或容量搜索。

当前 gait 几何由 `campaign.json`、`rows.json`、两个 flat plan 与主线 `simulation.py::_scale_commands` 对应的指令变换重建：原 CPG 缓存之后分别缩放 body yaw、leg yaw，leg pitch 保留；取 6000–6999 的 1 ms 周期。它包含完整 173 关节映射及源哈希，但不是地形接触、柔顺或 world-pitch 反馈下的实际动态姿态。旧 roughness 足点缓存与最新指令不同，未直接拿来当当前姿态。

## 文件与复现入口

- `terrain_preview.py generate`：生成初版；检测到已有 REPORT 时会停止，避免覆盖结果。
- `terrain_preview.py render`：只从已有数据刷新图片。
- `preview/terrains.npz`：唯一地形数据包，约 30 MiB；米为单位，行对应 Y、列对应 X，共用 `axis_m`。
- `preview/REPORT.json`：参数、源哈希、实际构造检查、窗口对照、各组统计与限制。
- `preview/generation_source.py`：实际执行的生成源码快照（证据，非新的入口）。当前入口只额外修复绘图坐标布局与空频带显示。
- `workflow.py`、`PREPARATION.json`、`measured_candidates.npz`、`current_gait_geometry.npz`：前序几何与来源准备。原始扫描只读保留，不重复搬运。

本次初版完成后没有后台任务、roughness 扫描、物理运行或自动关机安排。旧仿真与 Clean 未改写。


2026-10-01 late update: 2/4/8 mm through ×2 completed; the authorized ×2.25 stage is running after exclusion of two unvalidated native-startup-error cases, without scoring/retry or changes to PD/amplitude/physics. Whole-study final QA remains pending. [Current recovery and evidence](libo_cutoff_height_expansion_2026-10-01/README.md).
