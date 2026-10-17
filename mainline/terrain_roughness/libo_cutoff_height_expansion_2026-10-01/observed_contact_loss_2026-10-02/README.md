# Libo rough terrain：等效contact loss与前向位移（2026-10-02）

<!-- combined-nine-overview -->
## 九张合并总览

行：2/4/8 mm截断；列：A/B/C。每格左侧loss–roughness、右侧前向位移–loss；与下面九张单图使用相同数据和保留试次。

![九张合并总览](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/all_nine_contact_loss.png)

[可缩放SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/all_nine_contact_loss.svg) · [原始PNG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/all_nine_contact_loss.png)

本次只用已提交的2/4/8 mm、高度×0.25–2.25结果做绘图。没有新增simulation，也没有改变PD、amplitude、物理或保留样本。0–6 s位移与contact loss共用3413个rough保留观察＋5个flat单一对照；每个rough组沿用原位移规则剔除一次，n=7–29。原报告和三张位移–loss比例图保留，避免复制相同图。

主口径A：每个有效10 ms时刻，缺失足数／同wave该时刻平地接触足数，再等权平均；平地预期接触为0的时刻跳过。missing是expected-flat有接触而rough没有接触，足编号和CPG时刻对应，阈值1e−8 N。B先在各1 s周期内汇总missing/expected再平均六周期，C为全窗口总missing/总expected。百分比是观察到的接触差异，不是通过随机擦除仿真拟合的p；相位变化也会计入missing。

各曲线对应wave1.5/2/2.5/3/3.5；2/4/8 mm是三个panel。平地同样圆点并连线，淡色误差条为样本STD。位移–loss图同时显示横向loss STD和纵向位移STD。连线按高度递增，不按loss重排序或拟合交点。

## 九张主图：三个截断 × 三种聚合

每张左侧为loss–roughness、右侧为位移–loss。[九张图与定义](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/README.md)。

| 最短波长截断 | A：逐10 ms比例平均 | B：逐1 s周期比例平均 | C：全0–6 s汇总比例 |
|---|---|---|---|
| 2 mm | [2 mm · A](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_A.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_A.svg) | [2 mm · B](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_B.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_B.svg) | [2 mm · C](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_C.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_C.svg) |
| 4 mm | [4 mm · A](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_A.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_A.svg) | [4 mm · B](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_B.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_B.svg) | [4 mm · C](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_C.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_C.svg) |
| 8 mm | [8 mm · A](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_A.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_A.svg) | [8 mm · B](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_B.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_B.svg) | [8 mm · C](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_C.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_C.svg) |


## 之前的多截断总览与附图

| 口径 | Contact loss vs roughness | Forward displacement vs contact loss |
|---|---|---|
| A逐10 ms平均 | [PNG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/contact_loss_vs_roughness_instantaneous.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/contact_loss_vs_roughness_instantaneous.svg) | [PNG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/displacement_vs_instantaneous_loss_portion.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/displacement_vs_instantaneous_loss_portion.svg) |
| B逐周期平均 | [PNG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/contact_loss_vs_roughness_cycle.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/contact_loss_vs_roughness_cycle.svg) | [PNG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/displacement_vs_cycle_loss_portion.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/displacement_vs_cycle_loss_portion.svg) |
| C全窗口比例 | [PNG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/contact_loss_vs_roughness_pooled.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/contact_loss_vs_roughness_pooled.svg) | [PNG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/displacement_vs_pooled_loss_portion.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/displacement_vs_pooled_loss_portion.svg) |

- [位移与A-loss随roughness的并列对照（参考附图布局）](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/displacement_and_contact_loss_A_vs_roughness.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/displacement_and_contact_loss_A_vs_roughness.svg)
- [位移 vs原始missing expected support样本数](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/displacement_vs_missing_support_count.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/displacement_vs_missing_support_count.svg)
- [135组数据、保留spawn、定义、源哈希与实际审阅记录](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/REPORT.json)

原始count的一个样本是一只足在一个10 ms快照的缺失预期支持。平地expected总量随wave不同：8360/7728/7366/7708/7421；因此count = C百分比×该wave分母/100，不能把同样换算套到A/B。三种比例图是主要比较，count版用于与旧图对照。

同roughness下wave1.5的A-loss最高，wave3.5通常最低；以contact loss作横轴后，中段位移曲线更靠近但没有完全重合。不同wave在同一loss横坐标可能对应不同地形高度，不把交点当成同地形的因果消融。

高地形数值失败较多，图中均值以完成轨迹为条件；失败不赋零，STD不是置信区间或独立地形重复。停止判断/样本不足及完整失败图继续以原研究交付为准，进一步加高与random-erasure仍暂停，自动跟进保持关闭。
