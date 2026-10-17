# 九张contact-loss图：三个截断 × 三种聚合

<!-- combined-nine-overview -->
## 九张合并总览

行：2/4/8 mm截断；列：A/B/C。每格左侧loss–roughness、右侧前向位移–loss；与下面九张单图使用相同数据和保留试次。

![九张合并总览](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/all_nine_contact_loss.png)

[可缩放SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/all_nine_contact_loss.svg) · [原始PNG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/all_nine_contact_loss.png)

这里“九张”按2/4/8 mm三个截断 × A/B/C三种aggregation计算。每张均有两个panel：左为contact loss vs terrain roughness，右为forward displacement vs contact loss；不同曲线对应五个wave number。

A：每个有效10 ms快照计算missing/expected，再平均；expected=0时刻跳过。B：每个1 s周期先汇总missing/expected，再平均六周期。C：全0–6 s窗口总missing/总expected。

所有图沿用相同0–6 s窗口、既有剔除规则及保留spawn集合。淡色为sample STD，右图两轴都有STD；平地同样圆点并连线。只做已完成数据绘图，无新simulation。高地形均值以完整轨迹为条件，失败不赋零。

| 最短波长截断 | A：逐10 ms比例平均 | B：逐1 s周期比例平均 | C：全0–6 s汇总比例 |
|---|---|---|---|
| 2 mm | [2 mm · A](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_A.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_A.svg) | [2 mm · B](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_B.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_B.svg) | [2 mm · C](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_C.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_C.svg) |
| 4 mm | [4 mm · A](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_A.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_A.svg) | [4 mm · B](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_B.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_B.svg) | [4 mm · C](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_C.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_C.svg) |
| 8 mm | [8 mm · A](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_A.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_A.svg) | [8 mm · B](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_B.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_B.svg) | [8 mm · C](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_C.png) · [SVG](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_C.svg) |

[逐图来源、哈希及审阅记录](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/REPORT.json)

## 2 mm截断

![2 mm · aggregation A](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_A.png)

![2 mm · aggregation B](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_B.png)

![2 mm · aggregation C](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_2mm_aggregation_C.png)


## 4 mm截断

![4 mm · aggregation A](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_A.png)

![4 mm · aggregation B](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_B.png)

![4 mm · aggregation C](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_4mm_aggregation_C.png)


## 8 mm截断

![8 mm · aggregation A](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_A.png)

![8 mm · aggregation B](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_B.png)

![8 mm · aggregation C](X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/observed_contact_loss_2026-10-02/nine_cutoff_aggregation/cutoff_8mm_aggregation_C.png)

