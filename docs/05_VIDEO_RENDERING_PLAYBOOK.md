# Video Rendering Playbook

本文件是 clean release 的视频入口。先按用途选路线，不能把两条路线混用。

## 1. 路线 A：内部核验，默认

目的：检查 gait、spawn、terrain contact、flip/buckle 时序和 policy 行为。使用第三方离线渲染，不启动 RTX camera。

```powershell
python scripts\evaluate.py --terrain flat --fixed-wave 1.5 --num-envs 1 --steps 800 --video --headless --out outputs\demo
```

数据流：

```text
camera-free Isaac physics
  -> all_link_world_poses.npz (251 link world poses)
  -> scripts/render_offline.py
  -> video/offline_45deg.mp4
  -> video/offline_45deg.audit.json
```

这条路线不做 forward kinematics replay。每个 MJCF visual geom 直接放到 Isaac 记录的 link 世界位姿。21 个 body mesh、leg primitive 和 foot primitive 来自 `assets/offline_geometry/`。

地形必须保持两条物理约束：

1. `height[row,col] -> world(y,x)`，不转置、不翻轴。
2. Z 使用 `centipede.terrain.to_mujoco_collision`，不能画 mean-centred raw heightmap，否则机器人会视觉上浮空约一个 terrain offset。

高保真 renderer 还把裁剪后的 source pixel 索引、输出 shape 和 Z copy
误差写入 `high_fidelity_45deg.audit.json/terrain_alignment`。正式结果必须
满足所有 `terrain_alignment.checks=true` 且
`maximum_saved_z_copy_error_m=0`。为控制 Blender 网格规模可以只保留
轨迹附近的 source pixels 并做确定性抽样；抽样不得插值、转置或旋转 Z。

默认是干净的 45° chase 画面，无 HUD、标签或黑边。需要改变分辨率时用 `--video-width`、`--video-height`；需要改变采样帧率时用 `--video-fps`。

完成门槛是 `offline_45deg.audit.json` 中 `audit.pass=true`。审计同时要求：

- 黑帧、白帧、均匀空帧、局部白屏均为 0。
- strobe 和 clip outlier 均为 0。
- 每帧都检测到 robot red geometry。
- `mean_motion > 0`。

20-step smoke 只验证链路；科研视频使用 800 steps。

## 2. 路线 B：正式 Omniverse 导出

目的：PI 汇报或组会需要 Isaac/Omniverse 原生材质时使用。它不是定量数据来源，也不能替代路线 A 的核验。

clean runtime 不把 RTX viewport 绑进 `evaluate.py`。正式 exporter 必须作为独立工具遵守以下冻结规则：

1. 每个 condition 使用一个全新的 Isaac subprocess，`num_envs=1`；禁止一个 annotator 跨 reset/condition 复用。
2. 本机保持 Vulkan。不要把 D3D12 设备显示名当物理 GPU 型号；当前显示名可能被系统层改写，真实硬件仍由 Vulkan/PCI ID 核验。
3. 使用显式 `UsdGeom.Camera + omni.replicator.core render product + RGB annotator`；禁止 `Gymnasium RecordVideo` 和 `DirectRLEnv.render()` 默认 viewport。
4. 强制 `RaytracedLighting`，先做至少 50 次 render warmup。controller-pinned 单 clip 不在 warmup 后 reattach annotator，避免重新积累导致半段白屏。
5. 前 13 帧若全部 black/white/blank，或持续 partial-white/strobe，立即退出当前 boot；不要等完整 clip。
6. 最多 4 次 fresh-process retry。失败尝试保留 log 和 audit，不覆盖成功证据。
7. 每条成功视频必须运行 color-agnostic frame audit：black=0、white=0、blank=0、motion>0，并检查机器人在画面中。
8. 正式画面仍默认无标签、文字和黑边。只有 PI 明确要求展示 commanded/realized wave 时才加曲线。

原项目中形成这些规则的权威来源是：

- `docs/OFFLINE_RENDERER_PIPELINE.md`
- `docs/RENDERING_PIPELINE_DEBUG_NOTES.md`
- `docs/VISUAL_RENDERING_CLOSEUP_PIPELINE.md`
- `NOTES_RENDERING.md`

它们的结论已在这里压缩保留，历史脚本和失败样本没有搬入 clean release。

## 3. 路线 C：Blender 高保真展示皮肤

目的：论文、答辩或汇报需要真实蜈蚣形态与原 MuJoCo 配色时使用。它消费路线 A 已记录的精确 link 世界位姿，不重新运行 Isaac：

```powershell
python scripts\render_high_fidelity.py `
  --alllink-npz outputs\demo\flat\terrain_seed_244949\open_loop_w1.5\all_link_world_poses.npz `
  --out-dir outputs\demo\flat\terrain_seed_244949\open_loop_w1.5\video\high_fidelity
```

需要 Blender 5.x；默认位置是 `C:\Program Files\Blender Foundation\Blender 5.0\blender.exe`，其他位置用 `--blender` 指定。输出包括：

- `high_fidelity_45deg.mp4`：无标签、无黑边的 45° chase 视频。
- `high_fidelity_alignment.png`：首帧形态与绑定核验图。
- `high_fidelity_binding.json`：105 个高模 visual mesh 到 97 个当前刚体的映射、贴图和碰撞代理隔离审计。
- `high_fidelity_45deg.audit.json`：输入 hash、帧审计和颜色审计。

多机位使用同一条 trace，不重跑物理。`--view-name` 决定文件名，`--camera-offset=x,y,z` 决定相对轨迹质心的世界坐标偏移。负数开头的 offset 在 PowerShell 中必须使用等号形式：

```powershell
python scripts\render_high_fidelity.py --alllink-npz <TRACE> --out-dir <OUT> --view-name front --camera-offset='-0.26,0,0.15'
python scripts\render_high_fidelity.py --alllink-npz <TRACE> --out-dir <OUT> --view-name side  --camera-offset='0,-0.26,0.15'
python scripts\render_high_fidelity.py --alllink-npz <TRACE> --out-dir <OUT> --view-name top   --camera-offset='-0.035,-0.035,0.30'
```

默认 45° 机位为 `[-0.19, -0.15, 0.12] m`。它比最初的近景机位后移约 25%，用于减少高模表面接缝的视觉权重，并给头尾和触须留出稳定边距。上述三组 offset 已用首末帧确认：模型和触须不裁切，画面不露出深色 world background。非默认机位分别输出 `high_fidelity_<view>.mp4`、`high_fidelity_<view>_alignment.png`、binding JSON 和 audit JSON。

科研形态对照图可显式传入 `--terrain-color=r,g,b` 调整展示地面的线性 RGB，以保证正俯视时高模皮肤与平地有足够对比。默认值仍为 `0.22,0.24,0.21`，正式 locomotion 视频不得无记录地改变；该参数只影响下游展示材质，不改变轨迹、物理或碰撞。

同类静态对照还可显式使用 `--light-scale` 和 `--exposure` 抑制正俯视高光。两者默认分别为 `1.0` 和 `0.0`，所以既有正式视频外观不变；任何非默认值都必须写入输出审计，且禁止修改 packed textures、UV、色相或物理状态来补偿曝光。

同步 `2x2` 合成的固定象限顺序为 `[[45deg, front], [side, top]]`，不加标签或边框。当前正式输出为 `high_fidelity_multiview_2x2.mp4`，其 `high_fidelity_multiview_2x2.audit.json` 记录四条输入 hash、布局、193 帧审计和输出 hash。合成使用 imageio 附带的 FFmpeg，四路都缩放为 640x360 后依次 `hstack`/`vstack`，最终仍是 1280x720、20 fps；禁止补帧或改变时长。

高模位于 `assets/high_fidelity/source_Centipede_v1_CC0.blend`。身体、腿和尾部原始 packed textures、UV、sRGB 声明和材质节点全部保留；Blender 使用 AgX 和中性光，不做 hue 编辑。颜色审计把视频有色像素的中位 OpenCV hue 与旧 MuJoCo 参考值 19 比较，允许误差 6。`source_hue_preserved=true` 才通过。

这是严格的视觉替换，不是第二套机器人：

1. Isaac 的 USD、collision、mass、inertia、controller、observation、reward 和轨迹一律不变。
2. `.blend` 内 105 个 `col_*` 只用于旧资产注册，渲染时全部隐藏，永不进入 Isaac。
3. 21 个身体壳分别跟随 21 个 body link；旧 rig 最大 9.920 mm 的 mesh-center 偏移必须先归零，否则直线首帧看似正确、弯曲时会脱节。壳体仅在展示层使用 `[1.08, 1.04, 1.04]` 的纵向/横向/竖向尺度余量。
4. 源高模头壳相对自身后侧连接框架有 `-5.838°` 的几何俯角。渲染绑定以头壳后缘中心 `[0.00417015, 0, 0] m` 为支点，将头壳、左右颚肢、左右口器和两根触须根部整体旋转 `+16.5°`；最终头壳前端轴线为 `+8.094°`，必须落在 `8.0±1.0°` 审计带内。该修正只改展示 mesh 顶点，不改 `link_body_0` 的记录位姿、head joint target 或 collision。
5. 头后第一节壳体保留记录轨迹的 X/Y 与四元数，仅将展示层 world-Z 置于头壳和第二节壳体高度的平均值。直线参考帧修正为零，正式 193 帧轨迹中的最大修正为 1.804 mm；这不是物理位姿修改。
6. 76 个腿段先消除旧 MuJoCo rig 的横向参考偏移，再以当前关节端点绑定；每端向外延伸 0.50 mm、展示直径 1.05 mm，使同一膝关节两段总计重叠 1.00 mm，避免画面断缝。
7. Isaac 头部 link 的局部 `+X` 指向后方，因此两根触须绕随头壳移动后的头侧端点旋转到局部 `-X` 前方，最终左右各外展 18°、上扬 22°。这 22° 是抬头修正后的最终方向，不与 16.5° 重复相加。触须、颚肢、口器和终足仍按 PI 要求刚性附着头尾，不为它们新增物理关节。
8. 末节 `body_21.001` 只沿局部 X 轴缩短：局部 XYZ 系数为 `[0.67, 1.00, 1.00]`，再向倒数第二节平移 2.107 mm。头侧边界保持不动，最终约为 `8.56×7.16×5.27 mm`。左右终足必须和末节共享同一尺度，禁止恢复旧的 `[0.53, 0.30, 0.30]` 独立压缩；两者再额外向头侧嵌入 1.10 mm，根部搭入壳体 2.085/2.903 mm。终足共用一个纯展示坐标框，其局部 `+X` 每帧严格对齐 `link_body_19→link_body_20` 的尾向中心线切线，防止末节局部轴偏差让终足视觉上向前反折。顶视和侧视都应表现为顺着末节向尾侧延伸的两条分离终足，而不是一块综合色块。

绑定审计还必须满足 `body_mesh_centers_registered=true`、`leg_spans_match_joint_endpoints=true`、`head_visual_group_complete=true`、`head_visual_pitch_raised=true`、`head_visual_hinge_preserved=true`、`antennae_raised=true`、`antennae_forward=true`、`tail_shell_tapered=true`、`ultimate_legs_overlap_tail_shell=true` 和 `ultimate_legs_follow_terminal_tangent=true`。正式验收至少人工抽查首帧、中间帧和末帧，不能只看首帧或只信黑/白帧检测。

路线 C 适合视觉展示，但任何速度、存活或接触结论仍必须来自同目录的原始 metrics/trace，而不是从 Blender 画面重新测量。

## 4. 路线 D：碰撞模型、骨架与高模的连续结构揭示

目的：在同一条真实运动轨迹、同一个 45° chase 镜头和同一张碰撞地形上，展示三层模型之间的关系。先用 `evaluate.py --video` 记录足够长且全程存活的 exact-pose trace；随后只运行 Blender 展示层：

冻结的 canonical demo 优先使用一站式入口：

```powershell
python scripts\render_structure_reveal_demo.py
```

它默认核验并使用 `demo/structure_reveal/source/`，固定 120 fps 正式参数，且在已有 canonical 输出通过 hash/audit 时直接返回路径。显式重建使用 `--force`，短链路检查使用 `--smoke`，查看解析后的底层命令使用 `--dry-run`。

```powershell
python scripts\render_structure_reveal.py `
  --alllink-npz <TRACE> `
  --out-dir <OUT>
```

默认转场依次为：真实 MJCF `group="0"` 碰撞几何、从头到尾的碰撞壳退场、运动学骨架与近接触支撑网络、从头到尾套回 105 个高保真 visual mesh、完整高模继续行走。两次移动扫描环只标记当前转场前沿，不是物理对象。地形、机器人轨迹和 chase camera 在整个视频内连续，不通过拼接三个不同 renderer 制造假对齐。

`NearContactSupport` 节点由记录的 38 个足端世界坐标、精确 collision heightfield、足端碰撞半径 `0.45 mm` 和 `1.35 mm` 渐隐距离计算。它们表示近地支撑结构，**不是**原始 PhysX contact point、contact impulse 或 touch-sensor 读数；禁止在论文中将其改写为接触力证据。若需要真实接触冲量，必须重新运行 Isaac 并显式记录传感器量。

正式输出包括：

- `rough_structure_reveal_45deg.mp4`：无文字、无黑边的连续长视频。
- `01_collision_model_start.png` 与五张阶段关键帧：人工检查转场顺序和形态。
- `high_fidelity_binding.json`：沿用路线 C 的 105-mesh 绑定审计。
- `structure_reveal.scene_audit.json`：碰撞组、结构边、支撑语义和转场阶段审计。
- `rough_structure_reveal_45deg.audit.json`：帧数、黑白空帧、strobe、motion 与各阶段颜色证据。
- `MANIFEST.json`：trace/video hash、terrain、wave、camera、时长和所有证据路径。

正式长轨迹示例为 rough `L60/A12`、seed `244949`、open-loop wave `3.0`、`1667` control steps；source trace 在 20 fps 下应得到约 20 秒且不循环的 401 帧。正式展示可用 `--start-time-s 0.6` 跳过合法但视觉上无意义的 spawn settling，得到约 19.4 秒的连续运动；这只裁掉前缀，不循环、不补帧、不改变后续位姿。任何 `alive=false` 的保留区间会在 Blender 启动前被拒绝。

需要顺滑的 120 fps 汇报版本时使用：

```powershell
python scripts\render_structure_reveal.py `
  --alllink-npz <TRACE> `
  --out-dir <OUT> `
  --start-time-s 0.6 `
  --fps 120 `
  --render-fps 40 `
  --interpolate-pose-trace `
  --interpolate-video
```

该路线先从约 20.83 Hz 的 exact-pose trace 生成 40 fps 展示轨迹：刚体位置做线性插值，姿态做 shortest-arc quaternion SLERP；Blender 渲染 40 fps 后，再以 FFmpeg 双向运动补偿、AOBMC 和 VSBMC 生成 120 fps。两层插值都只服务于视觉连续性，**不是**新增的 120 Hz 物理、接触或传感器样本。原始 trace hash、40 fps 派生 trace hash、两层方法和帧数必须分别写入 `MANIFEST.json`。

路线 D 的展示地形仍复制 collision heightfield 的 Z 值和轴向，但正式高质量版本使用最高 `640` 点的确定性轨迹裁剪网格，并启用 smooth normals、低幅 procedural color variation 与 `0.22 mm` 视觉 micro-bump。这样隐藏规则网格方块而不修改地形高度、机器人接触或任何已记录轨迹；`MANIFEST.json/terrain/visual_surface/physics_geometry_unchanged` 必须为 `true`。

## 5. 本次故障如何判断

若无 `--video` 的 evaluate 产生 `metrics.json` 和 `traces.npz`，模型、环境、控制器与物理 rollout 主链成立。若加旧式 `--video` 后在 `omni.hydra.rtx` / `rtx.scenedb` 初始化阶段 access violation，那是路线 B 的 RTX 启动故障，不是 locomotion 代码故障。

`cuDeviceGetUuid` / CUDA error 36 在本机既出现在成功的 camera-free run，也出现在失败的 RTX run，不能单独作为根因。判断成功必须看最终 artifacts 和 audit，而不是看这一条 warning。

## 6. 手动重渲染已有 trace

不重跑 Isaac：

```powershell
python scripts\render_offline.py `
  --alllink-npz outputs\demo\flat\terrain_seed_244949\open_loop_w1.5\all_link_world_poses.npz `
  --out-mp4 outputs\demo\flat\terrain_seed_244949\open_loop_w1.5\video\offline_45deg_retry.mp4 `
  --report-json outputs\demo\flat\terrain_seed_244949\open_loop_w1.5\video\offline_45deg_retry.audit.json `
  --terrain
```

这一步纯 CPU，可反复调整视角或分辨率，不改变物理结果。
