from pathlib import Path
import json, hashlib, shutil, time

HERE = Path(__file__).resolve().parent
PROJECT = Path('X:/Projects/Centipede_IsaacLab_RL')
ROOT = PROJECT/'mainline/terrain_roughness'
BASE = ROOT/'libo_cutoff_height_expansion_2026-10-01'

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p, v): Path(p).write_text(json.dumps(v, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
def read(p): return json.loads(Path(p).read_text(encoding='utf-8-sig'))

assert not (BASE/'PROTOCOL.json').exists()
files = ['build_libo_cutoff_height_terrains.py', 'run_libo_cutoff_height_expansion.py',
         'resume_libo_cutoff_height_expansion.py', 'analyze_libo_cutoff_height_expansion.py',
         'continue_libo_cutoff_height_expansion.py']
for name in files:
    if (ROOT/name).exists() and name == 'run_libo_cutoff_height_expansion.py':
        assert sha(ROOT/name)==sha(HERE/name), 'Prepared campaign runner must remain unchanged'
    else:
        shutil.copy2(HERE/name, ROOT/name)
sources = {str(ROOT/name):sha(ROOT/name) for name in files}
shared_path = ROOT/'libo_spawn_repeats/extension_spawn30_2026-09-30/simulation/campaign.json'
shared = read(shared_path)
snapshot = BASE/'orchestration_source_v1'
snapshot.mkdir()
for name in files: shutil.copy2(ROOT/name, snapshot/name)
shutil.copy2(__file__, snapshot/'setup_source.py')
initial = BASE/'initial_4mm_h1p75_2p00/simulation'
assert read(initial/'PREPARATION_CHECK.json')['passed']
assert read(initial/'campaign.json')['condition_count']==300
preview_dir = BASE/'terrains/initial_4mm_h1p75_2p00'
new_preview = preview_dir/'libo_cutoff_height_preview_layout_v2.png'
shutil.copy2(HERE/'terrain_preview_layout_v2_review/libo_cutoff_height_preview.png', new_preview)
dump(preview_dir/'PREVIEW_LAYOUT_REVIEW.json', dict(
    reviewed=True, preview_path=str(new_preview), preview_sha256=sha(new_preview),
    original_preview_preserved=True, original_generation_report_and_manifest_unchanged=True,
    terrain_arrays_unchanged=True, review='Viewed both panels, labels/colorbar/footer fully visible; common camera/Z range.',
    original_layout_note='Archived initial preview clips long labels; repaired layout saved separately, no pinned input mutation.'))
protocol = dict(schema='libo_cutoff_height_expansion_protocol_v1', explicit_start=True,
    authorization='2026-10-01 user: 好，做1.75以上高度和2/8mm截断吧，今晚都做完，你有什么问题吗',
    stop_rule_user_confirmed=True, stop_rule_reply='采用这个判据（建议）',
    priority='matched_through2_then_increment', priority_source='Suggested order asked asynchronously; adopted after allowing time to reply. User authorized all higher-height/cutoff work; no extra approval gate.',
    initial_new_condition_count=2700,
    first_stage=dict(name='initial_4mm_h1p75_2p00',cutoffs=[4],heights=[1.75,2.],conditions=300),
    second_stage=dict(name='matched_2_8mm_h0p25_2p00',cutoffs=[2,8],heights=[.25,.5,.75,1.,1.25,1.5,1.75,2.],conditions=2400),
    continuation=dict(height_increment=.25,start_height=2.25,conditions_per_active_cutoff_per_height=150,
        policy='Evaluate each cutoff independently; extend progressing cutoffs. An eligible raw stall decision stops further heights for that cutoff. Insufficient complete trajectories hold that cutoff for user review, never classify numerical failures as stalled.'),
    stop_rule=dict(wave=3.5,window_s=[4,6],signed_forward_threshold_mm=2,minimum_complete=24,
        minimum_stalled_fraction=.8,planned_attempts=30,apply_outlier_omission=False,failed_trials_count_as_stalled=False),
    waves=[1.5,2,2.5,3,3.5],shared_spawn_campaign=str(shared_path),shared_spawn_campaign_sha256=sha(shared_path),
    spawn_indices=list(range(30)),new_flat_conditions=0,reused_previous_rough_conditions=900,reused_flat_controls=5,
    frozen_profile_sha256=shared['profile_sha256'],fixed_173_PD_sha256=shared['fixed_pd_sha256'],
    amplitudes=shared['flat_cluster_cells'],physics_dt_s=.0005,CPG_command_dt_s=.001,
    workers=4,threads_per_worker=2,environments_per_formal_batch=[3,2,3,2],
    terrain_policy='Use existing fine2/fine4 mother files, exact triangle subdivision of the historical nominal8 mother; all0.5mm grid, fixed area/phases and mean-preserving height multiplication. No RMS matching/filtering/new phase draw.',
    forward_metric='Mass-weighted full-body COM projected onto each initial forward heading; signed displacement.',
    statistics='Exactly one complete point farthest from group median omitted; tie lowestspawn index; same retained set for0–6/2–6/4–6 and A/B/C; sampleSTD ddof1. Failures separately retained.',
    boundary_guard_m=.015,paused_other_studies=['random_erasure_4060','random_erasure_light_remaining75','Newton_softleg','Clean_adoption'],
    no_shutdown=True,no_parameter_or_convergence_or_capacity_search=True,
    time_estimate=dict(reference_conditions=600,reference_active_dispatch_min=132,
        initial_2700_expected_active_hours=9.9,extra_full_three_cutoff_height_expected_min=99,
        note='Planning estimate, excludes preparation/postprocessing and variable failures; unbounded height continuation cannot be guaranteed overnight.'),
    orchestration_sources=sources,created_unix=time.time())
dump(BASE/'PROTOCOL.json', protocol)
intro = '''# Libo 高度与截断扩展（2026-10-01）

用户已启动更高地形和2/8 mm比较。当前首批输入核验完成，物理尚待启动；实际运行状态以 [STATUS.json](STATUS.json) 为准。

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
'''
(BASE/'README.md').write_text(intro,encoding='utf-8')
note = '**本轮已授权并完成启动准备（2026-10-01）：**Libo×1.75以上高度与2/8 mm截断扩展，30共享spawn、固定A/B/PD/物理。wave3.5停跑标准已确认：至少24完整、80%末两秒≤2 mm；数值失败不当卡住。先共同补到×2，再按×0.25继续。实际状态和新证据见[高度/截断扩展](LINK)。random-erasure继续暂停，无关机。\n\n'
for path, link in [(PROJECT/'PROJECT_STATE.md','mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/README.md'),
                   (PROJECT/'mainline/README.md','terrain_roughness/libo_cutoff_height_expansion_2026-10-01/README.md'),
                   (ROOT/'README.md','libo_cutoff_height_expansion_2026-10-01/README.md')]:
    text=path.read_text(encoding='utf-8-sig')
    first, rest=text.split('\n',1)
    text=first+'\n\n'+note.replace('LINK',link)+rest.lstrip('\n')
    text=text.replace('higher-height/2/8mm及random-erasure继续暂停。','该轮结束时higher-height/2/8mm与random-erasure暂停；本轮以最新启动条目为准。')
    text=text.replace('其余higher-height/cutoff和random-erasure仍暂停。','其他任务以最新启动条目为准；random-erasure继续暂停。')
    path.write_text(text,encoding='utf-8')
rules=PROJECT/'CLAUDE.md'
text=rules.read_text(encoding='utf-8-sig')
text+='\n\n2026-10-01 latest user START supersedes the higher-height/cutoff hold only. Protocol: mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/PROTOCOL.json. Keep30 sharedspawns, all frozen A/B/PD/physics and original nativeworker. Raw wave3.5 last2s<=2mm in>=80%complete, min24complete; failures nevercountasstall. Firstmatchedgrid throughheight2 then+.25. Inconclusivecutoff held fordiscussion; randomerasure/Newton/Clean held; no shutdown orcapacity/convergencesearch. Prior completion counts and historical holds remain historical evidence.\n'
rules.write_text(text,encoding='utf-8')
agents=HERE/'AGENTS.md'
text=agents.read_text(encoding='utf-8')
text+='\n2026-10-01 expansion preparation: exact first3004mm×1.75/×2 inputs passed; matched-through×2 order adopted as proposed after pending preference was given time to respond. Canonical protocol/status: X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01. Initial2700new conditions, then150peractivecutoff per+.25 height. Nativequeue is started only by the owned controller; consultSTATUS/supervisor and exactPID identities before any restart. Raw stoprule confirmed as above; incomplete wave3.5groups pause thatcutoff fordiscussion, notclaimstall. Otherstudies/no-shutdown constraints unchanged.\n'
agents.write_text(text,encoding='utf-8')
print(json.dumps(dict(protocol=str(BASE/'PROTOCOL.json'),initial_preparation_passed=True,orchestration_sources=sources),ensure_ascii=False))
