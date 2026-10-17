"""Read-only audit of the completed Libo wave 2.5/3 comparison."""
from pathlib import Path
import sys, json, hashlib, os
for key in ['OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS']:
    os.environ[key] = '1'
import numpy as np
ROOT = Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
sys.path.insert(0, str(ROOT))
from analyze_libo_repeats import extract_run
BASE = ROOT / 'libo_spawn_repeats'
OUT = Path(__file__).parent / 'outputs/wave2p5_3_audit_2026-09-30'
def read(path): return json.loads(path.read_text(encoding='utf-8-sig'))
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
paths = [BASE/'results/RESULTS.json', BASE/'extension_1p25_1p5/results/RESULTS.json']
derived_path = BASE/'trimmed_results/SUMMARY.json'
derived = read(derived_path)
raw = [x for p in paths for x in read(p)['cells']]
failed = [x for p in paths for x in read(p)['failed_cells']]
report = dict(complete=False, date='2026-09-30', new_physics_steps=0,
    definitions=dict(metric='All-251-body mass-weighted COM including rotated authored local offsets, signed projection along initial tail-to-head XY axis; 0-6 s and 4-6 s.',
    video='Preselected spawn2 only, 20 fps, last frame 5.95 s; each panel camera independently follows spine center.',
    statistics='Raw complete cases and user-specified one-farthest-from-median omission shown separately; failed runs have no six-second displacement.'),
    sources={str(p):sha(p) for p in [*paths,derived_path,ROOT/'analyze_libo_repeats.py',ROOT/'render_libo_extension.py',Path(__file__)]},
    comparisons=[],video_recomputations=[])
for height in [1.,1.25,1.5]:
    rs = {wave:{x['spawn_index']:x for x in raw if x['height_scale']==height and x['wave']==wave} for wave in [2.5,3.]}
    ds = {wave:next(x for x in derived['summaries'] if x['height_scale']==height and x['wave']==wave) for wave in rs}
    paired=[]
    for spawn in sorted(set(rs[2.5]) & set(rs[3.])):
        a,b=rs[2.5][spawn],rs[3.][spawn]
        paired.append(dict(spawn_index=spawn, wave2p5_mm=a['forward_0_6_mm'],wave3_mm=b['forward_0_6_mm'],
            difference_3_minus_2p5_mm=b['forward_0_6_mm']-a['forward_0_6_mm'],
            difference_4_6_mm=b['forward_4_6_mm']-a['forward_4_6_mm'],
            omitted_wave2p5=spawn==ds[2.5]['excluded_spawn_index'],omitted_wave3=spawn==ds[3.]['excluded_spawn_index']))
    keep=[x for x in paired if not x['omitted_wave2p5'] and not x['omitted_wave3']]
    def pair_stats(points):
        v=np.array([x['difference_3_minus_2p5_mm'] for x in points]);late=np.array([x['difference_4_6_mm'] for x in points])
        return dict(n=len(v),mean_3_minus_2p5_mm=float(v.mean()),std_3_minus_2p5_mm=float(v.std(ddof=1)),
            wave3_wins=int((v>0).sum()),mean_late_3_minus_2p5_mm=float(late.mean()),spawn_indices=[x['spawn_index'] for x in points])
    report['comparisons'].append(dict(height_scale=height,paired_raw=pair_stats(paired),paired_after_both_omissions=pair_stats(keep),pairs=paired,
        unpaired_raw={str(w):dict(n=len(rs[w]),mean_mm=float(np.mean([x['forward_0_6_mm'] for x in rs[w].values()]))) for w in rs},
        unpaired_trimmed={str(w):dict(n=ds[w]['n_retained'],mean_mm=ds[w]['forward_0_6_mm']['mean'],omitted_spawn=ds[w]['excluded_spawn_index']) for w in rs},
        failed=[dict(wave=x['wave'],spawn_index=x['spawn_index'],status=x['status']) for x in failed if x['height_scale']==height and x['wave'] in rs]))
series={}
with np.load(BASE/'extension_1p25_1p5/results/forward_timeseries.npz') as z:
    for height in [1.25,1.5]:
        for wave in [2.5,3.]:
            cell=next(x for x in raw if x['height_scale']==height and x['wave']==wave and x['spawn_index']==2)
            run=Path(cell['run_path']);active=read(run/'active.json');ex=extract_run(run,len(active['source_ids']),cell['start_yaw_deg']);env=cell['environment_index']
            key=f'h{height:g}_w{wave:g}_s02';fw=ex['forward'][:,env]
            np.testing.assert_allclose(fw,z[key],rtol=0,atol=1e-12)
            assert abs(fw[-1]-cell['forward_0_6_mm'])<1e-12
            ds=next(x for x in derived['summaries'] if x['height_scale']==height and x['wave']==wave)
            report['video_recomputations'].append(dict(height_scale=height,wave=wave,spawn_index=2,start_yaw_deg=cell['start_yaw_deg'],
                forward_axis_xy=ex['forward_xy'][env].tolist(),video_last_5p95_mm=float(fw[595]),forward_exact_6_mm=float(fw[-1]),
                forward_4_6_mm=float(fw[-1]-fw[400]),lateral_exact_6_mm=float(ex['lateral'][-1,env]),
                omitted_by_user_rule=ds['excluded_spawn_index']==2,
                max_abs_delta_from_video_metric_series_mm=float(np.max(abs(fw-z[key]))),
                trajectory_path=str(run/'trajectory.npz'),trajectory_sha256=sha(run/'trajectory.npz'),row_binding=active['source_ids']))
            for file in ['trajectory.npz','active.json','articulation_binding.json','amplitude_modulation.json']:
                report['sources'][str(run/file)]=sha(run/file)
            series[(height,wave)]=fw
video=read(BASE/'extension_1p25_1p5/demos/VIDEO_REPORT.json')
assert video['camera_follows_spine_center'] and video['selected_spawn_index']==2 and video['new_physics_steps_for_render']==0
assert video['forward_timeseries_sha256']==sha(BASE/'extension_1p25_1p5/results/forward_timeseries.npz')
for row in video['videos']:
    assert sha(Path(row['path']))==row['sha256']
    report['sources'][row['path']]=row['sha256']
report['sources'][str(BASE/'extension_1p25_1p5/demos/VIDEO_REPORT.json')]=sha(BASE/'extension_1p25_1p5/demos/VIDEO_REPORT.json')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.right':False,'axes.spines.top':False})
fig,axes=plt.subplots(2,2,figsize=(12,8));fig.subplots_adjust(left=.09,right=.97,top=.91,bottom=.14,hspace=.37,wspace=.25)
for column,height in enumerate([1.25,1.5]):
    ax=axes[0,column]
    for wave,color in [(2.5,'#26915a'),(3,'#bd4545')]:ax.plot(np.arange(601)*.01,series[(height,wave)],color=color,lw=2,label=f'Wave {wave:g}')
    ax.set(title=f'Height x{height:g}: video spawn2',xlabel='Time (s)',ylabel='Forward COM displacement (mm)');ax.legend(frameon=False);ax.grid(alpha=.15)
    ax=axes[1,column];p=next(x for x in report['comparisons'] if x['height_scale']==height)
    for row in p['pairs']:
        ax.plot([2.5,3],[row['wave2p5_mm'],row['wave3_mm']],color='#1d74b0' if row['spawn_index']==2 else '#8a9ba8',alpha=1 if row['spawn_index']==2 else .55,lw=2.5 if row['spawn_index']==2 else 1,marker='o',ms=6 if row['spawn_index']==2 else 4)
    for w,color in [(2.5,'#26915a'),(3,'#bd4545')]:
        y=p['unpaired_raw'][str(float(w))]['mean_mm'];ax.scatter([w],[y],s=75,color=color,marker='D',zorder=5,label=f'Wave {w:g} raw mean')
    ax.set(title=f'Common complete spawns: 3 wins {p["paired_raw"]["wave3_wins"]}/{p["paired_raw"]["n"]}',xticks=[2.5,3],xlabel='Wave number',ylabel='Forward displacement, 0-6 s (mm)');ax.grid(alpha=.15)
fig.suptitle('Libo: one demo spawn vs shared-spawn comparisons',fontsize=18,weight='bold')
fig.text(.09,.07,'Blue: demo spawn2. Gray: other common complete spawns. Diamonds: raw means over all complete cases.',fontsize=10,color='#52616f')
fig.text(.09,.025,'At height x1.5, wave3 / spawn2 is the observation omitted by the agreed median rule. Raw evidence remains.',fontsize=10,color='#52616f')
OUT.mkdir(parents=True,exist_ok=True);figure=OUT/'video_vs_shared_spawns.png';fig.savefig(figure,dpi=160);plt.close(fig)
report.update(complete=True,figure=dict(path=str(figure),sha256=sha(figure)),finding='At x1.5 demo spawn2 wave3 exceeds wave2.5, but only1/7 common complete spawns show this ranking. The demo wave3 case is the one omitted by the agreed rule; raw and paired comparisons still favor wave2.5 on average. No displacement projection or video binding mismatch found in this audit.')
(OUT/'AUDIT.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
print(json.dumps({k:report[k] for k in ['comparisons','video_recomputations','finding']},ensure_ascii=False))
