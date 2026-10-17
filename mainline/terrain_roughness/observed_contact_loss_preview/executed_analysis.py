"""Proposal preview from existing flat/rough records only; no simulator."""
from pathlib import Path
import os, json, hashlib, itertools
for k in ['OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS']:
    os.environ[k] = '1'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

ROOT = Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
OUT = ROOT / 'observed_contact_loss_preview'
INPUT = ROOT / 'contact_erasure_light_2026-09-30/rough_inputs/ROUGH_INPUTS.json'
ORIGINAL = ROOT / 'libo_spawn_repeats/trimmed_results/SUMMARY.json'
WAVES = [1.5, 2, 2.5, 3, 3.5]
COLORS = ['#2474b5', '#df7c13', '#26915a', '#bd4545', '#8862b2']

def read(p): return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def meanstd(v):
    return dict(n=len(v), mean=float(np.mean(v)) if len(v) else None,
                std=float(np.std(v, ddof=1)) if len(v)>1 else None, values=[float(x) for x in v])

r = read(INPUT); original = read(ORIGINAL)
assert r['complete'] and r['raw_completed_count'] == 297 and r['sample_dt_s'] == .01
for path, digest in r['source_hashes'].items(): assert sha(path) == digest, path
threshold = r['force_presence_threshold_N']; assert threshold == 1e-8
OUT.mkdir(exist_ok=False)
(OUT / 'executed_analysis.py').write_bytes(Path(__file__).read_bytes())
sources = {str(INPUT):sha(INPUT), str(ORIGINAL):sha(ORIGINAL)}
cache = {}
def contact(c):
    path = Path(c['run_path'])/'cluster_contacts.npz'
    if str(path) not in cache:
        with np.load(path) as z:
            cache[str(path)] = (z['t'].copy(), z['force'].copy(), z['foot_names'].tolist())
        sources[str(path)] = sha(path)
    t, f, names = cache[str(path)]
    assert names == [f'foot_{i//2}_{i%2}' for i in range(38)]
    np.testing.assert_allclose(t, np.arange(1,601)/100, atol=1e-12, rtol=0)
    force = f[:,c['environment_index']]
    assert force.shape == (600,38) and np.isfinite(force).all()
    return t, force, force>threshold

flats = {c['wave']:c for c in r['cells'] if c['height_scale']==0}
refs = {w:contact(flats[w]) for w in WAVES}
metrics = []
for c in r['cells']:
    t, f, observed = contact(c)
    tf, ff, expected = refs[c['wave']]
    np.testing.assert_array_equal(t, tf)
    missing = expected & ~observed
    gained = ~expected & observed
    retained = expected & observed
    m = {k:c[k] for k in ['height_scale','wave','spawn_index','run_path','environment_index']}
    m['forward_0_6_mm'] = c['forward_0_6_mm']
    m['windows'] = {}
    for label, start in [('0_6',0), ('2_6',2), ('4_6',4)]:
        ix = t>start+1e-10
        eligible = int(expected[ix].sum()); assert eligible>0
        m['windows'][label] = dict(expected_support_samples=eligible,
            missing_samples=int(missing[ix].sum()), retained_samples=int(retained[ix].sum()),
            gained_samples=int(gained[ix].sum()),
            loss=float(missing[ix].sum()/eligible),
            gained_foot_time_fraction=float(gained[ix].mean()),
            loaded_foot_time_fraction=float(observed[ix].mean()),
            flat_force_weighted_missing=float((ff[ix]*missing[ix]).sum()/ff[ix].sum()))
    assert abs(m['windows']['2_6']['loss']-c['phase_matched_foot_support_missing_fraction'])<1e-12
    metrics.append(m)

omap = {(s['height_scale'],s['wave']):s for s in original['summaries']}
summaries=[]
for height,w in itertools.product([0]+original['height_scales'],WAVES):
    if height==0:
        ids=[flats[w]['spawn_index']]; old=next(c for c in original['flat_controls'] if c['wave']==w)
        displacement=meanstd([old['forward_0_6_mm']]); roughness=0.
    else:
        old=omap[(height,w)]; ids=old['forward_0_6_mm']['spawn_indices']
        displacement=old['forward_0_6_mm']; roughness=old['fixed_pair_angle_rms_deg']
    cells=[c for c in metrics if c['height_scale']==height and c['wave']==w and c['spawn_index'] in ids]
    assert len(cells)==len(ids)
    assert abs(np.mean([c['forward_0_6_mm'] for c in cells])-displacement['mean'])<1e-10
    summaries.append(dict(height_scale=height,wave=w,fixed_pair_angle_rms_deg=roughness,
        retained_spawn_indices=ids,n=len(cells),original_forward_0_6_mm=displacement,
        contact_loss_0_6=meanstd([c['windows']['0_6']['loss'] for c in cells]),
        contact_loss_4_6=meanstd([c['windows']['4_6']['loss'] for c in cells]),
        gain_0_6=meanstd([c['windows']['0_6']['gained_foot_time_fraction'] for c in cells])))

crossings=[]
for a,b in itertools.combinations(WAVES,2):
    aa=sorted([s for s in summaries if s['wave']==a],key=lambda s:s['height_scale'])
    bb=sorted([s for s in summaries if s['wave']==b],key=lambda s:s['height_scale'])
    for lo in range(len(aa)-1):
        x=aa[lo]['original_forward_0_6_mm']['mean']-bb[lo]['original_forward_0_6_mm']['mean']
        y=aa[lo+1]['original_forward_0_6_mm']['mean']-bb[lo+1]['original_forward_0_6_mm']['mean']
        if x*y<0:
            crossings.append(dict(wave_a=a,wave_b=b,height_bracket=[aa[lo]['height_scale'],aa[lo+1]['height_scale']],
                roughness_bracket_deg=[aa[lo]['fixed_pair_angle_rms_deg'],aa[lo+1]['fixed_pair_angle_rms_deg']],
                mean_difference_endpoints_mm=[x,y],confirmed=False,
                interpretation='Sign reversal of existing mean curves at adjacent shared terrain conditions; not a new experiment or precise root.'))

examples=[]
fig,axs=plt.subplots(3,2,figsize=(13,8.5),sharex=True,constrained_layout=True)
for col,w in enumerate([1.5,3.5]):
    c=next(c for c in r['cells'] if c['height_scale']==1.5 and c['wave']==w and c['spawn_index']==2)
    t,f,observed=contact(c); expected=refs[w][2]; ix=t>2+1e-10
    states=np.zeros(expected.shape,dtype=np.uint8)
    states[expected & ~observed]=1;states[expected & observed]=2;states[~expected & observed]=3
    loss=float(((expected & ~observed)[ix]).sum()/expected[ix].sum())
    for row,data in enumerate([expected,observed,states]):
        cmap=ListedColormap(['#eef1f5','#294b6c']) if row<2 else ListedColormap(['#eef1f5','#df9d36','#294b6c','#42a89b'])
        axs[row,col].imshow(data[ix].T,aspect='auto',interpolation='nearest',origin='lower',
                            extent=[2.005,6.005,-.5,37.5],cmap=cmap,vmin=0,vmax=1 if row<2 else 3)
        axs[row,col].set_yticks([0,9,18,27,37],['0L','4R','9L','13R','18R'])
        axs[row,col].set_ylabel(['Flat support','Rough support','Difference'][row])
    axs[0,col].set_title(f'Wave {w:g} | Libo height x1.5 | spawn 2 | missing {loss:.1%}')
    axs[2,col].set_xlabel('CPG clock time (s), same-phase samples')
    examples.append(dict(wave=w,height_scale=1.5,spawn_index=2,loss_2_6=loss,run_path=c['run_path'],flat_reference=flats[w]['run_path']))
fig.legend(handles=[Patch(facecolor='#df9d36',label='Expected flat support missing'),Patch(facecolor='#294b6c',label='Support retained'),
                    Patch(facecolor='#42a89b',label='New support relative to flat'),Patch(facecolor='#eef1f5',label='No support in either')],
           loc='outside lower center',ncol=2,frameon=False)
fig.suptitle('Existing flat/rough contact patterns: 38 feet, 10 ms snapshots',fontsize=16)
fig.savefig(OUT/'contact_pattern_examples.png',dpi=160);plt.close(fig)

fig,axs=plt.subplots(1,2,figsize=(13,5.4));fig.subplots_adjust(left=.075,right=.99,bottom=.24,top=.83,wspace=.25)
for w,color in zip(WAVES,COLORS):
    rows=sorted([s for s in summaries if s['wave']==w],key=lambda s:s['height_scale']);xx=[s['fixed_pair_angle_rms_deg'] for s in rows]
    for ax,field,factor in [(axs[0],'original_forward_0_6_mm',1),(axs[1],'contact_loss_0_6',100)]:
        yy=[s[field]['mean']*factor for s in rows];sd=[(s[field]['std'] or 0)*factor for s in rows]
        ax.plot(xx,yy,'-o',color=color,lw=2,ms=5,label=f'Wave {w:g}')
        ax.errorbar(xx,yy,yerr=sd,fmt='none',ecolor=color,alpha=.24,capsize=3)
        ax.grid(alpha=.18);ax.set_xlabel('Shared fixed 38-foot / 703-pair angle RMS (deg)')
axs[0].set_title('Original displacement curves preserved');axs[0].set_ylabel('Forward COM displacement, 0-6 s (mm)')
axs[1].set_title('Observed missing flat support, same trials');axs[1].set_ylabel('Missing expected-support samples (%)');axs[1].set_ylim(0,100)
fig.legend(*axs[0].get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.94),ncol=5,frameon=False)
fig.text(.075,.13,'Same retained spawn sets as original user-approved displacement plots; no new omission by contact loss.',fontsize=10)
fig.text(.075,.075,'Flat: one reference per wave, no SD. Loss counts missing flat contacts; gains and timing changes are separate.',fontsize=10)
fig.text(.075,.025,'Preview from existing data only.10 ms snapshots,38 toes; loss is not calibrated Bernoulli erasure probability or anchoring force.',fontsize=9.5,color='#52616f')
fig.savefig(OUT/'original_displacement_and_observed_loss.png',dpi=160);plt.close(fig)

report=dict(status='proposal_preview_not_adopted_trial_definition',new_physics_steps=0,source_hashes=sources,
            source_code_sha256=sha(__file__),windows_s=[[0,6],[2,6],[4,6]],sample_dt_s=.01,threshold_N=threshold,
            raw_complete_trials=len(metrics),raw_numerical_failure_count=r['failed_count'],
            statistics='Contact summaries use exactly the old displacement-retained spawn sets; raw metrics also retained. Original mean/SD values are unchanged.',
            definitions=dict(contact='Positive sum of native foot normal forces above1e-8N',
              loss='sum_t,i C_flat_w(t,i)*(1-C_rough_w(t,i)) / sum_t,i C_flat_w(t,i)',
              gain='sum_t,i (1-C_flat_w(t,i))*C_rough_w(t,i)/(38*n_samples)',
              force_weighted_missing='Flat normal-force weight on expected samples missing on rough; not total force deficit.'),
            alignment='Fixed corresponding foot IDs and same commanded CPG clock; no response-driven phase shift fitting.',
            limitations=r['limitations']+['Wave-specific contact-loss axes do not preserve common-terrain comparisons or crossover positions.',
             'Observed missing support, gained contacts, timing shift and slip cannot be reduced to one independently sampled erasure p.',
             'Crossing is taken from original displacement vs shared terrain roughness; contact analysis is explanatory and does not prove causation.'],
            examples=examples,cells=metrics,summaries=summaries,original_mean_curve_crossing_brackets=crossings)
(OUT/'PREVIEW.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
print(json.dumps(dict(raw_trials=len(metrics),retained=sum(s['n'] for s in summaries),crossing_brackets=crossings,examples=examples)))
