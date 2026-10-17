"""Additional descriptive contact-loss plots from frozen combined/v03 data only."""
from pathlib import Path
import os, json, hashlib, argparse, shutil
from datetime import datetime, timezone

SOURCE=Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/combined/v03/RESULTS.json')
parser=argparse.ArgumentParser()
parser.add_argument('--out',required=True)
parser.add_argument('--replace-draft',action='store_true')
args=parser.parse_args()
out=Path(args.out)
if out.exists():
    assert args.replace_draft and out.parent.resolve()==Path(__file__).parent.resolve(),out
    previous=json.loads((out/'REPORT.json').read_text(encoding='utf-8'))
    for f in previous['new_figures']:
        assert Path(f['path']).parent.resolve()==out.resolve()
        assert hashlib.sha256(Path(f['path']).read_bytes()).hexdigest()==f['sha256']
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('MKL_NUM_THREADS','1')
os.environ.setdefault('MPLCONFIGDIR',str(out.parent/'mpl_cache'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter,StrMethodFormatter
from PIL import Image

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
source_sha=sha(SOURCE)
r=json.loads(SOURCE.read_text(encoding='utf-8'))
assert r['complete'] and r['planned_total_condition_count']==4055
groups=r['summaries'];flat={c['wave']:c for c in r['flat_controls']}
waves=[1.5,2.,2.5,3.,3.5];cutoffs=[2.,4.,8.]
colors=['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd']
metrics={'instantaneous':('A','Mean of valid 10 ms fractions'), 'cycle':('B','Mean of 1 s cycle fractions'),'pooled':('C','Pooled expected foot-time fraction')}
cellmap={(c['cutoff_mm'],c['height_scale'],c['wave'],c['spawn_index']):c for c in r['cells']}
points=[]
for g in groups:
    sp=g['forward_0_6_mm']['spawn_indices']
    keep=[cellmap[(g['cutoff_mm'],g['height_scale'],g['wave'],s)] for s in sp]
    assert len(keep)==g['n_retained']
    for m in metrics:
        saved=g['contact_loss_percent']['0_6'][m]
        assert saved['spawn_indices']==sp
        vals=np.array([c['contact_windows']['0_6']['loss_percent'][m] for c in keep],float)
        assert np.allclose(vals,saved['values'],rtol=0,atol=1e-12)
        assert np.isclose(vals.mean(),saved['mean'],rtol=0,atol=1e-12)
        assert np.isclose(vals.std(ddof=1),saved['std'],rtol=0,atol=1e-12)
    counts=np.array([c['contact_windows']['0_6']['missing_samples'] for c in keep],float)
    den=flat[g['wave']]['contact_windows']['0_6']['expected_samples']
    assert all(c['contact_windows']['0_6']['expected_samples']==den for c in keep)
    pooled=np.array(g['contact_loss_percent']['0_6']['pooled']['values'])
    assert np.allclose(100*counts/den,pooled,rtol=0,atol=1e-12)
    points.append(dict(cutoff_mm=g['cutoff_mm'],height_scale=g['height_scale'],wave=g['wave'],roughness_angle_rms_deg=g['fixed_pair_angle_rms_deg'],
       n_complete=g['n_complete_before'],n_failed=g['n_failed'],n_retained=g['n_retained'],retained_spawn_indices=sp,excluded_spawn_index=g['excluded_spawn_index'],
       forward_mm={k:g['forward_0_6_mm'][k] for k in ['mean','std']},
       loss_percent={m:{k:g['contact_loss_percent']['0_6'][m][k] for k in ['mean','std']} for m in metrics},
       missing_samples={'mean':float(counts.mean()),'std':float(counts.std(ddof=1)),'expected_samples':den}))
assert len(points)==135 and sum(p['n_retained'] for p in points)==3413
out.mkdir(parents=True,exist_ok=args.replace_draft)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'svg.fonttype':'none'})
outputs=[]
def save(fig,name):
    for suffix in ['.png','.svg']:
        p=out/(name+suffix);fig.savefig(p,dpi=170,facecolor='white')
        if suffix=='.png':
            with Image.open(p) as im:im.verify()
        outputs.append({'path':str(p),'sha256':sha(p)})
    plt.close(fig)
def style(ax):
    ax.grid(alpha=.16,zorder=0);ax.axhline(0,color='#777d85',alpha=.35,lw=.8)
    for s in ['top','right']:ax.spines[s].set_visible(False)
def rows(c,w):return sorted([p for p in points if p['cutoff_mm']==c and p['wave']==w],key=lambda p:p['height_scale'])
def curve(ax,x,y,ysd,color,w,xsd=None):
    ax.errorbar(x,y,yerr=ysd,xerr=xsd,fmt='none',ecolor=color,alpha=.23,capsize=3,lw=1.2,zorder=1)
    ax.plot(x,y,'-o',color=color,lw=2,ms=5,label=f'Wave {w:g}',zorder=3)
def legend(fig,ax,y=.90):
    h,l=ax.get_legend_handles_labels();fig.legend(h,l,ncol=5,loc='upper center',bbox_to_anchor=(.52,y),frameon=False)
def footer(fig,y=.112,extra=''):
    fig.text(.055,y,'0–6 s; same retained trials. One farthest-from-median complete point omitted per group; pale bars: sample SD.',fontsize=9.5)
    fig.text(.055,y-.055,extra or 'Pale bars: sample SD across shared spawns. Completed-trial means; failures have no six-second score.',fontsize=9.5)
def panels(title,ylabel):
    fig,axs=plt.subplots(1,3,figsize=(18.6,6.8),sharey=True,squeeze=False);axs=axs[0]
    fig.subplots_adjust(left=.055,right=.98,bottom=.24,top=.73,wspace=.13)
    fig.suptitle(title,y=.965,fontsize=18)
    for ax,c in zip(axs,cutoffs):style(ax);ax.set_title(f'Nominal cutoff {c:g} mm',pad=12)
    axs[0].set_ylabel(ylabel)
    return fig,axs
rmax=max(p['roughness_angle_rms_deg'] for p in points)
for m,(letter,label) in metrics.items():
    fig,axs=panels(f'Libo: observed contact loss vs. terrain roughness — {letter}', 'Missing expected-support portion (%)')
    fig.text(.52,.81,label,ha='center',fontsize=11,color='#49505b')
    for ax,c in zip(axs,cutoffs):
        for w,color in zip(waves,colors):
            rr=rows(c,w);xx=[0]+[p['roughness_angle_rms_deg'] for p in rr]
            yy=[0]+[p['loss_percent'][m]['mean'] for p in rr];ss=[np.nan]+[p['loss_percent'][m]['std'] for p in rr]
            curve(ax,xx,yy,ss,color,w)
        ax.set(xlim=(-.4,rmax+.8),ylim=(-2,102),xlabel='Shared fixed 38-foot / 703-pair angle RMS (deg)')
        ax.yaxis.set_major_formatter(PercentFormatter(xmax=100,decimals=0))
    legend(fig,axs[0]);footer(fig,extra='Same-phase flat reference for each wave; fractions describe contact differences, not calibrated random-erasure probabilities.')
    save(fig,'contact_loss_vs_roughness_'+m)

# Attachment-style paired view, using the user's selected A fraction.
fig,axs=plt.subplots(3,2,figsize=(13.7,15.2),sharex=True)
fig.subplots_adjust(left=.085,right=.98,top=.86,bottom=.12,hspace=.27,wspace=.21)
fig.suptitle('Libo: forward displacement and observed contact loss',y=.97,fontsize=19)
for ir,c in enumerate(cutoffs):
    left,right=axs[ir]
    for ax in [left,right]:style(ax);ax.set_xlim(-.4,rmax+.8)
    left.set_title(f'{c:g} mm cutoff · Forward displacement',fontsize=12)
    right.set_title(f'{c:g} mm cutoff · Contact loss A',fontsize=12)
    for w,color in zip(waves,colors):
        rr=rows(c,w);xx=[0]+[p['roughness_angle_rms_deg'] for p in rr]
        curve(left,xx,[flat[w]['forward_0_6_mm']]+[p['forward_mm']['mean'] for p in rr],[np.nan]+[p['forward_mm']['std'] for p in rr],color,w)
        curve(right,xx,[0]+[p['loss_percent']['instantaneous']['mean'] for p in rr],[np.nan]+[p['loss_percent']['instantaneous']['std'] for p in rr],color,w)
    left.set(ylim=(-35,245),ylabel='Forward COM displacement (mm)')
    right.set(ylim=(-2,102),ylabel='Missing expected-support portion (%)')
    right.yaxis.set_major_formatter(PercentFormatter(xmax=100,decimals=0))
for ax in axs[-1]:ax.set_xlabel('Shared fixed 38-foot / 703-pair angle RMS (deg)')
legend(fig,axs[0,0],y=.93)
fig.text(.085,.071,'A = mean of missing feet / expected flat feet at valid 10 ms snapshots; expected=0 snapshots omitted.',fontsize=10)
fig.text(.085,.041,'0–6 s; identical retained trials. Pale bars: sample SD. Single flat control per wave; uniform circle markers.',fontsize=10)
save(fig,'displacement_and_contact_loss_A_vs_roughness')

# Raw-count version requested by attachment; no new omission or failure substitution.
fig,axs=panels('Libo: displacement vs. missing expected support','Forward COM displacement (mm, 0–6 s)')
maxcount=max(p['missing_samples']['mean']+p['missing_samples']['std'] for p in points)
for ax,c in zip(axs,cutoffs):
    for w,color in zip(waves,colors):
        rr=rows(c,w)
        curve(ax,[0]+[p['missing_samples']['mean'] for p in rr],[flat[w]['forward_0_6_mm']]+[p['forward_mm']['mean'] for p in rr],
          [np.nan]+[p['forward_mm']['std'] for p in rr],color,w,xsd=[np.nan]+[p['missing_samples']['std'] for p in rr])
    ax.set(xlim=(-200,maxcount+250),ylim=(-35,245),xlabel='Missing expected support samples (count, 0–6 s)')
    ax.xaxis.set_major_formatter(StrMethodFormatter('{x:,.0f}'))
legend(fig,axs[0]);footer(fig,extra='One sample = one foot at one 10 ms snapshot. Lines follow increasing height; expected-support totals differ by wave.')
save(fig,'displacement_vs_missing_support_count')

references=[]
for m in metrics:
    for suffix in ['.png','.svg']:
        p=SOURCE.parent/('displacement_vs_'+m+'_loss_portion'+suffix)
        assert p.is_file();references.append({'metric':m,'path':str(p),'sha256':sha(p),'unchanged_existing_figure':True})
shutil.copyfile(__file__,out/'plot_observed_contact_loss.py')
report={'schema':'libo_observed_contact_loss_plot_update_v1','created_utc':datetime.now(timezone.utc).isoformat(),'complete':True,
  'source_results_path':str(SOURCE),'source_results_sha256':source_sha,'plot_source_sha256':sha(__file__),
  'source_counts':{'planned':4055,'complete_including_flat':3553,'failed':502,'retained_rough':3413,'flat_controls':5},
  'window_s':[0,6],'primary_metric':'instantaneous','contact_definitions':r['contact_definitions'],'foot_count':38,'snapshot_dt_s':.01,
  'flat_expected_sample_denominators':{str(w):flat[w]['contact_windows']['0_6']['expected_samples'] for w in waves},
  'statistic_selection':'Existing displacement-selected retained spawn sets exactly reused; no new omission or selection.',
  'retained_n_range':[min(p['n_retained'] for p in points),max(p['n_retained'] for p in points)],'groups':points,
  'line_order':'Increasing terrain height, not sorted by loss. No interpolation, smoothing or causal crosspoint fitting.',
  'new_figure_categories':5,'new_figures':outputs,'existing_displacement_loss_figures':references,
  'checks':{'all135_metric_values_match_retained_cells':True,'missing_count_normalization_matches_C_exactly':True,'original_results_unchanged':sha(SOURCE)==source_sha,'all_pngs_decode':True},
  'new_physics_steps':0,'manual_visual_review_complete':False,
  'limitations':['Complete-trial conditional means; especially height2.25 has large numerical failure fractions.','SD is across shared positions/headings on one terrain realization, not confidence intervals or independent terrain replicates.','Phase shifts count as missing support. A/B/C are descriptive equivalents, not fitted erasure probabilities.','Raw-count wave comparison uses different flat expected totals; C normalizes counts.']}
assert report['checks']['original_results_unchanged']
(out/'REPORT.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'out':str(out),'new_figure_categories':5,'pngs':5,'svgs':5,'existing_fwd_vs_loss_categories_reused':3,'retained_rough':3413,'no_new_physics':True},ensure_ascii=False))
