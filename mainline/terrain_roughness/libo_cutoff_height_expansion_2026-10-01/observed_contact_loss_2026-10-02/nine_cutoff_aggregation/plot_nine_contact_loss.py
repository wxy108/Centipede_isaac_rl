"""Nine cutoff x aggregation comparisons, from committed observations only."""
from pathlib import Path
import os, json, hashlib, shutil
from datetime import datetime, timezone
V=Path(__file__).parent
OUT=V/'nine_contact_loss_2026-10-02'
BASE=Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01')
PRIOR=BASE/'observed_contact_loss_2026-10-02/REPORT.json'
os.environ['MPLCONFIGDIR']=str(V/'mpl_cache')
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter
from PIL import Image
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
prior=json.loads(PRIOR.read_text(encoding='utf-8'))
source=Path(prior['source_results_path'])
assert sha(source)==prior['source_results_sha256']
r=json.loads(source.read_text(encoding='utf-8'))
points=prior['groups'];flat={c['wave']:c for c in r['flat_controls']}
assert len(points)==135 and sum(p['n_retained'] for p in points)==3413
assert prior['manual_visual_review_complete'] and prior['window_s']==[0,6]
assert not OUT.exists(),OUT
OUT.mkdir()
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'svg.fonttype':'none'})
waves=[1.5,2.,2.5,3.,3.5]
colors=['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd']
metrics=[('instantaneous','A','Mean of valid 10 ms missing / expected-flat fractions'),
         ('cycle','B','Mean of six 1 s cycle pooled missing / expected-flat fractions'),
         ('pooled','C','Whole 0–6 s pooled missing / expected-flat fraction')]
rmax=max(p['roughness_angle_rms_deg'] for p in points)
figures=[]
def style(ax):
    ax.grid(alpha=.16,zorder=0)
    ax.axhline(0,color='#777d85',alpha=.35,lw=.8)
    for s in ['top','right']:ax.spines[s].set_visible(False)
def curve(ax,x,y,ysd,color,w,xsd=None):
    ax.errorbar(x,y,yerr=ysd,xerr=xsd,fmt='none',ecolor=color,alpha=.23,capsize=3,lw=1.2,zorder=1)
    ax.plot(x,y,'-o',color=color,lw=2,ms=5,label=f'Wave {w:g}',zorder=3)
for cutoff in [2.,4.,8.]:
    for metric,letter,label in metrics:
        fig,axs=plt.subplots(1,2,figsize=(13.8,6.2))
        fig.subplots_adjust(left=.075,right=.98,bottom=.26,top=.71,wspace=.20)
        fig.suptitle(f'Libo · {cutoff:g} mm cutoff · Aggregation {letter}',y=.965,fontsize=18)
        fig.text(.52,.79,label,ha='center',fontsize=11,color='#49505b')
        left,right=axs
        for ax in axs:style(ax)
        left.set_title('Contact loss vs. terrain roughness',fontsize=12,pad=10)
        right.set_title('Forward displacement vs. contact loss',fontsize=12,pad=10)
        for w,color in zip(waves,colors):
            rr=sorted([p for p in points if p['cutoff_mm']==cutoff and p['wave']==w],key=lambda p:p['height_scale'])
            assert len(rr)==9
            loss=[0]+[p['loss_percent'][metric]['mean'] for p in rr]
            loss_sd=[np.nan]+[p['loss_percent'][metric]['std'] for p in rr]
            forward=[flat[w]['forward_0_6_mm']]+[p['forward_mm']['mean'] for p in rr]
            forward_sd=[np.nan]+[p['forward_mm']['std'] for p in rr]
            rough=[0]+[p['roughness_angle_rms_deg'] for p in rr]
            curve(left,rough,loss,loss_sd,color,w)
            curve(right,loss,forward,forward_sd,color,w,xsd=loss_sd)
            assert all(-2<=x<=102 for x in loss)
            for p in rr:
                l=p['loss_percent'][metric];d=p['forward_mm']
                assert -2<=l['mean']-l['std']<=l['mean']+l['std']<=102
                assert -35<=d['mean']-d['std']<=d['mean']+d['std']<=245
        left.set(xlim=(-.4,rmax+.8),ylim=(-2,102),xlabel='Shared fixed 38-foot / 703-pair angle RMS (deg)',ylabel='Missing expected-support portion (%)')
        left.yaxis.set_major_formatter(PercentFormatter(xmax=100,decimals=0))
        right.set(xlim=(-2,102),ylim=(-35,245),xlabel='Missing expected-support portion (%)',ylabel='Forward COM displacement (mm, 0–6 s)')
        right.xaxis.set_major_formatter(PercentFormatter(xmax=100,decimals=0))
        h,l=left.get_legend_handles_labels()
        fig.legend(h,l,ncol=5,loc='upper center',bbox_to_anchor=(.52,.91),frameon=False)
        fig.text(.075,.13,'0–6 s; identical retained trials. Pale bars: sample SD; horizontal and vertical SD in the right plot.',fontsize=9.5)
        fig.text(.075,.078,'One farthest-from-median complete point omitted per rough group; lines follow increasing height.',fontsize=9.5)
        fig.text(.075,.027,'Same-phase flat reference. Completed-trial means; failures not scored. Fractions are not calibrated erasure p.',fontsize=9.5)
        files=[]
        stem=f'cutoff_{cutoff:g}mm_aggregation_{letter}'
        for suffix in ['.png','.svg']:
            p=OUT/(stem+suffix)
            fig.savefig(p,dpi=170,facecolor='white')
            if suffix=='.png':
                with Image.open(p) as im:im.verify()
            files.append({'path':str(p),'sha256':sha(p)})
        plt.close(fig)
        figures.append({'cutoff_mm':cutoff,'metric':metric,'aggregation':letter,'files':files})
shutil.copyfile(__file__,OUT/Path(__file__).name)
report={'schema':'nine_cutoff_aggregation_contact_loss_v1','created_utc':datetime.now(timezone.utc).isoformat(),
 'source_results_path':str(source),'source_results_sha256':sha(source),'prior_checked_report_path':str(PRIOR),'prior_checked_report_sha256':sha(PRIOR),
 'plot_source_sha256':sha(__file__),'primary_figure_count':9,'panels_per_figure':2,'primary_layout':'3 cutoffs x 3 aggregation choices; left loss vs roughness, right forward displacement vs loss',
 'window_s':[0,6],'retained_rough':3413,'flat_controls':5,'same_preserved_selection':True,'new_physics_steps':0,
 'definitions':{letter:label for metric,letter,label in metrics},'figures':figures,
 'checks':{'all_nine_pairs_present':len(figures)==9,'all_mean_plus_minus_SD_inside_axes':True,'source_unchanged':sha(source)==prior['source_results_sha256'],'all_pngs_decode':True},
 'manual_visual_review_complete':False}
(OUT/'REPORT.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'out':str(OUT),'figures':9,'panels_per_figure':2,'new_physics_steps':0}))
