"""Rearrange the nine checked scientific comparisons into one figure."""
from pathlib import Path
import json,hashlib,os,shutil
from datetime import datetime,timezone
V=Path(__file__).parent
O=V/'contact_loss_overview_2026-10-02'
BASE=Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01')
NINE=BASE/'observed_contact_loss_2026-10-02/nine_cutoff_aggregation/REPORT.json'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
nine=json.loads(NINE.read_text(encoding='utf-8'))
SOURCE=Path(nine['source_results_path']);PRIOR=Path(nine['prior_checked_report_path'])
assert sha(SOURCE)==nine['source_results_sha256']
assert sha(PRIOR)==nine['prior_checked_report_sha256']
prior=json.loads(PRIOR.read_text(encoding='utf-8'))
r=json.loads(SOURCE.read_text(encoding='utf-8'))
points=prior['groups'];flat={c['wave']:c for c in r['flat_controls']}
assert nine['manual_visual_review_complete'] and nine['primary_figure_count']==9
assert len(points)==135
assert not O.exists(),O
O.mkdir()
os.environ['MPLCONFIGDIR']=str(V/'mpl_cache')
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter
from PIL import Image
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12,'svg.fonttype':'none'})
waves=[1.5,2.,2.5,3.,3.5]
colors=['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd']
metrics=[('instantaneous','A','Mean of valid 10 ms fractions'),('cycle','B','Mean of 1 s cycle fractions'),('pooled','C','Whole-window pooled fraction')]
fig=plt.figure(figsize=(32,15.8))
outer=fig.add_gridspec(3,3,left=.035,right=.99,bottom=.10,top=.84,wspace=.16,hspace=.38)
fig.suptitle('Libo: contact loss and forward displacement · all nine comparisons',y=.985,fontsize=24)
rmax=max(p['roughness_angle_rms_deg'] for p in points)
review=[];legend_ax=None
for ir,cutoff in enumerate([2.,4.,8.]):
 for ic,(metric,letter,label) in enumerate(metrics):
  inner=outer[ir,ic].subgridspec(1,2,wspace=.30)
  left=fig.add_subplot(inner[0,0]);right=fig.add_subplot(inner[0,1])
  for ax in [left,right]:
   ax.grid(alpha=.16,zorder=0);ax.axhline(0,color='#777d85',alpha=.35,lw=.8)
   for s in ['top','right']:ax.spines[s].set_visible(False)
  for w,color in zip(waves,colors):
   rr=sorted([p for p in points if p['cutoff_mm']==cutoff and p['wave']==w],key=lambda p:p['height_scale'])
   assert len(rr)==9
   xrough=[0]+[p['roughness_angle_rms_deg'] for p in rr]
   loss=[0]+[p['loss_percent'][metric]['mean'] for p in rr]
   loss_sd=[np.nan]+[p['loss_percent'][metric]['std'] for p in rr]
   fwd=[flat[w]['forward_0_6_mm']]+[p['forward_mm']['mean'] for p in rr]
   fwd_sd=[np.nan]+[p['forward_mm']['std'] for p in rr]
   left.errorbar(xrough,loss,yerr=loss_sd,fmt='none',ecolor=color,alpha=.23,capsize=2.5,lw=1,zorder=1)
   left.plot(xrough,loss,'-o',color=color,lw=2,ms=4.6,label=f'Wave {w:g}',zorder=3)
   right.errorbar(loss,fwd,xerr=loss_sd,yerr=fwd_sd,fmt='none',ecolor=color,alpha=.23,capsize=2.5,lw=1,zorder=1)
   right.plot(loss,fwd,'-o',color=color,lw=2,ms=4.6,zorder=3)
  left.set(xlim=(-.4,rmax+.8),ylim=(-2,102),xticks=[0,5,10,15],yticks=[0,25,50,75,100],xlabel='Terrain roughness (deg)',ylabel='Missing support (%)')
  right.set(xlim=(-2,102),ylim=(-35,245),xticks=[0,25,50,75,100],yticks=[0,50,100,150,200],xlabel='Missing support (%)',ylabel='Forward displacement (mm)')
  left.yaxis.set_major_formatter(PercentFormatter(xmax=100,decimals=0))
  right.xaxis.set_major_formatter(PercentFormatter(xmax=100,decimals=0))
  left.set_title('Loss vs. roughness',fontsize=13,pad=9)
  right.set_title('Displacement vs. loss',fontsize=13,pad=9)
  bounds=outer[ir,ic].get_position(fig)
  fig.text((bounds.x0+bounds.x1)/2,bounds.y1+.028,f'{cutoff:g} mm cutoff · {letter}',ha='center',fontsize=17,fontweight='bold')
  if ir==0:fig.text((bounds.x0+bounds.x1)/2,.903,f'{letter}: {label}',ha='center',fontsize=16,color='#49505b')
  review.append({'cutoff_mm':cutoff,'aggregation':letter,'metric':metric,'same_points_as_prior_nine':True,'panel_count':2})
  if legend_ax is None:legend_ax=left
h,l=legend_ax.get_legend_handles_labels()
fig.legend(h,l,ncol=5,loc='upper center',bbox_to_anchor=(.515,.96),frameon=False,fontsize=17)
fig.text(.035,.059,'0–6 s; same retained trials and flat controls. Pale bars: sample SD; displacement–loss panels show SD on both axes.',fontsize=14)
fig.text(.035,.029,'Roughness: shared fixed 38-foot / 703-pair angle RMS. Lines follow increasing height. Failures not scored; loss is not calibrated erasure p.',fontsize=14)
files=[]
for suffix in ['.png','.svg']:
 p=O/('all_nine_contact_loss'+suffix);fig.savefig(p,dpi=180,facecolor='white')
 if suffix=='.png':
  with Image.open(p) as im:im.verify()
 files.append({'path':str(p),'sha256':sha(p)})
plt.close(fig)
shutil.copyfile(__file__,O/Path(__file__).name)
report={'schema':'nine_contact_loss_combined_layout_v1','created_utc':datetime.now(timezone.utc).isoformat(),
 'source_results_path':str(SOURCE),'source_results_sha256':sha(SOURCE),'prior_nine_report_path':str(NINE),'prior_nine_report_sha256':sha(NINE),
 'prior_checked_report_path':str(PRIOR),'prior_checked_report_sha256':sha(PRIOR),'plot_source_sha256':sha(__file__),
 'layout':'Rows: 2/4/8 mm cutoff. Columns: A/B/C aggregation. Each cell: left loss–roughness, right displacement–loss.',
 'definitions':nine['definitions'],'window_s':[0,6],'shared_retained_rough':3413,'flat_controls':5,'new_physics_steps':0,
 'cells':review,'outputs':files,'checks':{'same_data_and_selection':True,'nine_combinations_18_panels':len(review)==9,'png_decodes':True},'manual_visual_review_complete':False}
(O/'OVERVIEW.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'out':str(O),'combined_images':1,'cutoff_aggregation_cells':9,'panels':18,'new_physics_steps':0}))
