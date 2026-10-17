"""Fixed 4 mm nominal cutoff: terrain-height previews, no robot simulation."""
from pathlib import Path
import argparse
import os
import json
import hashlib
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:
    os.environ[key]='1'
import numpy as np
from scipy.stats import qmc
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from PIL import Image
from workflow import surface_height, checks

ROOT=Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
SW=ROOT/'shortwave'
OUT=ROOT/'height_amplitude_4mm'
METRICS=SW/'displacement_roughness/RESULTS.json'
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,d):Path(p).write_text(json.dumps(d,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')


def combine():
    out=SW/'displacement_roughness/displacement_vs_wave_number_combined.png'
    if out.exists():raise RuntimeError(f'Output exists: {out}')
    rows=read(METRICS)['cells'];lookup={(r['site'],r['level'],r['wave']):r for r in rows}
    waves=[1.5,2,2.5,3,3.5,4]
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(1,2,figsize=(13.2,6.5),sharey=True)
    fig.subplots_adjust(left=.075,right=.98,bottom=.19,top=.69,wspace=.14)
    fig.suptitle('Shortwave-band side finding: forward displacement vs. wave number',fontsize=17,weight='bold',y=.965)
    for ax,site in zip(axes,['libo','narvik']):
        for level,label,color,marker in zip(['original','restore4','restore2'],
                ['Original: cutoff 8 mm, grid 2 mm','Cutoff 4 mm, grid 0.5 mm','Cutoff 2 mm, grid 0.5 mm'],
                ['#536b80','#d48924','#247c98'],['o','s','^']):
            yy=[lookup[(site,level,w)]['forward_0_6_mm'] for w in waves]
            ax.plot(waves,yy,lw=2.3,color=color,marker=marker,markersize=7,label=label,markeredgecolor='white',markeredgewidth=.8)
        ax.set(title=site.capitalize(),xlabel='Wave number',xlim=(1.4,4.1),ylim=(0,240))
        ax.set_xticks(waves,[f'{w:g}' for w in waves]);ax.set_yticks(range(0,241,40));ax.grid(alpha=.18)
    axes[0].set_ylabel('Forward displacement, 0-6 s (mm)')
    fig.legend(*axes[0].get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.53,.90),frameon=False,fontsize=10.5,ncol=1)
    fig.text(.075,.082,'Saved whole-body COM displacement along the initial forward axis (-X); one 6 s run per condition.',fontsize=10,color='#536170')
    fig.text(.075,.041,'Flat-cluster gait amplitudes and Sep 17 wave-4 PD unchanged. This is the existing shortwave comparison.',fontsize=10,color='#536170')
    fig.savefig(out,dpi=160,facecolor='white');plt.close(fig)
    with Image.open(out) as im:im.load()
    dump(out.with_suffix('.json'),dict(path=str(out),sha256=sha(out),source_metrics=str(METRICS),source_metrics_sha256=sha(METRICS),
                                     source_code=str(Path(__file__).resolve()),source_code_sha256=sha(__file__),data_replotted=True,new_physics=False))
    print(str(out),flush=True)


def sampler():
    method=read(METRICS)['fixed_roughness_sampling']
    with np.load(method['foot_template_path']) as z:foot=z[method['template_key']].astype(float)
    domain=np.array(method['common_center_domain_xy_m']);roi=np.array(method['physics_roi_xy_m'])
    u=qmc.Sobol(3,scramble=True,seed=method['sobol_seed']).random_base2(method['sobol_power'])
    centers=domain[:,0]+u[:,:2]*np.diff(domain,axis=1).ravel();theta=2*np.pi*u[:,2]
    c=np.cos(theta)[:,None];s=np.sin(theta)[:,None]
    points=np.stack([c*foot[:,0]-s*foot[:,1],s*foot[:,0]+c*foot[:,1]],-1)+centers[:,None]
    assert np.all((points>=roi[:,0])&(points<=roi[:,1]))
    i,j=np.triu_indices(len(foot),1);dist=np.linalg.norm(foot[i]-foot[j],axis=-1)
    assert len(dist)==703 and dist.min()>0
    return points,i,j,dist,method,foot


def roughness(h,axis,points,i,j,dist):
    heights=surface_height(h.astype(np.float32).astype(float),axis,axis,points[...,0],points[...,1])
    total=0.
    for start in range(0,len(points),2048):
        hh=heights[start:start+2048]
        beta=np.arctan2(abs(hh[:,i]-hh[:,j]),dist)
        total+=float((beta*beta).mean(-1).sum())
    return float(np.degrees(np.sqrt(total/len(points))))


def site_figure(site,rows,patches,a):
    # One location, common camera, common z limits and color range for all seven
    # panels. This makes height amplitude visible without changing XY detail.
    limit=max(float(abs(p).max()) for p in patches)*1000
    limit=float(np.ceil(limit*2)/2)
    norm=Normalize(-limit,limit);cmap='terrain'
    xx,yy=np.meshgrid(a*1000,a*1000)
    ncols=4;nrows=int(np.ceil((len(rows)+1)/ncols))
    fig=plt.figure(figsize=(19.2,9.8))
    fig.subplots_adjust(left=.018,right=.93,bottom=.075,top=.855,wspace=.07,hspace=.23)
    colors=plt.cm.viridis(np.linspace(.12,.86,len(rows)))
    for idx,(row,patch) in enumerate(zip(rows,patches)):
        ax=fig.add_subplot(nrows,ncols,idx+1,projection='3d')
        ax.plot_surface(xx,yy,patch*1000,cmap=cmap,norm=norm,rcount=201,ccount=201,linewidth=0,antialiased=True)
        ax.view_init(elev=29,azim=-125)
        ax.set(xlim=(-50,50),ylim=(-50,50),zlim=(-limit,limit),xlabel='X (mm)',ylabel='Y (mm)',zlabel='Height (mm)')
        ax.set_box_aspect((100,100,2*limit*3),zoom=.86)
        ax.set_title(f"Height x{row['height_scale']:g} | R = {row['fixed_pair_angle_rms_deg']:.2f} deg",fontsize=12,pad=1)
        ax.tick_params(labelsize=8,pad=0)
    ax=fig.add_subplot(nrows,ncols,len(rows)+1)
    for row,patch,color in zip(rows,patches,colors):
        ax.plot(a*1000,patch[100]*1000,lw=1.8,color=color,label=f"x{row['height_scale']:g}")
    ax.set(xlabel='X (mm)',ylabel='Height (mm)',xlim=(-50,50),ylim=(-limit,limit),title='Same center-line profile')
    ax.grid(alpha=.2);ax.legend(ncol=3,fontsize=9,frameon=False)
    cax=fig.add_axes([.947,.22,.015,.49]);fig.colorbar(ScalarMappable(norm=norm,cmap=cmap),cax=cax,label='Height (mm)')
    fig.suptitle(f'{site.capitalize()}: terrain height amplitude at fixed 4 mm cutoff',fontsize=18,weight='bold',y=.97)
    fig.text(.5,.916,'Same 100 x 100 mm window at (170, -120) mm | grid 0.5 mm | same phases and XY layout',ha='center',fontsize=11,color='#42586b')
    fig.text(.03,.024,'Height and color scales fixed within this figure; Z display exaggerated 3x. R uses the full shared simulation ROI, not this window.',fontsize=10,color='#52616f')
    path=OUT/f'{site}_height_amplitude_preview.png';fig.savefig(path,dpi=140,facecolor='white');plt.close(fig)
    with Image.open(path) as im:im.load()
    return dict(path=str(path),sha256=sha(path),window_center_m=[.17,-.12],window_width_m=.1,display_grid_m=.0005,
                z_limits_mm=[-limit,limit],vertical_display_exaggeration=3,source_scale_order=[r['height_scale'] for r in rows])


def summary_figure(records):
    fig,axes=plt.subplots(1,2,figsize=(11.5,4.9),layout='constrained')
    for site,color in [('libo','#237f98'),('narvik','#c37d24')]:
        rr=[r for r in records if r['site']==site];x=[r['height_scale'] for r in rr]
        axes[0].plot(x,[r['fixed_pair_angle_rms_deg'] for r in rr],'-o',lw=2,color=color,label=site.capitalize())
        axes[1].plot(x,[r['height_Sq_mm'] for r in rr],'-o',lw=2,color=color,label=site.capitalize())
    for ax in axes:
        ax.set_xlabel('Terrain height multiplier');ax.grid(alpha=.2);ax.legend(frameon=False)
        ax.set_xticks(sorted(set(r['height_scale'] for r in records)))
    axes[0].set_ylabel('Fixed-foot pair-angle RMS (deg)');axes[0].set_title('Roughness from 38 neutral feet / 703 pairs')
    axes[1].set_ylabel('Height RMS about mean, Sq (mm)');axes[1].set_title('Full saved terrain height amplitude')
    fig.suptitle('Fixed 4 mm cutoff | shared positions and all headings | no robot simulation',fontsize=13)
    path=OUT/'roughness_vs_height_amplitude.png';fig.savefig(path,dpi=160,facecolor='white');plt.close(fig)
    with Image.open(path) as im:im.load()
    return dict(path=str(path),sha256=sha(path))


def generate(scales):
    if OUT.exists():raise RuntimeError(f'Existing output must not be overwritten: {OUT}')
    assert 2<=len(scales)<=7 and scales==sorted(set(scales)) and min(scales)>0 and 1.0 in scales
    OUT.mkdir()
    points,i,j,dist,method,foot=sampler()
    construction=read(SW/'REPORT.json');records=[];figures=[]
    source_paths=[Path(__file__),ROOT/'workflow.py',METRICS,SW/'REPORT.json',Path(method['foot_template_path'])]
    sources={str(p):sha(p) for p in source_paths};checks_done=checks()
    a=np.linspace(-.05,.05,201);xx,yy=np.meshgrid(a+.17,a-.12)
    for site in ['libo','narvik']:
        ref=construction['sites'][site]['levels']['restore4'];path=Path(ref['path']);sources[str(path)]=sha(path)
        assert sources[str(path)]==ref['sha256']
        with np.load(path) as z:h0=z['height'].astype(float);dx=float(z['dx_m'])
        assert h0.shape==(2045,2045) and dx==.0005 and np.isfinite(h0).all()
        mean=float(h0.mean());centered=h0-mean;sd0=float(centered.std())
        ax=(np.arange(len(h0))-(len(h0)-1)/2)*dx;collision_axis=ax.astype(np.float32).astype(float)
        patches=[];rows=[]
        for scale in scales:
            h=h0.copy() if scale==1 else mean+scale*centered
            if scale==1:
                dst=path
            else:
                tag=f'{scale:.2f}'.replace('.','p');dst=OUT/f'{site}_height_x{tag}.npz'
                np.savez_compressed(dst,height=h,dx_m=dx)
                with np.load(dst) as z:np.testing.assert_array_equal(z['height'],h);assert float(z['dx_m'])==dx
            np.testing.assert_allclose(h.mean(),mean,atol=1e-16,rtol=0)
            np.testing.assert_allclose(h.std(),scale*sd0,atol=1e-15,rtol=1e-13)
            r=roughness(h,collision_axis,points,i,j,dist)
            if scale==1:
                old=next(rr['fixed_pair_angle_rms_deg'] for rr in read(METRICS)['roughness'] if rr['site']==site and rr['level']=='restore4')
                assert abs(r-old)<1e-10
            row=dict(site=site,height_scale=scale,path=str(dst),sha256=sha(dst),baseline_reused=scale==1,
                     source_4mm_path=str(path),source_4mm_sha256=ref['sha256'],shape=list(h.shape),grid_spacing_m=dx,
                     physical_span_m=[float(ax[0]),float(ax[-1])],mean_height_mm=float(h.mean()*1000),height_Sq_mm=float(h.std()*1000),
                     min_height_mm=float(h.min()*1000),max_height_mm=float(h.max()*1000),fixed_pair_angle_rms_deg=r)
            rows.append(row);records.append(row);patches.append(surface_height(h,ax,ax,xx,yy))
            print(f'{site} height x{scale:g}: R={r:.6f} deg, Sq={row["height_Sq_mm"]:.6f} mm',flush=True)
        assert np.all(np.diff([r['fixed_pair_angle_rms_deg'] for r in rows])>0)
        figures.append(site_figure(site,rows,patches,a))
    figures.append(summary_figure(records))
    assert all(sha(p)==v for p,v in sources.items())
    report=dict(complete=True,height_scales=scales,scale_selection_note='Union of the two proposed ranges after user asked to see more groups',cutoff_wavelength_nominal_mm=4,grid_spacing_m=.5/1000,
                height_transform='h_s = mean(h_0) + s * (h_0 - mean(h_0)); s=1 references original file exactly',
                preserved='XY layout, coordinate span, grid, phases and normalized spectral shape; non-DC PSD scales by s^2',
                source_note='Uses existing restore4 sample unchanged as mother surface; original triangle-representation high-frequency tails retained, no new filtering.',
                roughness_sampling=method,terrains=records,figures=figures,input_hashes=sources,
                checks=dict(**checks_done,height_readback_exact=True,mean_height_preserved=True,standard_deviation_scales_exactly=True,
                            baseline_R_matches_previous_analysis=True,roughness_increases_with_positive_scale=True,inputs_unchanged=True,images_decoded=True),
                previews_only=True,new_physics_steps=0,robot_amplitudes_PD_unchanged=True,per_gait_roughness_not_computed=True)
    dump(OUT/'REPORT.json',report)
    print('Completed 4 mm fixed-cutoff height previews; no physics run.',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['combine','generate']);p.add_argument('--scales',nargs='+',type=float)
    args=p.parse_args()
    if args.mode=='combine':combine()
    else:
        if args.scales is None:p.error('generate requires explicit --scales')
        generate(args.scales)
