"""Paired measured-shortwave recovery on the exact September 17 terrain control.
SI units. No gait/PD changes, CDF projection, global renormalization, or optimization.
"""
from pathlib import Path
import os
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']: os.environ[k]='1'
import json,hashlib,time,shutil
import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.interpolate import RegularGridInterpolator
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from terrain_preview import detrend,psd,radial,height_statistics
from workflow import surface_height

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'shortwave'
DATA=Path('X:/Data/Centipede_Terrain_Surface_2026-09-15')
DX=.0005; N=2048; END=2045; ORIGIN=-.511
SITES={'libo':'Libo-9-3-1-3','narvik':'Narvik-4-2-5-3'}
CONTROL_HASH={'libo':'caeede88cd6e4b2681090ae1d3112f75f81157646754642773b0ebd77d09e321','narvik':'72f9f2af6ee9fc6719df05114936ef36941e19c1159dc579e8676611428a5ae7'}
LEVELS=['original','restore4','restore2']
LABELS=['Original (8 mm cutoff)','Restore to 4 mm','Restore to 2 mm']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,d):Path(p).write_text(json.dumps(d,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
def cosine_rise(f,a,b):return .5-.5*np.cos(np.pi*np.clip((f-a)/(b-a),0,1))
def exact_triangle_upsample(h):
    # Four subdivisions along each original triangle edge. Last three rows/columns
    # close the periodic FFT field only; they are excluded from the physical mesh.
    q=np.arange(N)/4; j=np.floor(q).astype(int);u=q-j;k=(j+1)%len(h)
    a=h[np.ix_(j,j)];b=h[np.ix_(j,k)];c=h[np.ix_(k,j)];d=h[np.ix_(k,k)]
    uu=u[None,:];vv=u[:,None]
    return a*(1-np.maximum(uu,vv))+d*np.minimum(uu,vv)+np.where(uu>=vv,b,c)*abs(uu-vv)
def get_target(reference):
    p,fx,fy=psd(reference);smooth=gaussian_filter(np.fft.fftshift(p),.65,mode='constant')
    interp=RegularGridInterpolator((np.fft.fftshift(fy),np.fft.fftshift(fx)),smooth,bounds_error=False,fill_value=0.)
    f=np.fft.fftfreq(N,DX);target=np.empty((N,N))
    for j,ff in enumerate(f):target[j]=interp(np.c_[np.full(N,ff),f])
    neg=(-np.arange(N))%N;target=.5*(target+target[np.ix_(neg,neg)])
    return target,p,f

def make_site(site,raw):
    path=DATA/'open_loop_review_2026-09-16/optimization_surfaces_v1'/f'{SITES[site]}_seed1501_512.npz'
    assert sha(path)==CONTROL_HASH[site]
    with np.load(path) as z:old=z['height'].astype(float);assert float(z['dx_m'])==.002
    ref,plane=detrend(raw);base=exact_triangle_upsample(old);bf=np.fft.fft2(base)
    neg=(-np.arange(N))%N;bf=.5*(bf+np.conj(bf[np.ix_(neg,neg)]))
    target,source_p,f=get_target(ref);r=np.hypot(f[:,None],f[None,:]);low=r<=62.5
    rng=np.random.default_rng(2026092901 if site=='libo' else 2026092902)
    noise=np.fft.fft2(rng.standard_normal((N,N)));noise=.5*(noise+np.conj(noise[np.ix_(neg,neg)]));phase=noise/np.maximum(abs(noise),1e-30);del noise
    bphase=bf/np.maximum(abs(bf),1e-30)
    # Keep existing phase in the old rolloff region. Above 125 cycles/m there
    # is no measured old phase to recover; the paired variants share new phases.
    phase[r<125]=bphase[r<125]
    wanted_amp=np.sqrt(target)*N/DX
    old_axis=(np.arange(512)-255.5)*.002;axis=ORIGIN+np.arange(END)*DX
    testxy=rng.uniform(-.49,.49,(4000,2));exact=surface_height(old,old_axis,old_axis,*testxy.T)
    refined=surface_height(base[:END,:END],axis,axis,*testxy.T)
    triangle_error=float(abs(exact-refined).max());assert triangle_error<2e-15
    np.testing.assert_allclose(base[::4,::4],old,rtol=0,atol=0)
    info=dict(source_control=str(path),source_control_sha256=sha(path),raw_plane_coefficients=plane.tolist(),
              raw_statistics=height_statistics(ref),exact_triangle_refinement_max_error_m=triangle_error,
              source_PSD_method='Plane removed; Hann power-corrected directional PSD; Gaussian sigma 0.65 spectral bins; bilinear PSD interpolation; no height normalization',
              seed=2026092901 if site=='libo' else 2026092902,levels={})
    # Original file remains the exact control, not a resampled replacement.
    info['levels']['original']=dict(path=str(path),sha256=sha(path),grid_spacing_m=.002,shape=list(old.shape),statistics=height_statistics(old,.002))
    for level,cut in [('restore4',250.),('restore2',500.)]:
        w=cosine_rise(r,62.5,125.)*(1-cosine_rise(r,.8*cut,cut))
        # Convex power blend avoids phase cancellation in the crossover. The
        # correction vanishes outside its band, retaining triangle-representation
        # tails from the exact control rather than counting them as measured detail.
        amp=np.sqrt((1-w)*abs(bf)**2+w*wanted_amp**2)
        direction=(1-w)*bphase+w*phase
        direction=np.where(abs(direction)>1e-14,direction/np.maximum(abs(direction),1e-30),phase)
        ff=amp*direction;ff[w==0]=bf[w==0]
        result_complex=np.fft.ifft2(ff);imag=float(abs(result_complex.imag).max());assert imag<1e-14
        result=result_complex.real
        delta_spectrum=np.fft.fft2(result-base)
        frozen_error=float(np.linalg.norm(delta_spectrum[low])/np.linalg.norm(bf[low]));assert frozen_error<1e-12
        h=result[:END,:END].copy()
        output=OUT/f'{site}_{level}.npz'
        np.savez_compressed(output,height=h,dx_m=DX)
        with np.load(output) as saved:np.testing.assert_array_equal(saved['height'],h)
        info['levels'][level]=dict(path=str(output),sha256=sha(output),grid_spacing_m=DX,shape=list(h.shape),
            restoration_frequency_range_cycles_per_m=[62.5,cut],full_replacement_frequency_range_cycles_per_m=[125.,.8*cut],
            correction_rolloff_cycles_per_m=[.8*cut,cut],periodic_long_band_relative_change=frozen_error,
            imaginary_residual_m=imag,statistics=height_statistics(h),periodic_variance_mm2=float(result.var()*1e6),
            added_height_rms_mm=float((result-base).std()*1000),no_global_height_normalization=True,
            mesh_coordinate_span_m=[float(axis[0]),float(axis[-1])])
        print(json.dumps(dict(site=site,level=level,Sq_mm=info['levels'][level]['statistics']['Sq_mm'],long_band_error=frozen_error)),flush=True)
        del result_complex,result,h,ff,amp,direction,delta_spectrum
    # Same-size raw and synthetic windows, including sub-8-mm height differences.
    sr=np.hypot(np.fft.fftfreq(len(ref),DX)[:,None],np.fft.fftfreq(len(ref),DX)[None,:])
    total=float(source_p.sum());info['raw_height_power_fractions']={}
    for a,b in [(1,2),(2,4),(4,8),(8,16),(16,100.5)]:
        info['raw_height_power_fractions'][f'{a:g}-{b:g}mm']=float(source_p[(sr>=1000/b)&(sr<1000/a)].sum()/total)
    info['fidelity_status']='Measured-spectrum constrained paired synthetic detail; short-scale measurement noise and phase topology not validated. No exact height-CDF claim.'
    return info

def load_fields(site,report):
    data=[]
    for level in LEVELS:
        item=report['sites'][site]['levels'][level];p=Path(item['path']);assert sha(p)==item['sha256']
        with np.load(p) as z:h=z['height'].astype(float);dx=float(z['dx_m'])
        ax=(np.arange(h.shape[1])-(h.shape[1]-1)/2)*dx;ay=(np.arange(h.shape[0])-(h.shape[0]-1)/2)*dx
        data.append((h,dx,ax,ay))
    return data

def sample(field,xx,yy):return surface_height(field[0],field[2],field[3],xx,yy)
def render():
    report=json.loads((OUT/'REPORT.json').read_text());raw=np.load(ROOT/'measured_candidates.npz');fixed=np.load(ROOT/'preview/terrains.npz')['fixed_foot_xy_m']
    for site in SITES:
        fields=load_fields(site,report);ref=detrend(raw[site+'_height_m'])[0]
        # One shared location; simulation starts at head/root XY=(120,-120) mm.
        center=np.array([.17,-.12]);a=np.linspace(-.05,.05,201);xx,yy=np.meshgrid(a+center[0],a+center[1]);patches=[sample(v,xx,yy) for v in fields]
        limit=max(float(abs(v).max()) for v in patches)*1000
        fig,axs=plt.subplots(3,3,figsize=(14,11),layout='constrained')
        for c,(title,h) in enumerate(zip(LABELS,patches)):
            im=axs[0,c].imshow(h*1000,origin='lower',extent=[-50,50,-50,50],cmap='terrain',vmin=-limit,vmax=limit,interpolation='nearest')
            axs[0,c].scatter(fixed[:,0]*1000,fixed[:,1]*1000,s=5,c='black');axs[0,c].set(title=title,xlabel='X relative to window center (mm)',ylabel='Y (mm)')
            axs[1,c].plot(a*1000,patches[0][100]*1000,color='gray',lw=1,label='Original')
            axs[1,c].plot(a*1000,h[100]*1000,color='#107d86',lw=1,label=title);axs[1,c].set(xlim=(-50,50),ylim=(-limit,limit),xlabel='X (mm)',ylabel='Height (mm)',title='Same center-line profile');axs[1,c].legend(fontsize=8)
            detail=h[70:131,70:131]-patches[0][70:131,70:131]
            axs[2,c].imshow(detail*1000,origin='lower',extent=[-15,15,-15,15],cmap='RdBu_r',vmin=-.8,vmax=.8,interpolation='nearest');axs[2,c].set(title='Change from original; scale +/-0.8 mm',xlabel='X (mm)',ylabel='Y (mm)')
        fig.colorbar(im,ax=axs[0,:],label='Height (mm)',shrink=.7)
        fig.suptitle(f'{site.title()}: paired short-wavelength recovery | same location and height scale\n100 mm windows at (170, -120) mm; black dots: common neutral XY template',fontsize=14)
        fig.savefig(OUT/f'{site}_comparison.png',dpi=150);plt.close(fig)
        fig=plt.figure(figsize=(15,8.7));fig.subplots_adjust(left=.01,right=.98,bottom=.06,top=.88,wspace=.04,hspace=.2)
        for row,(axis,step,caption) in enumerate([(np.linspace(-.511,.511,257),4.,'Full 1.022 m terrain'),(np.linspace(-.05,.05,201),.5,'100 mm close-up')]):
            xx,yy=np.meshgrid(axis,axis)
            for col,(field,title) in enumerate(zip(fields,LABELS)):
                x=xx if row==0 else xx+center[0];y=yy if row==0 else yy+center[1]
                z=sample(field,x,y)*1000;ax=fig.add_subplot(2,3,row*3+col+1,projection='3d')
                ax.plot_surface(xx*1000,yy*1000,z,cmap='terrain',vmin=-limit,vmax=limit,rcount=len(axis),ccount=len(axis),linewidth=0,antialiased=True)
                ax.set(xlabel='X (mm)',ylabel='Y (mm)',zlabel='Z (mm)',title=title if row==0 else caption)
                ax.set_box_aspect((1,1,.25),zoom=.84);ax.tick_params(labelsize=7)
        fig.suptitle(f'{site.title()}: full surfaces and fine-detail views\nVertical exaggeration for visibility; full-view display about 4 mm, close-up 0.5 mm; collision uses saved mesh.',fontsize=14)
        fig.savefig(OUT/f'{site}_surfaces.png',dpi=140);plt.close(fig)
        centers=[(-.2,-.2),(0.,-.2),(.2,-.2),(-.2,0),(0,0),(.2,0),(-.2,.2),(0,.2),(.2,.2)]
        checks={};fig,axs=plt.subplots(1,3,figsize=(16,4.8),layout='constrained');edges=np.geomspace(9.94,900,26)
        source_p,f,_=psd(ref);rf,rp=radial(source_p,f,f,edges);axs[0].loglog(rf,rp,c='black',label='Raw measured')
        ds=[1,2,4,8,16,32,64];colors=['#6b7280','#16868d','#d57631']
        for d,style in [(1,'-'),(0,'--')]:
            vals=[np.sqrt(np.mean(np.diff(ref,n=1,axis=d)**2))*1000] if False else []
            vals=[np.sqrt(np.mean((np.take(ref,range(round(mm/.5),201),axis=d)-np.take(ref,range(201-round(mm/.5)),axis=d))**2))*1000 for mm in ds]
            axs[1].plot(ds,vals,style,c='black',label='Raw '+('X' if d==1 else 'Y'))
        for field,level,color,label in zip(fields,LEVELS,colors,LABELS):
            windows=[]
            for cx,cy in centers:
                x,y=np.meshgrid(a+cx,a+cy);windows.append(detrend(sample(field,x,y))[0])
            stats=[height_statistics(v) for v in windows];spectra=[psd(v)[0] for v in windows];p=np.mean(spectra,axis=0);_,curve=radial(p,f,f,edges)
            axs[0].loglog(rf,curve,c=color,label=label)
            increments={}
            for d,style in [(1,'-'),(0,'--')]:
                vals=[]
                for mm in ds:
                    k=round(mm/.5);vals.append(float(np.sqrt(np.mean([np.mean((np.take(v,range(k,201),axis=d)-np.take(v,range(201-k),axis=d))**2) for v in windows]))*1000))
                increments['x' if d==1 else 'y']=vals;axs[1].plot(ds,vals,style,c=color,label=label+' '+('X' if d==1 else 'Y'))
            axs[2].scatter([LEVELS.index(level)]*9,[s['rms_gradient'] for s in stats],c=color,s=20)
            checks[level]=dict(window_centers_m=centers,statistics=stats,separation_mm=ds,height_difference_rms_mm=increments)
        for cut in [62.5,125,250,500]:axs[0].axvline(cut,color='gray',lw=.5,alpha=.5)
        axs[0].set(xlabel='Spatial frequency (cycles/m)',ylabel='Directional PSD, radial display (m^4)',title='Matched 100 mm windows; mean of 9');axs[0].legend(fontsize=7)
        axs[1].set(xlabel='Point separation (mm)',ylabel='RMS height difference (mm)',title='Includes the restored short scales');axs[1].legend(fontsize=6,ncol=2)
        axs[2].axhline(height_statistics(ref)['rms_gradient'],c='black',ls='--',label='Raw measured');axs[2].set(xticks=range(3),xticklabels=['Original','4 mm','2 mm'],ylabel='Finite-difference RMS gradient',title='Same-size window spread');axs[2].legend(fontsize=8)
        for ax in axs:ax.grid(alpha=.2)
        fig.suptitle(f'{site.title()}: descriptive checks, not validation of natural morphology or dynamics',fontsize=14)
        fig.savefig(OUT/f'{site}_statistics.png',dpi=150);plt.close(fig)
        report['sites'][site]['matched_window_checks']=checks
        print('Rendered '+site,flush=True)
    report['preview_complete']=True;dump(OUT/'REPORT.json',report)

def generate():
    OUT.mkdir(exist_ok=True)
    if (OUT/'REPORT.json').exists():raise RuntimeError('Completed terrain evidence exists; do not overwrite')
    start=time.time();raw=np.load(ROOT/'measured_candidates.npz')
    report=dict(schema='paired_shortwave_recovery_v1',status='GENERATED_NOT_NATURAL_FIDELITY_VALIDATED',
        source_hashes={str(p):sha(p) for p in [Path(__file__),ROOT/'measured_candidates.npz',ROOT/'PREPARATION.json',ROOT/'terrain_preview.py',ROOT/'workflow.py']},
        method='Exact triangle-refined old mesh; measured directional PSD replaces short-band power with smooth convex-power crossovers and shared phases; all complex coefficients <=62.5 cycles/m held fixed on 1.024 m periodic construction field.',
        transition_note='16-8 mm blends old into measured PSD. Cutoff labels describe the restoration increment, not the tiny high-frequency tails of piecewise-planar old geometry.',
        grid_note='Old mesh remains 512 x 512 at 2 mm; restored 2045 x 2045 at 0.5 mm. Same physical span [-511,511] mm; three periodic closure rows/columns omitted.',
        representation_note='Old physical f(x,y) preserved exactly by triangle subdivision. Long Fourier coefficients frozen before boundary crop; a cropped/windowed spectrum is not claimed identical.',
        no_CDF_projection=True,no_global_Sq_normalization=True,physics_run=False,roughness_sweep=False,sites={})
    for site in SITES:report['sites'][site]=make_site(site,raw[site+'_height_m'])
    report['generation_elapsed_s']=time.time()-start;dump(OUT/'REPORT.json',report)
    shutil.copyfile(__file__,OUT/'generation_source.py');render()

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['generate','render']);a=p.parse_args()
    generate() if a.mode=='generate' else render()

