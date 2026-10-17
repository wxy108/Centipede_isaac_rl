"""Approved first terrain preview: measured PSD/CDF -> paired geometric variants.

No physics or foot-roughness sweep. All lengths SI. Fixed 40 IAAFT iterations;
display factors are examples for discussion, not adopted experiment levels.
"""
from pathlib import Path
import os
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:
    os.environ[key]='1'
import json, hashlib, time
import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.interpolate import RegularGridInterpolator
from scipy.signal import resample
from scipy.stats import wasserstein_distance
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'preview'
DX=.0005
MASTER_N=2048
N=1024
ITERATIONS=40
SEED=290929
VARIANTS=['baseline','higher','denser','sparser']
FACTORS={'baseline':(1.,1.),'higher':(1.5,1.),'denser':(1.,.5),'sparser':(1.,2.)}
LABELS={'baseline':'Baseline','higher':'Height x1.5','denser':'Spacing x0.5','sparser':'Spacing x2'}

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')

def detrend(h,dx=DX):
    y,x=np.indices(h.shape,dtype=float);x=(x-x.mean())*dx;y=(y-y.mean())*dx
    a=np.c_[np.ones(h.size),x.ravel(),y.ravel()]
    c=np.linalg.lstsq(a,h.ravel(),rcond=None)[0]
    return h-(a@c).reshape(h.shape),c

def psd(h,dx=DX):
    ny,nx=h.shape;w=np.outer(np.hanning(ny),np.hanning(nx))
    z=(h-(h*w).sum()/w.sum())*w
    p=dx**2/(nx*ny*np.mean(w*w))*abs(np.fft.fft2(z))**2
    fx=np.fft.fftfreq(nx,dx);fy=np.fft.fftfreq(ny,dx)
    np.testing.assert_allclose(p.sum()/(nx*ny*dx**2),np.mean(z*z)/np.mean(w*w),rtol=1e-12)
    return p,fx,fy

def rolloff(f):
    t=np.clip((f-300.)/100.,0,1)
    return .5+.5*np.cos(np.pi*t)

def radial(p,fx,fy,edges):
    freq=np.hypot(fy[:,None],fx[None,:]);values=[]
    for lo,hi in zip(edges[:-1],edges[1:]):
        m=(freq>=lo)&(freq<hi);values.append(float(p[m].mean()) if m.any() else np.nan)
    return np.sqrt(edges[:-1]*edges[1:]),np.array(values)

def height_statistics(h,dx=DX):
    z=h-h.mean();gy,gx=np.gradient(z,dx);cov=np.array([[np.mean(gx*gx),np.mean(gx*gy)],[np.mean(gx*gy),np.mean(gy*gy)]])
    eig,vec=np.linalg.eigh(cov);sq=z.std()
    return dict(Sq_mm=float(sq*1000),skewness=float(np.mean((z/sq)**3)),
                kurtosis=float(np.mean((z/sq)**4)),rms_gradient=float(np.sqrt(np.trace(cov))),
                gradient_anisotropy=float(eig[-1]/eig[0]),gradient_direction_deg=float(np.rad2deg(np.arctan2(vec[1,-1],vec[0,-1]))%180),
                min_mm=float(z.min()*1000),max_mm=float(z.max()*1000))

def increments(h):
    result={}
    for mm in [5,10,20,40,80]:
        k=round(mm/1000/DX)
        result[str(mm)]=dict(x_rms_mm=float(np.sqrt(np.mean((h[:,k:]-h[:,:-k])**2))*1000),
                            y_rms_mm=float(np.sqrt(np.mean((h[k:]-h[:-k])**2))*1000))
    return result

def make_master(reference,seed):
    start=time.monotonic();source_p,fx,fy=psd(reference)
    radius=np.hypot(fy[:,None],fx[None,:]);fmin=1/(len(reference)*DX)
    smooth=gaussian_filter(np.fft.fftshift(source_p),.65,mode='constant')
    interp=RegularGridInterpolator((np.fft.fftshift(fy),np.fft.fftshift(fx)),smooth,bounds_error=False,fill_value=0.)
    f=np.fft.fftfreq(MASTER_N,DX);target=np.empty((MASTER_N,MASTER_N))
    # Row-wise interpolation limits peak temporary memory.
    for j in range(MASTER_N):target[j]=interp(np.c_[np.full(MASTER_N,f[j]),f])
    kr=np.hypot(f[:,None],f[None,:]);target*=rolloff(kr);target[kr<fmin]=0
    neg=(-np.arange(MASTER_N))%MASTER_N;target=.5*(target+target[np.ix_(neg,neg)])
    rng=np.random.default_rng(seed);noise_fft=np.fft.fft2(rng.standard_normal(target.shape))
    phase=noise_fft/np.maximum(abs(noise_fft),1e-30)
    g=np.fft.ifft2(np.sqrt(target)*phase).real;g*=reference.std()/g.std()
    amplitude=abs(np.fft.fft2(g));amplitude[target==0]=0
    del noise_fft,phase
    wanted=np.quantile(reference.ravel(),(np.arange(g.size)+.5)/g.size)
    wanted-=wanted.mean();wanted*=reference.std()/wanted.std()
    trace=[]
    for it in range(ITERATIONS):
        fg=np.fft.fft2(g)
        projected=np.fft.ifft2(amplitude*fg/np.maximum(abs(fg),1e-30)).real
        order=np.argsort(projected.ravel());flat=np.empty(g.size);flat[order]=wanted;g=flat.reshape(g.shape)
        if it%10==9:
            error=float(np.linalg.norm(abs(np.fft.fft2(g))-amplitude)/np.linalg.norm(amplitude))
            trace.append(dict(iteration=it+1,spectrum_relative_error_after_CDF_projection=error))
            print(json.dumps(dict(iteration=it+1,spectral_discrepancy=error,elapsed_s=round(time.monotonic()-start,1))),flush=True)
    fg=np.fft.fft2(g);g=np.fft.ifft2(amplitude*fg/np.maximum(abs(fg),1e-30)).real
    actual=abs(np.fft.fft2(g));spectral_error=float(np.linalg.norm(actual-amplitude)/np.linalg.norm(amplitude))
    forbidden=float(np.sum(actual[target==0]**2)/np.sum(actual**2))
    assert spectral_error<1e-12 and forbidden<1e-25 and np.isfinite(g).all()
    source_lost=float(np.sum(source_p*(1-rolloff(radius)))/source_p.sum())
    slope_power=source_p*radius**2
    slope_lost=float(np.sum(slope_power*(1-rolloff(radius)))/slope_power.sum())
    # Quantile distance equals one-dimensional Wasserstein distance on equal quantiles.
    cdf_distance=float(np.mean(abs(np.sort(g.ravel())-wanted))*1000)
    report=dict(seed=seed,iterations=ITERATIONS,final_projection='Fourier amplitude; CDF approximate',trace=trace,
                master_grid=[MASTER_N,MASTER_N],master_period_m=MASTER_N*DX,
                source_frequency_min_cycles_per_m=fmin,high_frequency_rolloff_cycles_per_m=[300,400],
                source_height_power_fraction_removed_by_high_cut=source_lost,
                source_spectral_gradient_power_fraction_removed_by_high_cut=slope_lost,
                spectral_amplitude_relative_error=spectral_error,forbidden_band_power_fraction=forbidden,
                master_height_CDF_Wasserstein_mm=cdf_distance,
                master_height_CDF_Wasserstein_over_Sq=cdf_distance/(reference.std()*1000),
                master_statistics=height_statistics(g),master_sha256_float64=hashlib.sha256(g.tobytes()).hexdigest(),
                elapsed_s=time.monotonic()-start)
    return g,source_p,report

def variants(master,reference):
    center=MASTER_N//2;half=N//2
    raw={'baseline':master[center-half:center+half,center-half:center+half].copy(),
         'denser':master[::2,::2].copy()}
    # Exact periodic Fourier resampling; no bilinear attenuation in the sparse case.
    up=resample(resample(master,MASTER_N*2,axis=0),MASTER_N*2,axis=1)
    center=MASTER_N
    raw['sparser']=up[center-half:center+half,center-half:center+half].copy();del up
    output={};meta={}
    for key,h in raw.items():
        mean=float(h.mean());factor=float(reference.std()/h.std())
        output[key]=(h-mean)*factor
        meta[key]=dict(horizontal_scale=FACTORS[key][1],height_factor=1.,crop_mean_removed_m=mean,Sq_renormalization=factor)
    output['higher']=output['baseline']*1.5
    meta['higher']=dict(meta['baseline'],height_factor=1.5)
    for key in VARIANTS:
        h=output[key];np.testing.assert_allclose(h.std(),reference.std()*FACTORS[key][0],rtol=1e-12)
        assert h.shape==(N,N) and np.isfinite(h).all()
        meta[key].update(statistics=height_statistics(h),increment_statistics=increments(h))
    np.testing.assert_array_equal(output['higher'],output['baseline']*1.5)
    return output,meta

def same_size_check(reference,baseline):
    n=len(reference);starts=[48,411,774];patches=[]
    for y in starts:
        for x in starts:patches.append(detrend(baseline[y:y+n,x:x+n])[0])
    spectra=np.array([psd(h)[0] for h in patches]);p,fx,fy=psd(reference)
    per=[height_statistics(h) for h in patches];ref_stats=height_statistics(reference)
    band_edges=[2.5,5,10,20,50,100.5];freq=np.hypot(fy[:,None],fx[None,:]);bands={}
    df=1/(n*DX)**2
    for lo,hi in zip(band_edges[:-1],band_edges[1:]):
        mask=(freq>=1000/hi)&(freq<1000/lo)
        orig=float(p[mask].sum()*df*1e6);vals=spectra[:,mask].sum(1)*df*1e6
        bands[f'{lo:g}-{hi:g}mm']=dict(measured_mm2=orig,generated_window_mean_mm2=float(vals.mean()),generated_window_min_mm2=float(vals.min()),generated_window_max_mm2=float(vals.max()))
    keys=['Sq_mm','rms_gradient','gradient_anisotropy','skewness','kurtosis']
    ranges={k:dict(measured=ref_stats[k],generated_min=min(r[k] for r in per),generated_mean=float(np.mean([r[k] for r in per])),generated_max=max(r[k] for r in per)) for k in keys}
    report=dict(window_vertex_span_mm=100.,window_count=9,window_indices=starts,all_windows_plane_removed=True,
                note='Correlated descriptive windows, not independent validation samples; no automatic fidelity acceptance threshold.',
                statistics=ranges,band_power=bands,measured_increment_statistics=increments(reference),
                generated_window_increment_statistics=[increments(h) for h in patches])
    return report,spectra,patches

def figures(name,reference,fields,source_p,spectra,patches,foot_xy):
    titles=['Measured / plane removed']+[LABELS[k] for k in VARIANTS]
    cropped=[reference]+[fields[k][N//2-100:N//2+101,N//2-100:N//2+101] for k in VARIANTS]
    maxheight=max(float(np.max(abs(h))) for h in cropped)*1000
    fig,axs=plt.subplots(2,5,figsize=(18,7.6),layout='constrained')
    for col,(title,h) in enumerate(zip(titles,cropped)):
        im=axs[0,col].imshow(h*1000,origin='lower',extent=[-50,50,-50,50],cmap='terrain',vmin=-maxheight,vmax=maxheight)
        axs[0,col].scatter(foot_xy[:,0]*1000,foot_xy[:,1]*1000,s=5,color='black',alpha=.8)
        axs[0,col].set(title=title,xlabel='X (mm)',ylabel='Y (mm)')
        axs[1,col].plot(np.linspace(-50,50,len(h)),h[len(h)//2]*1000,color='#186b7a',lw=1.1)
        axs[1,col].axhline(0,color='gray',lw=.5);axs[1,col].set(xlim=(-50,50),ylim=(-maxheight,maxheight),xlabel='X (mm)',ylabel='Height (mm)',title='Center profile: Y = 0');axs[1,col].grid(alpha=.2)
    fig.colorbar(im,ax=axs[0,:],label='Height (mm)',shrink=.8)
    fig.suptitle(f'{name.title()}: same 100 x 100 mm viewing window and shared height scale\nBlack dots: the same neutral 38-foot XY template; generated factors are preview examples',fontsize=15)
    fig.savefig(OUT/f'{name}_comparison.png',dpi=155);plt.close(fig)
    full_limit=max(float(np.max(abs(fields[k]))) for k in VARIANTS)*1000
    fig=plt.figure(figsize=(12,9))
    fig.subplots_adjust(left=.02,right=.91,bottom=.04,top=.88,wspace=.06,hspace=.08)
    for col,key in enumerate(VARIANTS):
        ax=fig.add_subplot(2,2,col+1,projection='3d');h=fields[key];xy=(np.arange(N)-N/2)*DX*1000
        # Display decimation is distinct from saved 0.5 mm terrain geometry.
        ix=np.arange(0,N,8);xx,yy=np.meshgrid(xy[ix],xy[ix]);hh=h[np.ix_(ix,ix)]*1000
        ax.plot_surface(xx,yy,hh,cmap='terrain',vmin=-full_limit,vmax=full_limit,rcount=len(ix),ccount=len(ix),linewidth=0,rasterized=True)
        ax.set(xlabel='X (mm)',ylabel='Y (mm)',zlabel='Height (mm)',title=LABELS[key],zlim=(-full_limit,full_limit))
        ax.set_box_aspect((1,1,.3),zoom=.85);ax.set_zticks([-round(full_limit,1),0,round(full_limit,1)])
        ax.zaxis.set_tick_params(pad=1)
    fig.suptitle(f'{name.title()}: 511.5 mm terrain span; stored grid 0.5 mm\n3D view is vertically exaggerated; display spacing 4 mm. Use the 100 mm close-up for fine detail.',fontsize=13)
    fig.savefig(OUT/f'{name}_full.png',dpi=145);plt.close(fig)
    fig,axs=plt.subplots(1,3,figsize=(15,4.5),layout='constrained')
    f=np.fft.fftfreq(len(reference),DX);edges=np.geomspace(9.94,990,25)
    rf,rp=radial(source_p,f,f,edges);axs[0].loglog(rf,rp,label='Measured 100 mm',color='black',lw=2)
    ys=np.array([radial(p,f,f,edges)[1] for p in spectra]);valid=np.isfinite(ys).all(0)
    axs[0].fill_between(rf[valid],ys[:,valid].min(0),ys[:,valid].max(0),alpha=.2,color='#137b80',label='9 generated windows: range')
    axs[0].loglog(rf[valid],ys[:,valid].mean(0),label='Window mean',color='#137b80');axs[0].axvspan(300,400,color='orange',alpha=.2,label='Common roll-off')
    axs[0].set(xlabel='Spatial frequency (cycles/m)',ylabel='2D PSD (m^4)',title='Same-size window PSD (radial display only)');axs[0].legend(fontsize=8)
    qs=np.linspace(.001,.999,999)
    axs[1].plot(np.quantile(reference,qs)*1000,qs,color='black',lw=2,label='Measured')
    axs[1].plot(np.quantile(fields['baseline'],qs)*1000,qs,color='#137b80',label='Full baseline')
    for h in patches:axs[1].plot(np.quantile(h,qs)*1000,qs,color='#137b80',alpha=.15,lw=.6)
    axs[1].set(xlabel='Height (mm)',ylabel='Cumulative probability',title='Height distributions');axs[1].legend(fontsize=8)
    source_inc=increments(reference);colors=['#137b80','#d67b2c','#4b63b3','#9752a0']
    distances=np.array([5,10,20,40,80])
    for direction,style in [('x','-'),('y','--')]:
        axs[2].plot(distances,[source_inc[str(v)][direction+'_rms_mm'] for v in distances],style,color='black',lw=2,label='Measured '+direction.upper())
        for key,color in zip(VARIANTS,colors):
            inc=increments(fields[key]);axs[2].plot(distances,[inc[str(v)][direction+'_rms_mm'] for v in distances],style,color=color,lw=1,label=LABELS[key]+' '+direction.upper())
    axs[2].set(xlabel='Point separation (mm)',ylabel='RMS height difference (mm)',title='Height differences at foot-pair scales');axs[2].legend(fontsize=6,ncol=2)
    for ax in axs:ax.grid(alpha=.2)
    fig.suptitle(name.title()+': descriptive fidelity checks; matching PSD alone is not full morphological validation',fontsize=13)
    fig.savefig(OUT/f'{name}_statistics.png',dpi=150);plt.close(fig)

def phase_preview():
    with np.load(ROOT/'current_gait_geometry.npz') as z:
        masks=z['historical_nominal_stance'];waves=z['waves']
    fig,axs=plt.subplots(6,1,figsize=(11,8),sharex=True,layout='constrained')
    from matplotlib.colors import ListedColormap
    for ax,mask,wave in zip(axs,masks,waves):
        ax.imshow(mask.T,origin='lower',aspect='auto',extent=[0,1,0,38],cmap=ListedColormap(['#d87839','#197d88']),vmin=0,vmax=1,interpolation='nearest')
        ax.axhline(19,color='white',lw=.7);ax.set(yticks=[9.5,28.5],yticklabels=['Left','Right'],ylabel=f'k={wave:g}')
    axs[-1].set_xlabel('Phase of the existing 1 s command cycle')
    fig.suptitle('Historical nominal phase windows - for review, not a new adopted contact rule\nTeal: pitch >= 0 (stance); orange: pitch < 0 (swing). Each sample has 19 + 19 feet.',fontsize=13)
    fig.savefig(OUT/'phase_windows.png',dpi=145);plt.close(fig)

def main():
    OUT.mkdir(exist_ok=True)
    if (OUT/'REPORT.json').exists():raise RuntimeError('Preview already complete; preserve existing evidence before a new generation')
    start=time.monotonic();z=np.load(ROOT/'measured_candidates.npz');geom=np.load(ROOT/'geometry_candidates.npz')
    foot_xy=geom['feet_m'][0,:,:2]
    arrays={'axis_m':(np.arange(N)-N/2)*DX,'dx_m':DX,'fixed_foot_xy_m':foot_xy}
    report=dict(status='GEOMETRY_PREVIEW_ONLY_NOT_VALIDATED_AS_HIGH_FIDELITY',physics_run=False,roughness_sweep=False,
                factors_are_illustrations_not_adopted_levels=True,display_factors=FACTORS,
                coordinate_units='metres',grid_spacing_m=DX,terrain_vertex_span_m=(N-1)*DX,
                master_period_m=MASTER_N*DX,source_sites_not_mixed=True,plane_removed=True,
                method='Windowed directional 2D measured PSD, smoothed/interpolated within supported band; 40 IAAFT projections; final exact spectrum. Same master field transformed for variants; each non-height variant normalized to measured Sq.',
                supported_master_wavelength_mm=[2.5,100.5],rolloff_master_wavelength_mm=[2.5,1000/300],
                scale_transform_note='Spacing x2 explicitly stretches measured components up to 201 mm; these are transformed wavelengths, not new measurements. No independent unmeasured low-frequency components added.',
                no_claims=['Global PSD does not preserve phase topology','0.5 mm grid does not establish measurement accuracy','Single specimen per family; nine cropped windows are correlated','Final spectrum exact; height CDF approximate','Same-size window and slope discrepancies must be reported'],
                families={},sources={str(p):sha(p) for p in [Path(__file__),ROOT/'measured_candidates.npz',ROOT/'geometry_candidates.npz',ROOT/'current_gait_geometry.npz',ROOT/'PREPARATION.json']})
    for name in ['libo','narvik']:
        print('Starting '+name,flush=True)
        reference,plane=detrend(z[name+'_height_m']);arrays[name+'_reference_m']=reference
        master,source_p,generation=make_master(reference,SEED)
        fields,transforms=variants(master,reference);del master
        check,spectra,patches=same_size_check(reference,fields['baseline'])
        for key,h in fields.items():arrays[name+'_'+key+'_m']=h.astype(np.float32)
        report['families'][name]=dict(reference_statistics=height_statistics(reference),removed_plane_coefficients=plane.tolist(),generation=generation,variants=transforms,same_size_comparison=check)
        figures(name,reference,fields,source_p,spectra,patches,foot_xy)
    np.savez_compressed(OUT/'terrains.npz',**arrays)
    # Verify serialized terrains, rather than reporting only in-memory checks.
    with np.load(OUT/'terrains.npz') as saved:
        for name in ['libo','narvik']:
            ref=saved[name+'_reference_m'];baseline=saved[name+'_baseline_m'].astype(float)
            for key in VARIANTS:
                h=saved[name+'_'+key+'_m'].astype(float)
                assert np.isfinite(h).all() and h.shape==(N,N)
                np.testing.assert_allclose(h.std(),ref.std()*FACTORS[key][0],rtol=1e-7)
            np.testing.assert_allclose(saved[name+'_higher_m'],baseline*1.5,rtol=1e-6,atol=1e-9)
    phase_preview()
    report['artifact_sha256']=sha(OUT/'terrains.npz');report['elapsed_s']=time.monotonic()-start
    report['engineering_checks']=dict(finite=True,grid_shapes=True,PSD_parseval=True,spectral_support=True,height_factor=True,matched_Sq_for_horizontal_variants=True,serialized_readback=True)
    dump(OUT/'REPORT.json',report)
    print(json.dumps({'status':report['status'],'elapsed_s':report['elapsed_s'],'checks':report['engineering_checks']}),flush=True)

def render_saved():
    """Refresh plots from serialized geometry without regenerating terrain."""
    with np.load(OUT/'terrains.npz') as z:
        for name in ['libo','narvik']:
            reference=z[name+'_reference_m']
            fields={key:z[name+'_'+key+'_m'].astype(float) for key in VARIANTS}
            _,spectra,patches=same_size_check(reference,fields['baseline'])
            figures(name,reference,fields,psd(reference)[0],spectra,patches,z['fixed_foot_xy_m'])
    phase_preview()
    print('Rendered saved terrains; no regeneration or sweep.',flush=True)

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['generate','render'],nargs='?',default='generate')
    args=parser.parse_args()
    main() if args.action=='generate' else render_saved()
