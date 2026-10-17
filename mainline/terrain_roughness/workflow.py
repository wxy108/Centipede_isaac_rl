"""Prepare measured terrain/pose candidates; never selects a terrain or runs physics."""
from pathlib import Path
import json, hashlib, importlib.util, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
MAIN = HERE.parent
ARCH = Path('X:/Projects/_archive/Codex')
CON = ARCH/'2026-09-11/con'
OLD = ARCH/'2026-09-17/w-m'
MODEL = CON/'round9_wave_amplitude_2026-09-14'
DATA = Path('X:/Data/Centipede_Terrain_Surface_2026-09-15')
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
read = lambda p: json.loads(Path(p).read_text(encoding='utf-8-sig'))

def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    obj = importlib.util.module_from_spec(spec); spec.loader.exec_module(obj)
    return obj

def pair_angle_m2(xy, heights):
    """Equal unordered pairs, radians squared; average frames BEFORE sqrt.
    Foot selection is explicit. Coincident projections are never silently dropped.
    """
    xy, heights = np.asarray(xy, float), np.asarray(heights, float)
    if xy.shape[:-1] != heights.shape or xy.shape[-1] != 2 or xy.shape[-2] < 2:
        raise ValueError('Need matching XY and heights for at least two feet')
    if not (np.isfinite(xy).all() and np.isfinite(heights).all()):
        raise ValueError('Non-finite input')
    i, j = np.triu_indices(xy.shape[-2], 1)
    d = np.linalg.norm(xy[...,i,:] - xy[...,j,:], axis=-1)
    if np.any(d <= 0): raise ValueError('Coincident foot projections')
    return np.mean(np.arctan2(abs(heights[...,i]-heights[...,j]), d)**2, axis=-1)

def surface_height(h, ax, ay, x, y):
    """Triangle diagonal (x0,y0)-(x1,y1); reject extrapolation/wrapping."""
    h, ax, ay = np.asarray(h,float), np.asarray(ax), np.asarray(ay)
    x,y = np.broadcast_arrays(np.asarray(x,float),np.asarray(y,float))
    if h.shape != (len(ay),len(ax)) or not np.isfinite(h).all(): raise ValueError('Invalid grid')
    if not (np.all(np.diff(ax)>0) and np.all(np.diff(ay)>0)): raise ValueError('Axes must increase')
    if not (np.isfinite(x).all() and np.isfinite(y).all()): raise ValueError('Non-finite position')
    if np.any((x<ax[0])|(x>ax[-1])|(y<ay[0])|(y>ay[-1])): raise ValueError('Foot outside surface')
    i=np.clip(np.searchsorted(ax,x,side='right')-1,0,len(ax)-2)
    j=np.clip(np.searchsorted(ay,y,side='right')-1,0,len(ay)-2)
    u=(x-ax[i])/(ax[i+1]-ax[i]); v=(y-ay[j])/(ay[j+1]-ay[j])
    return h[j,i]*(1-np.maximum(u,v))+h[j+1,i+1]*np.minimum(u,v)+np.where(u>=v,h[j,i+1],h[j+1,i])*abs(u-v)

def detrend(h,x,y):
    xx,yy=np.meshgrid(x,y); a=np.column_stack([np.ones(h.size),xx.ravel(),yy.ravel()])
    c=np.linalg.lstsq(a,h.ravel(),rcond=None)[0]
    return h-(a@c).reshape(h.shape),c

def checks():
    xy=np.array([[0.,0.],[1.,0.],[0.,1.],[1.,1.]])
    assert pair_angle_m2(xy,np.zeros(4))==0
    h=.2*xy[:,0]-.3*xy[:,1]
    expected=np.mean([np.arctan(abs(h[i]-h[j])/np.linalg.norm(xy[i]-xy[j]))**2 for i in range(4) for j in range(i)])
    np.testing.assert_allclose(pair_angle_m2(xy,h),expected,rtol=1e-14)
    np.testing.assert_allclose(pair_angle_m2(xy[::-1],h[::-1]),expected)
    np.testing.assert_allclose(pair_angle_m2(xy,[0,1,1,0]),4*(np.pi/4)**2/6)
    grid=np.array([[0,.2],[-.3,-.1]])
    np.testing.assert_allclose(surface_height(grid,[0,1],[0,1],[.2,.8,1],[.8,.2,1]),[-.2,.1,-.1],atol=1e-15)
    try: surface_height(grid,[0,1],[0,1],-.01,.5)
    except ValueError: pass
    else: raise AssertionError('Extrapolation accepted')
    return dict(flat_zero=True,analytic_plane=True,pair_permutation=True,checkerboard=True,triangle_and_boundary=True,no_extrapolation=True)

def prepare():
    inputs=[Path(__file__),MAIN/'campaign.json',MAIN/'rows.json',MAIN/'unified_trial/simulation.py',MODEL/'assets/common.xml',MODEL/'assets/mapping.json',MODEL/'shared.py',MODEL/'COMMON_CONTRACT.json']
    fk_path=OLD/'work/roughness_scan.py'; ply_path=CON/'terrain_surface_stage1_2026-09-15/inspect_cloud.py'
    inputs += [fk_path,ply_path]
    assert sha(MODEL/'assets/common.xml')=='1e3cda186d3c9da830c80d9162222feb030de10f550d6b33025f3c5374bc79cb'
    fk=module('archived_fk',fk_path).FK(MODEL/'assets/common.xml')
    load_ply=module('archived_ply',ply_path).load_ply
    names=fk.joints; assert len(names)==173 and len(set(names))==173
    pitch_ids=[names.index(f'legbase_pitch_{i}_{s}') for s in ['L','R'] for i in range(19)]
    q=np.zeros((2,173));q[1,pitch_ids]=np.deg2rad(20)
    p,r=fk.solve(q,names);center=p[:,fk.spine_ids].mean(1)
    level=np.einsum('tji,tkj->tki',r[:,fk.trunk_id],p-center[:,None]);feet=level[:,fk.foot_ids]
    np.savez_compressed(HERE/'geometry_candidates.npz',pitch_candidates_deg=[0,20],joint_positions_rad=q,joint_names=names,feet_m=feet)
    fig,axes=plt.subplots(1,2,figsize=(13,3.5),layout='constrained')
    for k,(label,color) in enumerate([('Pitch 0 deg','#137b80'),('Pitch +20 deg','#d87731')]):
        for ax,dims,labels in zip(axes,[(0,1),(1,2)],[('X (mm)','Y (mm)'),('Y (mm)','Z (mm)')]):
            pts=level[k]*1000
            for i,(parent,*_) in enumerate(fk.nodes):
                if parent>=0: ax.plot(pts[[parent,i],dims[0]],pts[[parent,i],dims[1]],color=color,alpha=.35,lw=.8)
            ax.scatter(pts[fk.foot_ids,dims[0]],pts[fk.foot_ids,dims[1]],s=23,color=color,label=label)
            ax.set(xlabel=labels[0],ylabel=labels[1]);ax.set_aspect('equal');ax.grid(alpha=.2)
    axes[0].set_title('Top: all 38 foot centers');axes[1].set_title('Front: positive pitch lowers feet')
    axes[0].legend(loc='lower center',bbox_to_anchor=(.5,1.15),ncol=2,fontsize=9)
    fig.suptitle('Fixed-pose candidates only - neither selected\nAll other joints zero; level trunk frame; no dynamics',fontsize=14)
    fig.savefig(HERE/'fixed_pose_options.png',dpi=160);plt.close(fig)
    surfaces=[]
    for site,filename in [('Libo','Libo-9-3-1-3.ply'),('Narvik','Narvik-4-2-5-3.ply')]:
        path=DATA/'rock_joint/database'/site/'01 PLY File'/filename
        xyz,_=load_ply(path);xyz*=.001
        x,y=np.unique(xyz[:,0]),np.unique(xyz[:,1]);assert len(x)==len(y)==201 and len(xyz)==40401
        ix,iy=np.searchsorted(x,xyz[:,0]),np.searchsorted(y,xyz[:,1]);assert len(np.unique(iy*len(x)+ix))==len(xyz)
        h=np.full((len(y),len(x)),np.nan);h[iy,ix]=xyz[:,2];x-=(x[0]+x[-1])/2;y-=(y[0]+y[-1])/2
        inputs.append(path);surfaces.append((site.lower(),site+' measured rock',h,x,y,np.ones_like(h,bool),str(path)))
    grid_path=CON/'outputs/terrain_surface_reconstruction_2026-09-15/full_fft_roundtrip.npz'
    audit_path=grid_path.parent/'RECONSTRUCTION_AUDIT.json';audit=read(audit_path);cloud=Path(audit['source'])
    assert sha(cloud)==audit['source_sha256'];inputs += [grid_path,audit_path,cloud]
    with np.load(grid_path) as z:
        h=z['raw_gridded_height_m'];axis=(np.arange(h.shape[-1])-(h.shape[-1]-1)/2)*float(z['dx_m'])
        for i in range(len(h)):surfaces.append((f'soil_{i+1}',f'Measured soil window {i+1}',h[i],axis,axis,z['observed_masks'][i],str(grid_path)))
    arrays={};terrain_meta=[]
    for key,label,h,x,y,mask,path in surfaces:
        residual,plane=detrend(h,x,y)
        arrays.update({key+'_height_m':h,key+'_x_m':x,key+'_y_m':y,key+'_observed_mask':mask})
        terrain_meta.append(dict(id=key,label=label,source=path,shape=list(h.shape),vertex_span_mm=[float(np.ptp(x)*1000),float(np.ptp(y)*1000)],spacing_mm=[float(np.median(np.diff(x))*1000),float(np.median(np.diff(y))*1000)],observed_fraction=float(mask.mean()),fitted_plane_coefficients=plane.tolist(),residual_Sq_mm=float(np.std(residual)*1000)))
    np.savez_compressed(HERE/'measured_candidates.npz',**arrays)
    fig=plt.figure(figsize=(13,11),layout='constrained')
    for row,index in enumerate([0,1,2]):
        key,label,h,x,y,mask,_=surfaces[index];xx,yy=np.meshgrid(x*1000,y*1000)
        ax=fig.add_subplot(3,2,row*2+1,projection='3d')
        ax.plot_surface(xx,yy,(h-h.mean())*1000,cmap='terrain',rcount=90,ccount=90,linewidth=0)
        ax.set(xlabel='X (mm)',ylabel='Y (mm)',zlabel='Z - mean (mm)',title=label+': original slope retained')
        ax.set_box_aspect((np.ptp(x),np.ptp(y),max(np.ptp(h),.003)))
        ax.set_zticks([float((h.min()-h.mean())*1000),float((h.max()-h.mean())*1000)])
        from matplotlib.ticker import FormatStrFormatter
        ax.zaxis.set_major_formatter(FormatStrFormatter('%.1f'))
        residual,_=detrend(h,x,y);ax2=fig.add_subplot(3,2,row*2+2);v=float(abs(residual).max()*1000)
        im=ax2.pcolormesh(x*1000,y*1000,residual*1000,shading='nearest',cmap='RdBu_r',vmin=-v,vmax=v)
        ax2.set(xlabel='X (mm)',ylabel='Y (mm)',title='Plane-removed preview only; not adopted');ax2.set_aspect('equal')
        fig.colorbar(im,ax=ax2,label='Residual height (mm)')
        if not mask.all():
            my,mx=np.where(~mask);ax2.scatter(x[mx]*1000,y[my]*1000,c='black',s=2,label='Nearest-filled cells');ax2.legend(fontsize=8)
    fig.suptitle('Measured surface candidates at original XY size\nRock: 100 mm; soil: 508 mm vertex span (512 mm source window)',fontsize=15)
    fig.savefig(HERE/'measured_surface_options.png',dpi=150);plt.close(fig)
    cache_path=CON/'open_loop_review_2026-09-16/optimization/command_cache_nonnegative.py';inputs.append(cache_path)
    assert sha(cache_path)=='11ce115d9e297e7c96a978d31491912ca8a3acb832d64d8b8f9ae78890607750'
    sys.path.insert(0,str(MODEL));cache_module=module('current_command_cache',cache_path)
    rows,campaign=read(MAIN/'rows.json'),read(MAIN/'campaign.json');selected=[];cells=[]
    for job in campaign['jobs']:
        if job['name'] not in ['flat_low','flat_high']:continue
        plan_path=Path(job['args']['experiment_plan']);assert sha(plan_path)==job['args']['experiment_plan_sha256'];plan=read(plan_path)
        assert plan['body_factors']==[c['body_factor'] for c in job['cells']]
        assert plan['leg_cluster_factors']==[c['leg_factor'] for c in job['cells']]
        inputs.append(plan_path);selected += [rows[i] for i in job['source_ids']];cells += job['cells']
    cache=cache_module.CommandCache(names,.001,selected);inputs += list({Path(r['command_file']) for r in selected})
    q_cycle=cache.position[6000:7000].copy()
    body_ids=[i for i,n in enumerate(names) if n.startswith('body_yaw_')];yaw_ids=[i for i,n in enumerate(names) if n.startswith('legbase_yaw_')]
    for e,c in enumerate(cells):
        q_cycle[:,e,body_ids]*=np.float32(c['body_factor']);q_cycle[:,e,yaw_ids]*=np.float32(c['leg_factor'])
    gait_feet=np.array([fk.footprints(q_cycle[:,e],names) for e in range(len(cells))]);pitches=q_cycle[:,:,pitch_ids].transpose(1,0,2)
    stance=pitches>=0 # Historical diagnostic convention, not a newly adopted window.
    np.savez_compressed(HERE/'current_gait_geometry.npz',waves=[c['wave'] for c in cells],joint_names=names,phase=np.arange(1000)/1000,joint_positions_rad=q_cycle.transpose(1,0,2),feet_m=gait_feet,leg_pitch_rad=pitches,historical_nominal_stance=stance)
    old_path=OLD/'work/footprints.npz';inputs.append(old_path)
    with np.load(old_path) as old:
        order=[old['joint_names'].tolist().index(n) for n in names]
        baseline_error=float(np.max(abs(old['command_q'][:1000,:,order]-cache.position[6000:7000])))
        geometry_error=float(np.max(abs(old['feet_m'][:,:1000]-gait_feet)))
    report=dict(status='PREPARATION_ONLY_PENDING_USER_CHOICES',no_sweep_or_physics_run=True,
        pending=['measured surface family','height variants and treatment of plane','placement/heading sampling and nominal phase windows'],
        fixed_definition='One shared neutral 38-foot XY template; foot Z excluded; pitch is not a sweep parameter',
        metric='sqrt(mean_frames(mean_unordered_pairs(atan2(abs(delta_height),distance_XY)**2)))',metric_units='Degrees after square root; calculate angles in radians',
        fixed_pair_count=703,nominal_stance_pair_count=171,foot_point_definition='Foot-sphere center XY projected onto height surface; not finite-sphere tangent contact or measured force contact',foot_radius_mm=.45,
        coordinate_frame='Level link_body_1, centered at 21-spine-body centroid',
        fixed_candidates=[dict(pitch_deg=d,foot_span_xyz_mm=(np.ptp(feet[i],axis=0)*1000).tolist(),mean_foot_z_mm=float(feet[i,:,2].mean()*1000)) for i,d in enumerate([0,20])],
        terrains=terrain_meta,gait_cells=cells,original_command_cycle_vs_old_cache_max_error_rad=baseline_error,
        latest_geometry_vs_old_cache_max_coordinate_difference_mm=geometry_error*1000,
        historical_nominal_stance_count_range=[int(stance.sum(-1).min()),int(stance.sum(-1).max())],
        cycle_source_indices=[6000,6999],sample_hold_s=.001,gait_max_radius_mm=float(np.linalg.norm(gait_feet[...,:2],axis=-1).max()*1000),
        gait_geometry_limits='Nominal CPG/FK geometry; no terrain contact, compliance, world-pitch feedback or dynamic body motion.',
        checks=checks(),sources={str(p):sha(p) for p in sorted(set(inputs))})
    (HERE/'PREPARATION.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    print(json.dumps({k:report[k] for k in ['status','fixed_candidates','original_command_cycle_vs_old_cache_max_error_rad','latest_geometry_vs_old_cache_max_coordinate_difference_mm','historical_nominal_stance_count_range','gait_max_radius_mm','checks']},indent=2))

if __name__=='__main__':prepare()
