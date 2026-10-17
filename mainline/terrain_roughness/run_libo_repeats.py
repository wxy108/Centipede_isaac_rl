"""Paired Libo spawn observations; stuck/negative motion are valid outcomes."""
from pathlib import Path
import os
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:os.environ[key]='1'
os.environ['PXR_WORK_THREAD_LIMIT']='2'
import sys,json,hashlib,copy,argparse,subprocess,time,traceback,ast,itertools
import numpy as np
import run_shortwave as engine
ROOT=Path(__file__).resolve().parent;MAIN=ROOT.parent
OUT=ROOT/'libo_spawn_repeats';SIM=OUT/'simulation'
ROI=engine.ROI;SCALES=[.25,.5,.75,1.];WAVES=[1.5,2.,2.5,3.,3.5]
read,sha,dump=engine.read,engine.sha,engine.dump
SEED=2026092901

def collision_radii():
    """Exact authored colliders, conservatively enclosed by owner-centered spheres."""
    lib=Path(sys.prefix)/'Lib/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+8131b85d.wx64.r.cp311'
    sys.path.insert(0,str(lib));dll=os.add_dll_directory(str(lib/'bin'))
    from pxr import Usd,UsdGeom,UsdPhysics,Gf
    asset=Path('X:/Projects/_archive/Codex/2026-09-11/con/round9_wave_amplitude_2026-09-14/assets/common.usda')
    stage=Usd.Stage.Open(str(asset));cache=UsdGeom.XformCache()
    bodies=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    radii={p.GetName():0. for p in bodies};assert len(radii)==251
    shapes=[]
    for p in stage.Traverse():
        if not p.HasAPI(UsdPhysics.CollisionAPI):continue
        assert UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()
        owner=p
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
        assert owner
        kind=p.GetTypeName()
        if kind=='Mesh':
            vertices=np.array(UsdGeom.Mesh(p).GetPointsAttr().Get(),float);extent=[vertices.min(0),vertices.max(0)]
        elif kind=='Sphere':
            radius=float(UsdGeom.Sphere(p).GetRadiusAttr().Get());extent=[np.full(3,-radius),np.full(3,radius)]
        elif kind=='Capsule':
            cap=UsdGeom.Capsule(p);rr=float(cap.GetRadiusAttr().Get());half=np.full(3,rr)
            half['XYZ'.index(cap.GetAxisAttr().Get())]+=float(cap.GetHeightAttr().Get())/2;extent=[-half,half]
        else:raise ValueError(f'Unreviewed collider type:{kind}')
        points=np.array(list(itertools.product(*zip(extent[0],extent[1]))),float)
        transform=cache.GetLocalToWorldTransform(p)
        world=np.array([transform.Transform(Gf.Vec3d(*x)) for x in points])
        origin=np.array(cache.GetLocalToWorldTransform(owner).ExtractTranslation())
        radius=float(np.linalg.norm(world-origin,axis=1).max())
        radii[owner.GetName()]=max(radii[owner.GetName()],radius)
        shapes.append(dict(path=str(p.GetPath()),owner=owner.GetName(),type=p.GetTypeName(),radius_m=radius))
    assert len(shapes)==249 and max(radii.values())<.02
    return dict(asset=str(asset),asset_sha256=sha(asset),body_radius_m=radii,shapes=shapes,
        method='Mesh vertex bounds; Sphere radius; Capsule radius+halfheight, using actual primitive parameters rather than stale authored extent metadata. Transform corners to world, enclose about rigid-body origin. No geometry changes.')

def prepare():
    from scipy.spatial import ConvexHull
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    assert not (SIM/'campaign.json').exists(),'Preserve prepared campaign'
    old=read(MAIN/'campaign.json');base=read(MAIN/'rows.json');pd=read(MAIN/'pd_wave4_firm20.json')['absolute_joint_pd']
    for p,h in old['execution_and_input_pins'].items():assert sha(p)==h,p
    flats=[j for j in old['jobs'] if j['name'].startswith('flat')]
    cells=sum([j['cells'] for j in flats],[])[:5];assert [c['wave'] for c in cells]==WAVES
    bounds=collision_radii();rmax=max(bounds['body_radius_m'].values())
    refs=[];origin_offset=None;flat_sources={}
    for job in flats:
        path=MAIN/'runs/v4'/job['name']/'trajectory.npz';flat_sources[str(path)]=sha(path)
        with np.load(path) as z:
            pose=z['pose'][z['t']<=6+1e-9,:,:,:3]
            for e,c in enumerate(job['cells']):
                if c['wave']>3.5:continue
                origin=pose[0,e].mean(0)
                root_pose=np.array(read(path.parent/'readback.json')['initial_pose'][e])
                delta=origin[:2]-root_pose[:2]
                if origin_offset is None:origin_offset=delta
                np.testing.assert_allclose(delta,origin_offset,atol=1e-7,rtol=0)
                refs.append((pose[:,e,:,:2]-origin[:2]).reshape(-1,2))
    refs=np.concatenate(refs);hull=refs[ConvexHull(refs).vertices]
    rng=np.random.default_rng(SEED);spawns=[];roi=np.array([[ROI[0],ROI[1]],[ROI[2],ROI[3]]])
    for i in range(10):
        yaw_deg=90. if i==2 else float((i+rng.random())*36)
        angle=np.deg2rad(yaw_deg);rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        rotated=hull@rotation.T
        lo=roi[:,0]-rotated.min(0)+.03+rmax;hi=roi[:,1]-rotated.max(0)-.03-rmax
        assert np.all(hi>lo)
        center=lo+rng.uniform(.1,.9,2)*(hi-lo)
        root=center-origin_offset@rotation.T
        actual=rotated+center
        margins=[float(actual[:,0].min()-ROI[0]-rmax),float(ROI[1]-actual[:,0].max()-rmax),float(actual[:,1].min()-ROI[2]-rmax),float(ROI[3]-actual[:,1].max()-rmax)]
        assert min(margins)>.03
        spawns.append(dict(spawn_index=i,start_xy_m=root.tolist(),start_yaw_deg=yaw_deg,initial_mean_link_center_xy_m=center.tolist(),
            initial_center_sampling_box_m=[lo.tolist(),hi.tolist()],predicted_flat_collision_margins_m=margins,
            forward_xy=[float(-np.cos(angle)),float(-np.sin(angle))],CPG_initial_phase='Original command sample 0'))
    OUT.mkdir(exist_ok=True);SIM.mkdir(exist_ok=False);snap=SIM/'executed_source';snap.mkdir()
    adapted=engine.adapted_source()
    # Reference row identity is independent of the new 50-row spawn list.
    a=adapted.index('    reference_ids = [row["baseline_source_id"] for row in rows]')
    b=adapted.index('    # New terrain placement is a declared generated expectation',a)
    adapted=adapted[:a]+'''    reference_ids = [row["baseline_source_id"] for row in rows]
    with np.load(experiment["baseline_backend"]["path"]) as original:
        np.testing.assert_array_equal(as_numpy(articulation_view.get_masses()),original["masses"][reference_ids])
        np.testing.assert_array_equal(as_numpy(articulation_view.get_inertias()),original["inertias"][reference_ids])
    expected_pd=np.array([table[name] for name in joint_names],dtype=stiffness.dtype)
    np.testing.assert_array_equal(stiffness,np.repeat(expected_pd[None,:,0],environment_count,axis=0))
    np.testing.assert_array_equal(damping,np.repeat(expected_pd[None,:,1],environment_count,axis=0))
'''+adapted[b:]
    start=adapted.index('        # A cropped collision mesh is valid only while every body stays well inside.')
    end=adapted.index('        if (step_index + 1) % recording_stride == 0:',start)
    adapted=adapted[:start]+'''        # Measurement-only shape envelope guard; no wall, clamp or reset applied.
        local_xy = robot.data.body_link_pose_w[:, :, :2].detach().cpu().numpy() - offsets[:, None, :2]
        radii=np.array([experiment["collision_radius_m"][n] for n in body_names])[None,:]
        bx0,bx1,by0,by1=experiment["terrain_roi_m"]
        env_margins=np.stack([(local_xy[:,:,0]-radii-bx0).min(1),(bx1-local_xy[:,:,0]-radii).min(1),
                             (local_xy[:,:,1]-radii-by0).min(1),(by1-local_xy[:,:,1]-radii).min(1)],axis=1)
        boundary_margin=float(env_margins.min())
        minimum_collision_margin=min(minimum_collision_margin,boundary_margin)
        if boundary_margin < 0.015:
            failure_t=(step_index+1)*args.dt
            failure_pose=as_numpy(robot.data.body_link_pose_w);failure_pose[:,:,:3]-=offsets[:,None,:]
            failure_v=as_numpy(robot.data.joint_vel);failure_body_v=as_numpy(robot.data.body_link_vel_w)
            summary=[]
            for ee,row in enumerate(rows):
                p=failure_pose[ee,:,:3]
                summary.append(dict(environment_index=ee,source_id=source_ids[ee],wave=row["wave"],spawn_index=row["spawn_index"],
                    margins_m=env_margins[ee].tolist(),xyz_min_m=p.min(0).tolist(),xyz_max_m=p.max(0).tolist(),
                    max_abs_joint_velocity_rad_s=float(abs(failure_v[ee]).max()),
                    max_body_linear_speed_m_s=float(np.linalg.norm(failure_body_v[ee,:,:3].astype(float),axis=-1).max())))
            common.write(run_directory/"BOUNDARY_FAILURE.json",dict(t=failure_t,minimum_collision_margin_m=boundary_margin,environments=summary,
                offender_source_ids=[source_ids[ee] for ee in range(environment_count) if env_margins[ee].min()<.015],
                saved_regular_frame_end_s=frame_times[-1],collision_radius_m=experiment["collision_radius_m"]))
            np.savez_compressed(run_directory/"failure_state.npz",t=failure_t,pose=failure_pose,q=as_numpy(robot.data.joint_pos),v=failure_v,body_velocity=failure_body_v,body_names=body_names,joint_names=joint_names)
            np.savez_compressed(run_directory/"trajectory_partial.npz",t=frame_times,episode=frame_episodes,pose=body_poses,q=joint_positions,v=joint_velocities,body_velocity=body_velocities,body_names=body_names,joint_names=joint_names)
            raise RuntimeError("Collision envelope entered 15 mm boundary guard")
'''+adapted[end:]
    adapted=engine.changed(adapted,'    for step_index in range(steps):','    minimum_collision_margin=float("inf")\n    for step_index in range(steps):')
    adapted=engine.changed(adapted,'    measured_wall = time.perf_counter() - measured_started if measured_started else None','    common.write(run_directory/"collision_guard.json",dict(minimum_all_step_collision_margin_m=minimum_collision_margin,all_step_guard_m=.015,body_radius_m=experiment["collision_radius_m"]))\n    measured_wall = time.perf_counter() - measured_started if measured_started else None')
    ast.parse(adapted);(snap/'adapted_functions.py').write_text(adapted,encoding='utf-8')
    terrain=read(ROOT/'height_amplitude_4mm/REPORT.json')
    jobs=[]
    for scale in [0.,*SCALES]:
        isflat=scale==0.;level='flat' if isflat else 'height_x'+f'{scale:.2f}'.replace('.','p')
        tr=None if isflat else next(t for t in terrain['terrains'] if t['site']=='libo' and t['height_scale']==scale)
        if tr:
            assert sha(tr['path'])==tr['sha256']
            with np.load(tr['path']) as z:h=z['height'];dx=float(z['dx_m'])
        rows=[];initials=[]
        spawn_ids=[2] if isflat else range(10)
        for sid in spawn_ids:
            sp=spawns[sid]
            for cell in cells:
                row=copy.deepcopy(base[cell['source_id']]);i=len(rows)
                row.update(id=f'{level}_s{sid:02d}_w{cell["wave"]:g}',terrain='flat' if isflat else 'rough',
                    start_xy=sp['start_xy_m'],start_yaw_deg=sp['start_yaw_deg'],start_index=0,spawn_index=sid,
                    baseline_source_id=cell['source_id'],height_scale=scale,terrain_site='flat' if isflat else 'libo')
                if tr:row.update(surface_file=tr['path'],surface_sha256=tr['sha256'],terrain_family=1,terrain_seed=1501,restoration_level='restore4')
                pose=np.array(row['matched_initial_root_pose'],float);pose[:2]=sp['start_xy_m'];ang=np.deg2rad(sp['start_yaw_deg'])
                pose[3:]=[np.cos(ang/2),0,0,np.sin(ang/2)]
                if tr:
                    ny,nx=h.shape;ix=int(pose[0]/dx+(nx-1)/2);iy=int(pose[1]/dx+(ny-1)/2);rad=int(np.ceil(.075/dx))
                    pose[2]+=h[max(0,iy-rad):min(ny,iy+rad+1),max(0,ix-rad):min(nx,ix+rad+1)].max()+.002
                assert row['absolute_joint_pd']==pd and not row['family_multipliers']
                rows.append(row);initials.append(pose.tolist())
        rp=SIM/f'{level}_rows.json';dump(rp,rows)
        for sid in spawn_ids:
            loc=0 if isflat else sid*5
            for group,ids,blueprint in [('low',list(range(loc,loc+3)),flats[0]),('high',list(range(loc+3,loc+5)),flats[1])]:
                name=f'{level}_s{sid:02d}_{group}'
                selected=[]
                for i in ids:
                    cc=cells[i%5]
                    selected.append(dict(cc,source_id=i,baseline_source_id=cc['source_id'],spawn_index=sid,height_scale=scale,
                        surface=level,fixed_pair_angle_rms_deg=0. if isflat else tr['fixed_pair_angle_rms_deg']))
                plan=read(blueprint['args']['experiment_plan']);n=len(ids)
                plan.update(schema='libo_paired_spawn_observation_v1',run_name=name,duration_s=6.,benchmark_duration_s=6.,
                    selected_environments=ids,benchmark_source_ids=ids,rows_sha256=sha(rp),pd_overrides={str(i):pd for i in ids},
                    planned_initial_pose=initials,terrain_roi_m=ROI,body_factors=[c['body_factor'] for c in selected],
                    leg_cluster_factors=[c['leg_factor'] for c in selected],joint_cells=selected,quality_offsets_m=[[0.,0.,0.]]*n,
                    episode1_permutation=list(range(n)),worker_filename=Path(__file__).name,collision_radius_m=bounds['body_radius_m'],
                    campaign_note='User authorized 200 paired Libo trials plus five rotated flat controls; stuck and reverse motion are valid outcomes',
                    quality_protocol=dict(duration_s=6.,comparison='Shared10spawns, four terrain heights, five fixed waves',only_focal_poses_recorded=False),
                    scaling_benchmark=dict(backend='cpu',replicas=n,independent_trials=False,warmup_s=.1,measured_s=5.9,all_replicas_world_pitch_front5=True,contact_force_logging=False,all_replicas_recorded=True))
                pp=SIM/f'{name}_plan.json';dump(pp,plan)
                args=copy.deepcopy(blueprint['args']);args.update(rows=str(rp),name=name,duration=6.,experiment_plan=str(pp),experiment_plan_sha256=sha(pp))
                jobs.append(dict(name=name,level=level,height_scale=scale,spawn_index=sid,group=group,args=args,source_ids=ids,cells=selected,rows_sha256=sha(rp)))
    batches=[dict(name='flat_controls',jobs=[j['name'] for j in jobs if j['level']=='flat'])]
    for pair in range(5):
        # Cyclic terrain order balances first/last position without changing comparison slots.
        order=SCALES[pair%4:]+SCALES[:pair%4]
        for scale in order:
            js=[j['name'] for sid in [2*pair,2*pair+1] for group in ['low','high'] for j in jobs if j['height_scale']==scale and j['spawn_index']==sid and j['group']==group]
            assert len(js)==4;batches.append(dict(name=f'pair{pair:02d}_height{scale:g}',jobs=js))
    sources=[Path(__file__),ROOT/'run_shortwave.py',MAIN/'trial_profile.json',MAIN/'pd_wave4_firm20.json',*sorted((MAIN/'unified_trial').glob('*.py')),MAIN/'campaign.json',MAIN/'rows.json',ROOT/'height_amplitude_4mm/REPORT.json']
    pins={str(p):sha(p) for p in sources};pins[str(snap/'adapted_functions.py')]=sha(snap/'adapted_functions.py');pins[bounds['asset']]=bounds['asset_sha256']
    for p in sources:(snap/p.name).write_bytes(p.read_bytes())
    campaign=dict(schema='libo_spawn_repeats_v1',jobs=jobs,batches=batches,pins=pins,profile=old['profile'],profile_sha256=old['profile_sha256'],fixed_pd_sha256=old['fixed_pd_sha256'],
        duration_s=6.,terrain_roi_m=ROI,workers=4,threads_per_worker=2,environments_per_worker=[3,2,3,2],height_scales=SCALES,waves=WAVES,
        rough_condition_count=200,new_flat_condition_count=5,condition_count=205,spawns=spawns,spawn_seed=SEED,collision_bounds=bounds,
        flat_reference_sources=flat_sources,flat_cluster_cells=cells,initialization='Original local-max height clearance rule, zero root/joint velocity, zero q, originalCPGphase0; actual yaw authored per spawn',
        roughness_definition=terrain['roughness_sampling'],scope='Paired location/heading observations; no amplitude optimization, convergence or capacity search',
        boundary_policy='Thirty mm precomputed collision-envelope reserve based on known flat movement;15mm all-step shape-sphere guard. No walls/clamps.',
        failure_policy='Triggering environment retained as failure; peer interruptions rerun at same spawn with actual remaining conditions and same two threads. No resampling failed spawn.')
    dump(SIM/'campaign.json',campaign)
    fig,ax=plt.subplots(figsize=(9,7));ax.set_aspect('equal');ax.set(xlim=(ROI[0],ROI[1]),ylim=(ROI[2],ROI[3]),xlabel='X (m)',ylabel='Y (m)',title='Libo: ten shared spawn positions and initial headward directions')
    colors=plt.cm.tab10(np.arange(10))
    for sp,color in zip(spawns,colors):
        a=np.deg2rad(sp['start_yaw_deg']);rot=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
        pp=hull@rot.T+sp['initial_mean_link_center_xy_m'];ax.fill(pp[:,0],pp[:,1],color=color,alpha=.08)
        x,y=sp['initial_mean_link_center_xy_m'];fx,fy=sp['forward_xy'];ax.scatter(x,y,color=color);ax.arrow(x,y,.035*fx,.035*fy,width=.001,color=color,length_includes_head=True);ax.text(x+.005,y+.005,str(sp['spawn_index']))
    ax.grid(alpha=.2);fig.text(.05,.015,'Shaded: known six-second flat body-center envelope; spawn safety also includes collider radius + 30 mm reserve.',fontsize=9)
    fig.savefig(OUT/'spawn_preview.png',dpi=150);plt.close(fig)
    check();print('PREPARED:200 rough+5 flat conditions, ten fixed spawns,20 four-worker batches+flat controls',flush=True)

def check():
    c=read(SIM/'campaign.json')
    for p,h in c['pins'].items():assert sha(p)==h,p
    pd=read(MAIN/'pd_wave4_firm20.json')['absolute_joint_pd'];seen=set()
    for j in c['jobs']:
        args=j['args'];assert args['dt']==.0005 and args['duration']==6 and args['position_iterations']==64 and args['velocity_iterations']==16
        assert sha(args['rows'])==j['rows_sha256'] and sha(args['experiment_plan'])==args['experiment_plan_sha256']
        p=read(args['experiment_plan']);rows=read(args['rows']);assert p['benchmark_source_ids']==j['source_ids']
        for cell,i in zip(j['cells'],j['source_ids']):
            row=rows[i];assert row['absolute_joint_pd']==pd and not row['family_multipliers']
            reference=next(t for t in c['flat_cluster_cells'] if t['wave']==cell['wave'])
            assert all(cell[k]==reference[k] for k in ['A_deg','B_deg','body_factor','leg_factor'])
            sp=c['spawns'][row['spawn_index']];assert row['start_xy']==sp['start_xy_m'] and row['start_yaw_deg']==sp['start_yaw_deg']
            if row['terrain']=='rough':assert sha(row['surface_file'])==row['surface_sha256']
            key=(j['height_scale'],cell['wave'],cell['spawn_index']);assert key not in seen;seen.add(key)
    assert len(seen)==205
    print('INPUT CHECKS PASS:full173PD, frozen A/B, rotated placement, collision envelope, timing, source hashes',flush=True)

def worker(name,attempt,keep=None):
    c=read(SIM/'campaign.json');j=next(j for j in c['jobs'] if j['name']==name)
    for p,h in c['pins'].items():assert sha(p)==h,p
    args=argparse.Namespace(**j['args']);rows=read(args.rows);plan=read(args.experiment_plan)
    selected=j['source_ids'] if keep is None else keep;assert selected and set(selected)<=set(j['source_ids'])
    D=SIM/'runs'/attempt/name;D.mkdir(parents=True,exist_ok=False)
    if keep is not None:
        positions=[j['source_ids'].index(i) for i in selected];n=len(selected)
        plan.update(benchmark_source_ids=selected,selected_environments=selected,body_factors=[plan['body_factors'][i] for i in positions],
            leg_cluster_factors=[plan['leg_cluster_factors'][i] for i in positions],joint_cells=[plan['joint_cells'][i] for i in positions],
            quality_offsets_m=[[0.,0.,0.]]*n,episode1_permutation=list(range(n)),peer_recovery=True,original_case_slots=positions)
        plan['scaling_benchmark']['replicas']=n
        dump(D/'effective_plan.json',plan);args.experiment_plan=str(D/'effective_plan.json');args.experiment_plan_sha256=sha(args.experiment_plan)
    log=(D/'worker.log').open('w',encoding='utf-8');sys.stdout.flush();sys.stderr.flush();os.dup2(log.fileno(),sys.stdout.fileno());os.dup2(log.fileno(),sys.stderr.fileno())
    active=dict(pid=os.getpid(),job=name,status='starting',source_ids=selected,started_unix=time.time());dump(D/'active.json',active);exitcode=1
    try:
        os.environ['WARP_CACHE_PATH']=str(D/'warp_cache');sys.path.insert(0,str(engine.CON/'open_loop_review_2026-09-16'));sys.path.insert(0,str(engine.NI/'work/runtime_speed_2026-09-17/deps'))
        import warp as wp
        assert wp.__version__=='1.8.1';wp.init()
        from isaaclab.app import AppLauncher
        launcher=AppLauncher(args)
        import torch
        torch.set_num_threads(1);torch.set_num_interop_threads(1)
        sys.path.insert(0,str(engine.CON/'terrain_surface_library_2026-09-15'));sys.path.insert(0,str(MAIN))
        from unified_trial import simulation
        patch=SIM/'executed_source/adapted_functions.py';exec(compile(patch.read_text(encoding='utf-8'),str(patch),'exec'),simulation.__dict__)
        from types import SimpleNamespace
        runtime=SimpleNamespace(args=args,experiment=plan,rows=rows,input_receipt=dict(input_rows_sha256=j['rows_sha256'],profile_sha256=c['profile_sha256'],fixed_pd_sha256=c['fixed_pd_sha256'],full_table_exact=True,flat_cluster_amplitudes_unchanged=True),run_directory=D,execution_sources=dict(c['pins'],**{str(args.experiment_plan):sha(args.experiment_plan)}))
        active['status']='running';dump(D/'active.json',active);simulation.run_trial(runtime)
        result=read(D/'results.json');assert result['complete'] and result['whole_batch_finite'] and result['actual_duration_s']==6.
        assert not read(D/'native_error_audit.json')['errors']
        assert read(D/'collision_guard.json')['minimum_all_step_collision_margin_m']>.015
        active['status']='finished';exitcode=0
    except BaseException:active['status']='failed';traceback.print_exc()
    finally:active.update(exit_code=exitcode,finished_unix=time.time());dump(D/'active.json',active);sys.stdout.flush();sys.stderr.flush();log.flush()
    os._exit(exitcode)

def failure_peers(d,source_ids):
    if (d/'BOUNDARY_FAILURE.json').exists():offenders=read(d/'BOUNDARY_FAILURE.json')['offender_source_ids']
    elif (d/'NUMERICAL_FAILURE.json').exists():offenders=[source_ids[i] for i in read(d/'NUMERICAL_FAILURE.json')['environments']]
    else:raise RuntimeError(f'Non-quality worker failure; stop for diagnosis:{d}')
    assert offenders and set(offenders)<=set(source_ids)
    return [i for i in source_ids if i not in offenders],offenders

def run():
    import psutil
    check();c=read(SIM/'campaign.json');assert not (SIM/'runs/v1').exists()
    assert psutil.virtual_memory().available>10*1024**3
    started=time.time();finished=[];queue=[]
    def batch(entries,label):
        procs=[]
        for name,attempt,keep in entries:
            cmd=[sys.executable,'-B',str(Path(__file__)),'worker','--job',name,'--attempt',attempt]
            if keep is not None:cmd+=['--keep',','.join(map(str,keep))]
            with (SIM/f'{attempt}_{name}_launch.log').open('wb') as log:
                p=subprocess.Popen(cmd,cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            procs.append((name,attempt,keep,p));time.sleep(2)
        while True:
            current=[]
            for name,attempt,keep,p in procs:
                d=SIM/'runs'/attempt/name
                try:a=read(d/'active.json')
                except(FileNotFoundError,json.JSONDecodeError):a={}
                current.append(dict(job=name,attempt=attempt,keep=keep,pid=p.pid,exit=p.poll(),status=a.get('status')))
            dump(SIM/'supervisor.json',dict(complete=False,batch=label,started_unix=started,elapsed_s=time.time()-started,finished=finished,current=current))
            if all(p.poll() is not None for *_,p in procs):break
            time.sleep(10)
        finished.extend(current)
        for item in current:
            if item['exit']!=0:
                j=next(j for j in c['jobs'] if j['name']==item['job']);ids=item['keep'] or j['source_ids']
                peers,off=failure_peers(SIM/'runs'/item['attempt']/item['job'],ids)
                item['offender_source_ids']=off;item['interrupted_peer_source_ids']=peers
                if peers:queue.append((item['job'],'peer'+str(len(ids)-len(peers))+f'_from_{item["attempt"]}',peers))
        print(json.dumps(dict(batch=label,attempted_groups=len(finished),elapsed_s=time.time()-started,peer_recoveries_pending=len(queue))),flush=True)
    for b in c['batches']:batch([(name,'v1',None) for name in b['jobs']],b['name'])
    recovery_round=0
    while queue:
        recovery_round+=1;assert recovery_round<=3
        current=queue[:];queue.clear()
        for i in range(0,len(current),4):batch(current[i:i+4],f'peer_recovery{recovery_round}_{i//4}')
    dump(SIM/'supervisor.json',dict(complete=True,all_attempts_accounted_for=True,started_unix=started,elapsed_s=time.time()-started,finished=finished,current=[],meaning='Complete means all205 planned observations attempted and peers recovered; divergent/boundary trials remain failed.'))
    print('CAMPAIGN ATTEMPTS FINISHED; no further physics queued',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','check','run','worker']);p.add_argument('--job');p.add_argument('--attempt',default='v1');p.add_argument('--keep');a=p.parse_args()
    if a.mode=='prepare':prepare()
    elif a.mode=='check':check()
    elif a.mode=='run':run()
    else:worker(a.job,a.attempt,None if a.keep is None else list(map(int,a.keep.split(','))))
