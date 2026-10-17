"""36 short Isaac CPU PhysX previews with frozen flat-cluster amplitudes and full PD.
prepare: build reviewable inputs; run: exactly three four-worker batches, no retries.
"""
from pathlib import Path
import os
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:os.environ[key]='1'
os.environ['PXR_WORK_THREAD_LIMIT']='2'
import sys,json,hashlib,time,copy,argparse,subprocess,traceback,ast
import numpy as np
ROOT=Path(__file__).resolve().parent;MAIN=ROOT.parent;OUT=ROOT/'shortwave';SIM=OUT/'simulation'
CON=Path('X:/Projects/_archive/Codex/2026-09-11/con');NI=Path('X:/Projects/_archive/Codex/2026-09-15/ni')
LEVELS=['original','restore4','restore2'];SITES=['libo','narvik'];DURATION=6.
ROI=[-.079,.321,-.241,-.001] # Same vertex-aligned rectangle in every condition.
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,d):Path(p).parent.mkdir(parents=True,exist_ok=True);Path(p).write_text(json.dumps(d,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
def changed(s,a,b):
    assert s.count(a)==1,(a[:100],s.count(a));return s.replace(a,b)

def adapted_source():
    p=MAIN/'unified_trial/simulation.py';s=p.read_text();tree=ast.parse(s);lines=s.splitlines(True)
    wanted=['_create_environments','_prepare_plant','_load_commands','run_trial'];parts={n.name:''.join(lines[n.lineno-1:n.end_lineno]) for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in wanted}
    e=parts['_create_environments']
    a='            height_rows, height_columns = height.shape\n'
    e=changed(e,a,'''            full_height = height
            ny_full, nx_full = height.shape
            ax_full = (np.arange(nx_full) - (nx_full - 1) / 2) * spacing
            ay_full = (np.arange(ny_full) - (ny_full - 1) / 2) * spacing
            bx0, bx1, by0, by1 = experiment["terrain_roi_m"]
            ix = np.flatnonzero((ax_full >= bx0-1e-10) & (ax_full <= bx1+1e-10))
            iy = np.flatnonzero((ay_full >= by0-1e-10) & (ay_full <= by1+1e-10))
            np.testing.assert_allclose([ax_full[ix[0]], ax_full[ix[-1]], ay_full[iy[0]], ay_full[iy[-1]]], experiment["terrain_roi_m"], atol=1e-12, rtol=0)
            height = full_height[np.ix_(iy, ix)]
            height_rows, height_columns = height.shape
''')
    e=changed(e,'(np.arange(height_columns) - (height_columns - 1) / 2) * spacing,','ax_full[ix],')
    e=changed(e,'(np.arange(height_rows) - (height_rows - 1) / 2) * spacing,','ay_full[iy],')
    e=changed(e,'pose = common.initial(row, height, spacing)','pose = common.initial(row, full_height if height is not None else None, spacing)')
    parts['_create_environments']=e
    p=parts['_prepare_plant'];start=p.index('    with np.load(experiment["baseline_backend"]');end=p.index('    baseline_readback =',start)
    block=p[start:end].replace('[source_ids]','[reference_ids]')
    p=p[:start]+'    reference_ids = [row["baseline_source_id"] for row in rows]\n'+block+p[end:]
    start=p.index('    baseline_readback =');end=p.index('    np.testing.assert_allclose(\n        np.array(initial_poses)',start)
    p=p[:start]+'''    # New terrain placement is a declared generated expectation, not historical readback.
    expected_initial = np.array(experiment["planned_initial_pose"])[source_ids].copy()
    expected_initial[:, :3] += offsets
'''+p[end:]
    p=changed(p,'collision_enable_and_geometry_unchanged=True,','robot_collision_enable_and_geometry_unchanged=True,\n            terrain_changed_as_declared=True,')
    parts['_prepare_plant']=p
    l=parts['_load_commands'];l=changed(l,'[original_commands[i] for i in unique_ids]','[original_commands[contract_rows[i]["baseline_source_id"]] for i in unique_ids]');parts['_load_commands']=l
    r=parts['run_trial'];r=changed(r,'Run one original 12 s episode and write its readbacks and recorded telemetry.','Run one declared 6 s terrain preview; retain original numerical/controller code.')
    needle='        if (step_index + 1) % recording_stride == 0:\n'
    r=changed(r,needle,'''        # A cropped collision mesh is valid only while every body stays well inside.
        local_xy = robot.data.body_link_pose_w[:, :, :2].detach().cpu().numpy() - offsets[:, None, :2]
        bx0, bx1, by0, by1 = experiment["terrain_roi_m"]
        boundary_margin = min(float(local_xy[:,:,0].min()-bx0), float(bx1-local_xy[:,:,0].max()), float(local_xy[:,:,1].min()-by0), float(by1-local_xy[:,:,1].max()))
        if boundary_margin < 0.015:
            common.write(run_directory / "BOUNDARY_FAILURE.json", dict(t=(step_index+1)*args.dt, minimum_body_center_margin_m=boundary_margin))
            raise RuntimeError("Body entered 15 mm crop-boundary guard; this preview cannot be accepted")
'''+needle)
    parts['run_trial']=r
    return '# Narrow terrain-preview adaptations; controller, gains, solver and stepping preserved.\n\n'+'\n\n'.join(parts[k] for k in wanted)

def prepare():
    assert not (SIM/'campaign.json').exists(),'Preserve prepared campaign'
    report=read(OUT/'REPORT.json');assert report['preview_complete']
    campaign=read(MAIN/'campaign.json');base_rows=read(MAIN/'rows.json');pd=read(MAIN/'pd_wave4_firm20.json')['absolute_joint_pd']
    flat_jobs=[j for j in campaign['jobs'] if j['name'].startswith('flat')];cells=sum([j['cells'] for j in flat_jobs],[])
    assert [c['wave'] for c in cells]==[1.5,2.,2.5,3.,3.5,4.]
    assert sha(MAIN/'pd_wave4_firm20.json')==campaign['fixed_pd_sha256']
    for p,h in campaign['execution_and_input_pins'].items():assert sha(p)==h,p
    SIM.mkdir(exist_ok=True);snapshot=SIM/'executed_source';snapshot.mkdir()
    for p in [Path(__file__),ROOT/'shortwave_terrain.py',MAIN/'trial_profile.json',MAIN/'pd_wave4_firm20.json',*sorted((MAIN/'unified_trial').glob('*.py'))]:
        (snapshot/p.name).write_bytes(p.read_bytes())
    (snapshot/'adapted_functions.py').write_text(adapted_source(),encoding='utf-8')
    jobs=[]
    for level in LEVELS:
        for site in SITES:
            surface=report['sites'][site]['levels'][level];assert sha(surface['path'])==surface['sha256']
            z=np.load(surface['path']);h=z['height'];dx=float(z['dx_m']);rows=[];initials=[]
            for c in cells:
                i=c['source_id'];row=copy.deepcopy(base_rows[i]);row.update(id=f'{site}_{level}_w{c["wave"]:g}',terrain='rough',surface_file=surface['path'],surface_sha256=surface['sha256'],terrain_family=1 if site=='libo' else 2,terrain_seed=1501,start_index=0,start_xy=[.12,-.12],start_yaw_deg=0.,baseline_source_id=i,restoration_level=level,terrain_site=site)
                assert row['absolute_joint_pd']==pd and row['family_multipliers']=={}
                # Exact legacy initial() rule, computed independently before Isaac startup.
                pose=np.array(row['matched_initial_root_pose'],float);pose[:2]=row['start_xy'];ny,nx=h.shape;ix=int(pose[0]/dx+(nx-1)/2);iy=int(pose[1]/dx+(ny-1)/2);radius=int(np.ceil(.075/dx))
                pose[2]+=h[max(0,iy-radius):min(ny,iy+radius+1),max(0,ix-radius):min(nx,ix+radius+1)].max()+.002
                rows.append(row);initials.append(pose.tolist())
            rowfile=SIM/f'{site}_{level}_rows.json';dump(rowfile,rows)
            for group,ids,oldjob in [('low',[0,1,2],flat_jobs[0]),('high',[3,4,5],flat_jobs[1])]:
                name=f'{site}_{level}_{group}';plan=read(oldjob['args']['experiment_plan']);selected=[dict(cells[i],source_id=i,baseline_source_id=cells[i]['source_id'],surface=f'{site}_{level}') for i in ids]
                plan.update(schema='shortwave_preview_v1',run_name=name,duration_s=DURATION,benchmark_duration_s=DURATION,selected_environments=ids,benchmark_source_ids=ids,
                    rows_sha256=sha(rowfile),pd_overrides={str(c['source_id']):pd for c in cells},planned_initial_pose=initials,terrain_roi_m=ROI,
                    body_factors=[c['body_factor'] for c in selected],leg_cluster_factors=[c['leg_factor'] for c in selected],joint_cells=selected,
                    quality_offsets_m=[[0.,0.,0.]]*3,episode1_permutation=[0,1,2],worker_filename=Path(__file__).name,
                    campaign_note='User-approved short preview. Reuse frozen flat-cluster amplitudes and all173 wave4 PD. No amplitude optimization, convergence, or capacity search.',
                    quality_protocol=dict(duration_s=DURATION,comparison='Terrain intervention preview only',only_focal_poses_recorded=False),
                    scaling_benchmark=dict(backend='cpu',replicas=3,independent_trials=False,warmup_s=.1,measured_s=DURATION-.1,all_replicas_world_pitch_front5=True,contact_force_logging=False,all_replicas_recorded=True))
                planfile=SIM/f'{name}_plan.json';dump(planfile,plan);args=copy.deepcopy(oldjob['args']);args.update(rows=str(rowfile),name=name,duration=DURATION,experiment_plan=str(planfile),experiment_plan_sha256=sha(planfile))
                jobs.append(dict(name=name,site=site,level=level,group=group,args=args,source_ids=ids,cells=selected,rows_sha256=sha(rowfile)))
    pins={str(p):sha(p) for p in [Path(__file__),ROOT/'shortwave_terrain.py',OUT/'REPORT.json',MAIN/'campaign.json',MAIN/'rows.json',MAIN/'trial_profile.json',MAIN/'pd_wave4_firm20.json',*sorted((MAIN/'unified_trial').glob('*.py')),snapshot/'adapted_functions.py']}
    dump(SIM/'campaign.json',dict(jobs=jobs,pins=pins,profile=campaign['profile'],profile_sha256=campaign['profile_sha256'],fixed_pd_sha256=campaign['fixed_pd_sha256'],duration_s=DURATION,terrain_roi_m=ROI,workers=4,threads_per_worker=2,environments_per_worker=3,batch_order=LEVELS,worker_order=['libo_low','libo_high','narvik_low','narvik_high'],flat_cluster_cells=cells,scope='36 six-second previews, no amplitude optimization or new roughness sweep; original controls and 4/2 mm restorations',initialization='Original initial() rule on each full terrain, conservative maximum clearance; height can differ by terrain',render_backend='Saved actual Isaac poses, original USD geometry, VTK OpenGL; not native RTX'))
    check();print('PREPARED 36 conditions, 3 batches x 4 workers x 3 actual conditions',flush=True)

def check():
    c=read(SIM/'campaign.json')
    for p,h in c['pins'].items():assert sha(p)==h,p
    for j in c['jobs']:
        assert sha(j['args']['rows'])==j['rows_sha256'];assert sha(j['args']['experiment_plan'])==j['args']['experiment_plan_sha256']
        rows=read(j['args']['rows']);pd=read(MAIN/'pd_wave4_firm20.json')['absolute_joint_pd'];assert all(r['absolute_joint_pd']==pd for r in rows)
        p=read(j['args']['experiment_plan']);assert p['dt_s']==.0005 and p['control_dt_s']==.001 and p['feedback_dt_s']==.0005
        for i,cell in zip(j['source_ids'],j['cells']):
            expected=c['flat_cluster_cells'][i];assert all(cell[k]==expected[k] for k in ['wave','A_deg','B_deg','body_factor','leg_factor'])
        for r in rows:assert sha(r['surface_file'])==r['surface_sha256']
    print('INPUT CHECKS PASS: full PD, unchanged flat-cluster A/B, terrain hashes, timing, source pins',flush=True)

def worker(name,attempt):
    c=read(SIM/'campaign.json');j=next(j for j in c['jobs'] if j['name']==name)
    for p,h in c['pins'].items():assert sha(p)==h,p
    args=argparse.Namespace(**j['args']);assert sha(args.rows)==j['rows_sha256'] and sha(args.experiment_plan)==args.experiment_plan_sha256
    rows=read(args.rows);plan=read(args.experiment_plan);D=SIM/'runs'/attempt/name;D.mkdir(parents=True,exist_ok=False)
    log=(D/'worker.log').open('w',encoding='utf-8');sys.stdout.flush();sys.stderr.flush();os.dup2(log.fileno(),sys.stdout.fileno());os.dup2(log.fileno(),sys.stderr.fileno())
    active=dict(pid=os.getpid(),job=name,status='starting',started_unix=time.time());dump(D/'active.json',active);exitcode=1
    try:
        os.environ['WARP_CACHE_PATH']=str(D/'warp_cache');sys.path.insert(0,str(CON/'open_loop_review_2026-09-16'));sys.path.insert(0,str(NI/'work/runtime_speed_2026-09-17/deps'))
        import warp as wp
        assert wp.__version__=='1.8.1';wp.init()
        from isaaclab.app import AppLauncher
        launcher=AppLauncher(args);assert launcher.app is not None
        import torch
        torch.set_num_threads(1);torch.set_num_interop_threads(1)
        sys.path.insert(0,str(CON/'terrain_surface_library_2026-09-15'));sys.path.insert(0,str(MAIN))
        from unified_trial import simulation
        patch=SIM/'executed_source/adapted_functions.py';exec(compile(patch.read_text(),str(patch),'exec'),simulation.__dict__)
        from types import SimpleNamespace
        runtime=SimpleNamespace(args=args,experiment=plan,rows=rows,input_receipt=dict(input_rows_sha256=j['rows_sha256'],profile_sha256=c['profile_sha256'],fixed_pd_sha256=c['fixed_pd_sha256'],full_table_exact=True,flat_cluster_amplitudes_unchanged=True,initial_reference='Generated expectation using original terrain-placement rule'),run_directory=D,execution_sources=c['pins'])
        active['status']='running';dump(D/'active.json',active);simulation.run_trial(runtime)
        result=read(D/'results.json');assert result['complete'] and result['whole_batch_finite'] and result['actual_duration_s']==DURATION
        assert not read(D/'native_error_audit.json')['errors']
        z=np.load(D/'trajectory.npz');xy=z['pose'][...,:2];margins=[float(xy[...,0].min()-ROI[0]),float(ROI[1]-xy[...,0].max()),float(xy[...,1].min()-ROI[2]),float(ROI[3]-xy[...,1].max())];assert min(margins)>.015
        dump(D/'crop_guard.json',dict(terrain_roi_m=ROI,all_step_guard_m=.015,saved_pose_margins_m=margins,passed=True,interior_geometry='Exact vertex-aligned subset of full surface; original diagonals preserved'))
        active['status']='finished';exitcode=0
    except BaseException:active['status']='failed';traceback.print_exc()
    finally:active.update(exit_code=exitcode,finished_unix=time.time());dump(D/'active.json',active);sys.stdout.flush();sys.stderr.flush();log.flush()
    os._exit(exitcode)

def run(attempt):
    import psutil
    check();assert not (SIM/'runs'/attempt).exists(),'Attempt already exists; do not rerun or overwrite'
    assert psutil.virtual_memory().available>10*1024**3,'Insufficient available memory for fixed four-worker preview'
    started=time.time();states=[]
    for level in LEVELS:
        procs=[]
        for j in [j for j in read(SIM/'campaign.json')['jobs'] if j['level']==level]:
            logpath=SIM/f'{attempt}_{j["name"]}_launch.log';log=logpath.open('wb')
            p=subprocess.Popen([sys.executable,'-B',str(Path(__file__)), 'worker','--job',j['name'],'--attempt',attempt],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW);log.close();procs.append((j['name'],p));time.sleep(2)
        while True:
            current=[]
            for name,p in procs:
                try:a=read(SIM/'runs'/attempt/name/'active.json')
                except (FileNotFoundError,json.JSONDecodeError):a={}
                current.append(dict(job=name,pid=p.pid,exit=p.poll(),status=a.get('status'),rss_gib=psutil.Process(p.pid).memory_info().rss/1024**3 if p.poll() is None else None))
            receipt=dict(attempt=attempt,level=level,started_unix=started,elapsed_s=time.time()-started,available_memory_gib=psutil.virtual_memory().available/1024**3,finished=states,current=current,complete=False)
            dump(SIM/f'supervisor_{attempt}.json',receipt)
            if all(p.poll() is not None for _,p in procs):break
            time.sleep(15)
        states.extend(current)
        print(json.dumps(dict(level=level,workers=current,elapsed_s=time.time()-started)),flush=True)
        if not all(p.poll()==0 for _,p in procs):raise RuntimeError('A worker failed; later batches not launched; no automatic retry')
    receipt.update(complete=True,finished=states,current=[],elapsed_s=time.time()-started);dump(SIM/f'supervisor_{attempt}.json',receipt);print('ALL 36 PREVIEWS COMPLETE',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','check','run','worker']);p.add_argument('--attempt',default='v1');p.add_argument('--job');a=p.parse_args()
    if a.mode=='prepare':prepare()
    elif a.mode=='check':check()
    elif a.mode=='run':run(a.attempt)
    else:worker(a.job,a.attempt)
