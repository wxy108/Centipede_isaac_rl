"""84 fixed-cutoff, variable-height Isaac previews; original controller and PD."""
from pathlib import Path
import os
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:
    os.environ[k]='1'
os.environ['PXR_WORK_THREAD_LIMIT']='2'
import sys,json,hashlib,copy,argparse,subprocess,time
import numpy as np
import run_shortwave as engine

ROOT=Path(__file__).resolve().parent
MAIN=ROOT.parent
OUT=ROOT/'height_amplitude_4mm'
SIM=OUT/'simulation'
SCALES=[.25,.5,.75,1.,1.25,1.5,2.]
def level_for(s):return 'height_x'+f'{s:.2f}'.replace('.','p')
LEVELS=[level_for(s) for s in SCALES]
SITES=['libo','narvik']
DURATION=6.
ROI=engine.ROI
read,sha,dump=engine.read,engine.sha,engine.dump
ENGINE_PATH=ROOT/'run_shortwave.py'
engine.OUT=OUT;engine.SIM=SIM;engine.LEVELS=LEVELS
# Reused functions resolve the new campaign and launch this wrapper's worker mode.
engine.__file__=str(Path(__file__).resolve())


def prepare():
    assert not (SIM/'campaign.json').exists(),'Preserve prepared campaign'
    terrains=read(OUT/'REPORT.json');assert terrains['complete'] and terrains['height_scales']==SCALES
    surfaces={(r['site'],r['height_scale']):r for r in terrains['terrains']}
    original=read(MAIN/'campaign.json');base_rows=read(MAIN/'rows.json');pd=read(MAIN/'pd_wave4_firm20.json')['absolute_joint_pd']
    flat_jobs=[j for j in original['jobs'] if j['name'].startswith('flat')]
    cells=sum([j['cells'] for j in flat_jobs],[])
    assert [c['wave'] for c in cells]==[1.5,2.,2.5,3.,3.5,4.]
    assert sha(MAIN/'pd_wave4_firm20.json')==original['fixed_pd_sha256']
    for p,h in original['execution_and_input_pins'].items():assert sha(p)==h,p
    SIM.mkdir(exist_ok=False);snapshot=SIM/'executed_source';snapshot.mkdir()
    sources=[Path(__file__),ENGINE_PATH,ROOT/'height_amplitude_preview.py',MAIN/'trial_profile.json',MAIN/'pd_wave4_firm20.json',*sorted((MAIN/'unified_trial').glob('*.py'))]
    for p in sources:(snapshot/p.name).write_bytes(p.read_bytes())
    adapted=engine.adapted_source()
    previous=ROOT/'shortwave/simulation_v2/executed_source/adapted_functions.py'
    assert adapted==previous.read_text(encoding='utf-8'),'Numerical/controller adaptation changed'
    (snapshot/'adapted_functions.py').write_text(adapted,encoding='utf-8')
    jobs=[]
    for scale,level in zip(SCALES,LEVELS):
        for site in SITES:
            surface=surfaces[site,scale];assert sha(surface['path'])==surface['sha256']
            with np.load(surface['path']) as z:h=z['height'];dx=float(z['dx_m'])
            assert h.shape==(2045,2045) and dx==.0005
            rows=[];initials=[]
            for c in cells:
                i=c['source_id'];row=copy.deepcopy(base_rows[i])
                row.update(id=f'{site}_{level}_w{c["wave"]:g}',terrain='rough',surface_file=surface['path'],surface_sha256=surface['sha256'],
                    terrain_family=1 if site=='libo' else 2,terrain_seed=1501,start_index=0,start_xy=[.12,-.12],start_yaw_deg=0.,
                    baseline_source_id=i,restoration_level='restore4',terrain_site=site,height_scale=scale,
                    fixed_pair_angle_rms_deg=surface['fixed_pair_angle_rms_deg'])
                assert row['absolute_joint_pd']==pd and row['family_multipliers']=={}
                pose=np.array(row['matched_initial_root_pose'],float);pose[:2]=row['start_xy']
                ny,nx=h.shape;ix=int(pose[0]/dx+(nx-1)/2);iy=int(pose[1]/dx+(ny-1)/2);radius=int(np.ceil(.075/dx))
                pose[2]+=h[max(0,iy-radius):min(ny,iy+radius+1),max(0,ix-radius):min(nx,ix+radius+1)].max()+.002
                rows.append(row);initials.append(pose.tolist())
            rowfile=SIM/f'{site}_{level}_rows.json';dump(rowfile,rows)
            for group,ids,oldjob in [('low',[0,1,2],flat_jobs[0]),('high',[3,4,5],flat_jobs[1])]:
                name=f'{site}_{level}_{group}';plan=read(oldjob['args']['experiment_plan'])
                selected=[dict(cells[i],source_id=i,baseline_source_id=cells[i]['source_id'],surface=f'{site}_{level}',height_scale=scale,
                               fixed_pair_angle_rms_deg=surface['fixed_pair_angle_rms_deg']) for i in ids]
                plan.update(schema='height_amplitude_preview_v1',run_name=name,duration_s=DURATION,benchmark_duration_s=DURATION,
                    selected_environments=ids,benchmark_source_ids=ids,rows_sha256=sha(rowfile),pd_overrides={str(c['source_id']):pd for c in cells},
                    planned_initial_pose=initials,terrain_roi_m=ROI,body_factors=[c['body_factor'] for c in selected],
                    leg_cluster_factors=[c['leg_factor'] for c in selected],joint_cells=selected,quality_offsets_m=[[0.,0.,0.]]*3,
                    episode1_permutation=[0,1,2],worker_filename=Path(__file__).name,
                    campaign_note='User-authorized six-second height sweep. Fixed 4 mm cutoff. Frozen flat-cluster gait amplitudes and all173 wave4 PD.',
                    quality_protocol=dict(duration_s=DURATION,comparison='Fixed-cutoff terrain-height intervention',only_focal_poses_recorded=False),
                    scaling_benchmark=dict(backend='cpu',replicas=3,independent_trials=False,warmup_s=.1,measured_s=DURATION-.1,
                                           all_replicas_world_pitch_front5=True,contact_force_logging=False,all_replicas_recorded=True))
                planfile=SIM/f'{name}_plan.json';dump(planfile,plan)
                args=copy.deepcopy(oldjob['args']);args.update(rows=str(rowfile),name=name,duration=DURATION,experiment_plan=str(planfile),experiment_plan_sha256=sha(planfile))
                jobs.append(dict(name=name,site=site,level=level,height_scale=scale,group=group,args=args,source_ids=ids,cells=selected,rows_sha256=sha(rowfile)))
    pins={str(p):sha(p) for p in [*sources,OUT/'REPORT.json',MAIN/'campaign.json',MAIN/'rows.json',snapshot/'adapted_functions.py']}
    dump(SIM/'campaign.json',dict(jobs=jobs,pins=pins,profile=original['profile'],profile_sha256=original['profile_sha256'],
        fixed_pd_sha256=original['fixed_pd_sha256'],duration_s=DURATION,terrain_roi_m=ROI,workers=4,threads_per_worker=2,
        environments_per_worker=3,batch_order=LEVELS,height_scales=SCALES,worker_order=['libo_low','libo_high','narvik_low','narvik_high'],
        flat_cluster_cells=cells,condition_count=84,scope='84 short actual simulations, 4 mm cutoff, seven height scales, six waves, two sites; no optimization or convergence/capacity search',
        initialization='Original initial() rule on each full terrain, conservative maximum clearance; height can differ by terrain',
        adapted_physics_source_identical_to=str(previous),render_backend='Saved actual Isaac poses, original USD geometry, VTK OpenGL; not native RTX'))
    engine.check()
    print('PREPARED: 84 conditions, 7 batches x 4 workers x 3 actual conditions; previous stepping/controller source identical.',flush=True)


def run(attempt):
    import psutil
    engine.check();assert not (SIM/'runs'/attempt).exists(),'Attempt exists; preserve all results'
    assert psutil.virtual_memory().available>10*1024**3,'Insufficient memory for fixed four-worker allocation'
    started=time.time();states=[]
    for level in LEVELS:
        procs=[]
        for j in [j for j in read(SIM/'campaign.json')['jobs'] if j['level']==level]:
            logpath=SIM/f'{attempt}_{j["name"]}_launch.log'
            with logpath.open('wb') as log:
                p=subprocess.Popen([sys.executable,'-B',str(Path(__file__)), 'worker','--job',j['name'],'--attempt',attempt],
                    cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            procs.append((j['name'],p));time.sleep(2)
        while True:
            current=[]
            for name,p in procs:
                try:a=read(SIM/'runs'/attempt/name/'active.json')
                except (FileNotFoundError,json.JSONDecodeError):a={}
                rss=None
                if p.poll() is None:
                    try:rss=psutil.Process(p.pid).memory_info().rss/1024**3
                    except psutil.NoSuchProcess:pass
                current.append(dict(job=name,pid=p.pid,exit=p.poll(),status=a.get('status'),rss_gib=rss))
            receipt=dict(attempt=attempt,level=level,started_unix=started,elapsed_s=time.time()-started,
                available_memory_gib=psutil.virtual_memory().available/1024**3,finished=states,current=current,complete=False)
            dump(SIM/f'supervisor_{attempt}.json',receipt)
            if all(p.poll() is not None for _,p in procs):break
            time.sleep(15)
        states.extend(current)
        print(json.dumps(dict(level=level,workers=current,elapsed_s=time.time()-started)),flush=True)
        if not all(p.poll()==0 for _,p in procs):raise RuntimeError('Worker failure: later batches not launched; inspect evidence before retry')
    receipt.update(complete=True,finished=states,current=[],elapsed_s=time.time()-started)
    dump(SIM/f'supervisor_{attempt}.json',receipt);print('ALL 84 HEIGHT-SWEEP PREVIEWS COMPLETE',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','check','run','worker']);p.add_argument('--attempt',default='v1');p.add_argument('--job')
    args=p.parse_args()
    if args.mode=='prepare':prepare()
    elif args.mode=='check':engine.check()
    elif args.mode=='run':run(args.attempt)
    else:engine.worker(args.job,args.attempt)
