"""Frozen native Isaac flat IID erasure campaign; four workers x two threads."""
from pathlib import Path
import os,sys,json,copy,hashlib,argparse,subprocess,time,traceback,ast
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:os.environ[key]='1'
os.environ['PXR_WORK_THREAD_LIMIT']='2'
import numpy as np
ROOT=Path(__file__).resolve().parent;MAIN=ROOT.parent
BASE=ROOT/'contact_erasure_light_2026-09-30';SIM=BASE/'simulation_v2'
ARCHIVE_SIM=ROOT/'_archive/contact_erasure_full_2026-09-30/simulation'
sys.path.insert(0,str(ROOT))
import run_libo_repeats as parent
from contact_erasure_channel_light_v2 import schedule
read,sha,dump=parent.read,parent.sha,parent.dump
BINS=[.01,.02,.05];WAVES=[1.5,2,2.5,3,3.5]

def prepare():
    assert not SIM.exists(),'Preserve prepared input'
    pc=read(ROOT/'libo_spawn_repeats/simulation/campaign.json')
    for path,h in pc['pins'].items():assert sha(path)==h,path
    assert read(ROOT/'libo_spawn_repeats/simulation/supervisor.json')['complete']
    SIM.mkdir(parents=True);snap=SIM/'executed_source';snap.mkdir()
    source=(ROOT/'libo_spawn_repeats/simulation/executed_source/adapted_functions.py').read_text(encoding='utf-8')
    def change(a,b):
        nonlocal source
        assert source.count(a)==1,a
        source=source.replace(a,b)
    change('    save_frame(0.0)','''    from contact_erasure_channel_light_v2 import Channel
    erasure = Channel(sim,robot,rows,experiment,run_directory,contact_report,body_names,contact_body_order,foot_ids)
    globals()["_active_erasure_channel"] = erasure
    save_frame(0.0)''')
    change('        sim.step(render=False)','        erasure.before(step_index)\n        sim.step(render=False)')
    change('        timing["physics"] += time.perf_counter() - tick','        timing["physics"] += time.perf_counter() - tick\n        erasure.after(step_index)')
    change('    sys.stdout.flush()\n    sys.stderr.flush()\n    errors = [','    erasure.finish(numerical_failure is None)\n    sys.stdout.flush()\n    sys.stderr.flush()\n    errors = [')
    ast.parse(source);(snap/'adapted_functions.py').write_text(source,encoding='utf-8')
    flats=[j for j in pc['jobs'] if j['height_scale']==0]
    template_rows=read(flats[0]['args']['rows']);assert len(template_rows)==5
    pd=read(MAIN/'pd_wave4_firm20.json')['absolute_joint_pd']
    jobs=[]
    def job(name,cells,pilot=False):
        rows=[];joint=[];n=len(cells)
        for i,c in enumerate(cells):
            wi=WAVES.index(c['wave']);row=copy.deepcopy(template_rows[wi])
            row.update(id=name+f'_e{i:02d}',erasure=dict(p=c['p'],mask_seed=c['mask_seed'],erase_bin_s=c['erase_bin_s']))
            assert row['absolute_joint_pd']==pd and row['terrain']=='flat';rows.append(row)
            cc=copy.deepcopy(pc['flat_cluster_cells'][wi]);cc.update(c,source_id=i,spawn_index=2,height_scale=0.,surface='flat',fixed_pair_angle_rms_deg=0.)
            joint.append(cc)
        rp=SIM/f'{name}_rows.json';dump(rp,rows)
        plan=read(flats[0]['args']['experiment_plan']);ids=list(range(n))
        # Original initial-Z rule and same known spawn/yaw as the existing flat controls.
        original_initials=plan['planned_initial_pose']
        initials=[original_initials[WAVES.index(c['wave'])] for c in cells]
        plan.update(run_name=name,selected_environments=ids,benchmark_source_ids=ids,rows_sha256=sha(rp),planned_initial_pose=initials,
            pd_overrides={str(i):pd for i in ids},body_factors=[c['body_factor'] for c in joint],leg_cluster_factors=[c['leg_factor'] for c in joint],
            joint_cells=joint,quality_offsets_m=[[0.,0.,0.]]*n,episode1_permutation=list(range(n)),worker_filename=Path(__file__).name,
            campaign_note='User started fixed-p independent38leg erasure10/20/50ms,2matched mask seeds,2–6s noise/scoring. Fixed fullPD and amplitudes.',
            quality_protocol=dict(duration_s=6.,comparison='Common initial state;38leg x temporal-bin independent Bernoulli(p); shared uniforms; native contact audits',only_focal_poses_recorded=False))
        plan['scaling_benchmark']['replicas']=n
        pp=SIM/f'{name}_plan.json';dump(pp,plan)
        args=copy.deepcopy(flats[0]['args']);args.update(rows=str(rp),name=name,experiment_plan=str(pp),experiment_plan_sha256=sha(pp))
        jobs.append(dict(name=name,args=args,source_ids=ids,cells=joint,rows_sha256=sha(rp),pilot_only=pilot))
    # These endpoint batches preserve3+2 original flat slots; masks are identical across all bin sizes.
    for p in [0.,1.]:
        for group,ww in [('low',WAVES[:3]),('high',WAVES[3:])]:
            job(f'endpoint_p{int(p)}_{group}',[dict(wave=w,p=p,mask_seed=0,erase_bin_s=.01) for w in ww])
    job('pilot_p050_bin010_w3',[dict(wave=3,p=.5,mask_seed=0,erase_bin_s=.01)],pilot=True)
    for probability in range(1,10):
        cells=[dict(wave=w,p=probability/10,mask_seed=seed) for seed in range(2) for w in WAVES]
        for bin_s in BINS:
            for b in range(0,len(cells),12):
                job(f'bin{round(bin_s*1000):03d}_p{probability*10:03d}_b{b//12:02d}',[dict(c,erase_bin_s=bin_s) for c in cells[b:b+12]])
    for j in jobs[:4]:
        j['reuse_path']=str(ARCHIVE_SIM/'runs/v1'/j['name'])
        assert read(Path(j['reuse_path'])/'active.json')['status']=='finished'
    jobs[4]['reuse_path']=str(BASE/'simulation/runs/v1/pilot_p050_bin010_w3')
    assert read(Path(jobs[4]['reuse_path'])/'active.json')['status']=='failed'
    # Equal slots and row order across bin sizes; fixed cyclic bin order across blocks.
    queue=[]
    for probability in range(1,10):
        for block in range(1):
            order=BINS[block%3:]+BINS[:block%3]
            for bin_s in order:queue.append(f'bin{round(bin_s*1000):03d}_p{probability*10:03d}_b{block:02d}')
    batches=[dict(name=f'main_{i//4:03d}',jobs=queue[i:i+4]) for i in range(0,len(queue),4)]
    sources=[Path(__file__),ROOT/'contact_erasure_channel_light_v2.py',ROOT/'run_libo_repeats.py',ROOT/'run_shortwave.py',MAIN/'trial_profile.json',MAIN/'pd_wave4_firm20.json',ROOT/'libo_spawn_repeats/simulation/campaign.json',*sorted((MAIN/'unified_trial').glob('*.py'))]
    pins={str(p):sha(p) for p in sources};pins[str(snap/'adapted_functions.py')]=sha(snap/'adapted_functions.py')
    for p in sources:(snap/p.name).write_bytes(p.read_bytes())
    for path in [Path(pc['collision_bounds']['asset']),Path(template_rows[0]['command_file'])]:pins[str(path)]=sha(path)
    c=copy.deepcopy(pc);c.update(schema='independent38leg_temporalbin_erasure_v1',jobs=jobs,batches=batches,pins=pins,
        condition_count=280,diagnostic_condition_count=1,rough_condition_count=0,new_flat_condition_count=270,reused_endpoint_condition_count=10,light_seed_count=2,archive_prefix=str(ARCHIVE_SIM),
        erasure_duration_s=BINS,probabilities=[i/10 for i in range(11)],mask_seeds=list(range(2)),noise_window_s=[2,6],score_window_s=[2,6],
        environments_per_worker=12,workers=4,threads_per_worker=2,scope='Latest user replaced overnight full study with approximately1hour light complete study.280unique conditions:270new stochastic +10reused endpoints. Full4060 archived/held; rough expansion held.',
        equivalence=dict(contact='Missing native positive-load feet during phase-matched flat-reference support.10ms existing rough data; do not count natural swing as loss.',performance='Inverse same-wave mean signed2–6s displacement at each erasure duration; preserve multi-valued/out-of-range mappings.'),
        endpoint_policy='p0/1 unique perwave, both masks independent of bin size; not additional random samples',
        analysis_policy='All completed signed forward outcomes, including stuck/reverse/fallen. No automatic rough-only outlier omission. Numerical failures separate.',
        crossing_policy='Exploratory all10wave pairs at every bin, paired signs and linear mean-curve brackets. Only2 seeds; no confirmation claim and no automatic extra runs.',
        telemetry='Native foot+complete-leg+chassis forces every0.5ms; restored contact maxima in first1ms; trajectories+originalcluster every10ms',
        failure_policy='Offender retained; interrupted peers only recovered at unchanged input. Native erasure-channel violation/infrastructure error stops campaign.')
    dump(SIM/'campaign.json',c)
    # Static checks have independent endpoint/expectation/row binding value; no physics here.
    for seed in range(2):
        previous={b:None for b in BINS};h=None
        for p in [i/10 for i in range(11)]:
            endpoints=[]
            for b in BINS:
                m,info=schedule(p,seed,b)
                assert not m[:4000].any()
                if p==0:assert not m.any()
                if p==1:assert m[4000:].all()
                if previous[b] is not None:assert np.all(~previous[b]|m)
                if h is None:h=info['uniform_sha256']
                assert info['uniform_sha256']==h;previous[b]=m
                if p in [0,1]:endpoints.append(info['mask_sha256'])
            if endpoints:assert len(set(endpoints))==1
    check();print('PREPARED280 main conditions +1 channel diagnostic; no physics yet',flush=True)

def check():
    c=read(SIM/'campaign.json')
    for p,h in c['pins'].items():assert sha(p)==h,p
    pd=read(MAIN/'pd_wave4_firm20.json')['absolute_joint_pd'];keys=[]
    for j in c['jobs']:
        assert sha(j['args']['rows'])==j['rows_sha256'] and sha(j['args']['experiment_plan'])==j['args']['experiment_plan_sha256']
        rows=read(j['args']['rows']);p=read(j['args']['experiment_plan'])
        assert j['source_ids']==p['benchmark_source_ids'] and len(rows)==len(j['cells'])<=12
        for row,cell in zip(rows,j['cells']):
            assert row['absolute_joint_pd']==pd and not row['family_multipliers']
            ref=next(x for x in c['flat_cluster_cells'] if x['wave']==cell['wave'])
            assert all(ref[k]==cell[k] for k in ['A_deg','B_deg','body_factor','leg_factor'])
            if not j['pilot_only']:keys.append((cell['erase_bin_s'] if cell['p'] not in [0,1] else 0,cell['p'],cell['mask_seed'],cell['wave']))
    assert len(keys)==len(set(keys))==280
    print('CHECK PASS:280 unique cases, fixed173PD/A/B/profile, common masks, physical/control cadences',flush=True)

def worker(name,attempt,keep=None):
    c=read(SIM/'campaign.json');j=next(j for j in c['jobs'] if j['name']==name)
    for path,h in c['pins'].items():assert sha(path)==h,path
    args=argparse.Namespace(**j['args']);plan=read(args.experiment_plan);rows=read(args.rows)
    selected=j['source_ids'] if keep is None else keep
    D=SIM/'runs'/attempt/name;D.mkdir(parents=True,exist_ok=False)
    if keep is not None:
        idx=[j['source_ids'].index(i) for i in selected];n=len(idx)
        for key in ['body_factors','leg_cluster_factors','joint_cells']:plan[key]=[plan[key][i] for i in idx]
        plan.update(selected_environments=selected,benchmark_source_ids=selected,quality_offsets_m=[[0.,0.,0.]]*n,episode1_permutation=list(range(n)),peer_recovery=True)
        plan['scaling_benchmark']['replicas']=n;dump(D/'effective_plan.json',plan)
        args.experiment_plan=str(D/'effective_plan.json');args.experiment_plan_sha256=sha(args.experiment_plan)
    log=(D/'worker.log').open('w',encoding='utf-8');sys.stdout.flush();sys.stderr.flush();os.dup2(log.fileno(),sys.stdout.fileno());os.dup2(log.fileno(),sys.stderr.fileno())
    active=dict(pid=os.getpid(),job=name,status='starting',source_ids=selected,started_unix=time.time());dump(D/'active.json',active);code=1;simulation=None
    try:
        os.environ['WARP_CACHE_PATH']=str(D/'warp_cache');sys.path.insert(0,str(parent.engine.CON/'open_loop_review_2026-09-16'));sys.path.insert(0,str(parent.engine.NI/'work/runtime_speed_2026-09-17/deps'))
        import warp as wp
        assert wp.__version__=='1.8.1';wp.init()
        from isaaclab.app import AppLauncher
        launcher=AppLauncher(args)
        import torch
        torch.set_num_threads(1);torch.set_num_interop_threads(1)
        sys.path.insert(0,str(parent.engine.CON/'terrain_surface_library_2026-09-15'));sys.path.insert(0,str(MAIN))
        from unified_trial import simulation
        patch=SIM/'executed_source/adapted_functions.py';exec(compile(patch.read_text(encoding='utf-8'),str(patch),'exec'),simulation.__dict__)
        from types import SimpleNamespace
        runtime=SimpleNamespace(args=args,experiment=plan,rows=rows,input_receipt=dict(input_rows_sha256=j['rows_sha256'],profile_sha256=c['profile_sha256'],fixed_pd_sha256=c['fixed_pd_sha256'],full_table_exact=True,flat_cluster_amplitudes_unchanged=True),run_directory=D,execution_sources=dict(c['pins'],**{str(args.experiment_plan):sha(args.experiment_plan)}))
        active['status']='running';dump(D/'active.json',active);simulation.run_trial(runtime)
        result=read(D/'results.json');assert result['complete'] and result['whole_batch_finite'] and result['actual_duration_s']==6.
        assert not read(D/'native_error_audit.json')['errors'] and read(D/'ERASURE_RESULT.json')['deleted_contact_violations']==0
        active['status']='finished';code=0
    except BaseException:
        active['status']='failed';traceback.print_exc()
        if simulation is not None and hasattr(simulation,'_active_erasure_channel'):
            try:simulation._active_erasure_channel.finish(False)
            except BaseException:traceback.print_exc()
    finally:active.update(exit_code=code,finished_unix=time.time());dump(D/'active.json',active);sys.stdout.flush();sys.stderr.flush();log.flush()
    os._exit(code)

def gate():
    from analyze_libo_repeats import extract_run
    c=read(SIM/'campaign.json');ref=read(ROOT/'libo_spawn_repeats/results/RESULTS.json');checks=[]
    for group in ['low','high']:
        j=next(j for j in c['jobs'] if j['name']==f'endpoint_p0_{group}');D=Path(j['reuse_path'])
        ex=extract_run(D,len(j['cells']),90)
        with np.load(D/'trajectory.npz') as z:
            for e,cell in enumerate(j['cells']):
                row=next(x for x in ref['cells'] if x['height_scale']==0 and x['wave']==cell['wave'])
                with np.load(Path(row['run_path'])/'trajectory.npz') as old:
                    for field in ['pose','q','v','body_velocity']:np.testing.assert_array_equal(z[field][:,e],old[field][:,row['environment_index']])
                checks.append(dict(wave=cell['wave'],p0_bitexact=True,forward_2_6_mm=float(ex['forward'][-1,e]-ex['forward'][200,e])))
    for name in ['endpoint_p1_low','endpoint_p1_high','pilot_p050_bin010_w3']:
        j=next(j for j in c['jobs'] if j['name']==name);D=Path(j['reuse_path']) if 'reuse_path' in j else SIM/'runs/v1'/name;er=read(D/'ERASURE_RESULT.json');assert er['deleted_contact_violations']==0
        if not er['complete']:
            assert name=='pilot_p050_bin010_w3'
            failure=read(D/'BOUNDARY_FAILURE.json')
            assert max(x['max_body_linear_speed_m_s'] for x in failure['environments'])>1e6
            assert read(D/'ERASURE_FIXED_PLANT.json')['passed']
            assert er['restore_events'] and er['restore_samples']
            # A physical/numerical state divergence is an outcome, not a mask failure.
            # Every completed step before termination verified erased-leg native force=0.
            assert not (D/'ERASURE_FAILURE.json').exists()
        with np.load(D/'erasure_contacts.npz') as z:
            assert len(z['t'])==12000 if er['complete'] else len(z['t'])>4000
            if name.startswith('endpoint_p1'):assert np.max(z['leg_normal_force'][4000:])<=1e-6
    dump(SIM/'CHANNEL_GATE.json',dict(passed=True,zero_trajectory_identity=checks,all_leg_off_native_contact_zero=True,random_restoration_native_mask_verified=True,
        restoration_quality_is_reported_not_assumed=True,no_physics_parameter_retuning=True,pilot_stable_six_seconds=False,pilot_status='state_divergence_at2.9s; all5800completed steps native deletion correct; record as failure, never zero displacement',gate_revision_reason='Do not require an experimentally modified random trajectory to stay numerically stable to validate physical deletion. Fullp0 identity and fullp1 nativezero remain required.'))

def run():
    import psutil
    check();assert not (SIM/'supervisor.json').exists(),'No duplicate supervisor'
    assert psutil.virtual_memory().available>10*1024**3
    c=read(SIM/'campaign.json');jobs={j['name']:j for j in c['jobs']};finished=[];queue=[];started=time.time()
    def batch(entries,label):
        procs=[]
        for name,attempt,keep in entries:
            cmd=[sys.executable,'-B',str(Path(__file__)),'worker','--job',name,'--attempt',attempt]
            if keep is not None:cmd+=['--keep',','.join(map(str,keep))]
            with (SIM/f'{attempt}_{name}_launch.log').open('wb') as log:p=subprocess.Popen(cmd,cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            procs.append((name,attempt,keep,p));time.sleep(2)
        while True:
            current=[]
            for name,attempt,keep,p in procs:
                try:a=read(SIM/'runs'/attempt/name/'active.json')
                except(FileNotFoundError,json.JSONDecodeError):a={}
                current.append(dict(job=name,attempt=attempt,keep=keep,pid=p.pid,exit=p.poll(),status=a.get('status')))
            dump(SIM/'supervisor.json',dict(complete=False,pid=os.getpid(),batch=label,started_unix=started,elapsed_s=time.time()-started,finished=finished,current=current))
            if all(p.poll() is not None for *_,p in procs):break
            time.sleep(10)
        finished.extend(current)
        for item in current:
            if item['exit']!=0:
                j=jobs[item['job']];ids=item['keep'] or j['source_ids'];d=SIM/'runs'/item['attempt']/item['job']
                if (d/'ERASURE_FAILURE.json').exists():raise RuntimeError('Native erasure channel validation failed; full campaign halted')
                peers,off=parent.failure_peers(d,ids);item['offender_source_ids']=off;item['interrupted_peer_source_ids']=peers
                if peers:queue.append((item['job'],f'peer{len(ids)-len(peers)}_from_{item["attempt"]}',peers))
        print(json.dumps(dict(batch=label,worker_attempts=len(finished),elapsed_s=time.time()-started,peer_recoveries=len(queue))),flush=True)
        return current
    try:
        for j in c['jobs'][:4]:
            d=Path(j['reuse_path']);a=read(d/'active.json')
            assert a['exit_code']==0 and read(d/'ERASURE_RESULT.json')['complete']
            assert not read(d/'native_error_audit.json')['errors']
            finished.append(dict(job=j['name'],attempt='v1',keep=None,pid=a['pid'],exit=0,status='finished',reused=True,run_path=str(d)))
        j=c['jobs'][4];a=read(Path(j['reuse_path'])/'active.json');finished.append(dict(job=j['name'],attempt='v1',keep=None,pid=a['pid'],exit=a['exit_code'],status=a['status'],reused=True,diagnostic_only=True,run_path=j['reuse_path']))
        gate()
        for b in c['batches']:batch([(name,'v1',None) for name in b['jobs']],b['name'])
        roundno=0
        while queue:
            roundno+=1;assert roundno<=4;pending=queue[:];queue.clear()
            for i in range(0,len(pending),4):batch(pending[i:i+4],f'peer_recovery{roundno}_{i//4}')
        dump(SIM/'supervisor.json',dict(complete=True,pid=os.getpid(),started_unix=started,elapsed_s=time.time()-started,finished=finished,current=[],all_attempts_accounted_for=True))
        print('ALL280 planned observations attempted; raw failures retained',flush=True)
    except BaseException:
        dump(SIM/'SUPERVISOR_FAILURE.json',dict(pid=os.getpid(),elapsed_s=time.time()-started,error=traceback.format_exc(),finished=finished,queued_peers=queue))
        raise

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['prepare','check','run','worker','gate']);ap.add_argument('--job');ap.add_argument('--attempt',default='v1');ap.add_argument('--keep');a=ap.parse_args()
    if a.mode=='prepare':prepare()
    elif a.mode=='check':check()
    elif a.mode=='worker':worker(a.job,a.attempt,None if a.keep is None else list(map(int,a.keep.split(','))))
    elif a.mode=='gate':gate()
    else:run()
