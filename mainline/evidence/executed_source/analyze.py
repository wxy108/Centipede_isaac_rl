"""Verify completed run evidence and summarize motion without ranking amplitudes."""
import json,hashlib,sys,re,shutil
from pathlib import Path
import numpy as np
W=Path(__file__).resolve().parent;R=W.parents[1];O=R/'outputs/unified_demo_2026-09-28';O.mkdir(exist_ok=True)
attempt=sys.argv[1] if len(sys.argv)>1 else 'v4'
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
campaign=read(W/'campaign.json');profile=read(Path(campaign['profile']));pd=read(Path(profile['fixed_pd']['path']))['absolute_joint_pd']
evidence=[];conditions=[];reference_pd=None
for job in campaign['jobs']:
    D=W/'runs'/attempt/job['name'];result=read(D/'results.json');rb=read(D/'readback.json');th=read(D/'thread_settings.json');iters=read(D/'iteration_readback.json');isolation=read(D/'collision_group_readback.json')
    assert isolation['invert'] and isolation['global_paths']==[]
    assert len(isolation['groups'])==3
    for i,g in enumerate(isolation['groups']):assert g['includes']==[f'/World/env_{i}'] and g['allowed_groups']==[f'/World/collisions/group{i}']
    offsets=read(D/'native_contact_offsets.json')
    np.testing.assert_allclose(offsets['contact'],2e-5,rtol=0,atol=1e-11)
    np.testing.assert_allclose(offsets['rest'],0,rtol=0,atol=0)
    attrs=read(D/'effective_fields_after_pd.json')['attributes']
    fields={(a['path'],a['name']):a['value'] for a in attrs}
    for key,val in [('enableCCD','False'),('enableEnhancedDeterminism','False'),('enableStabilization','False'),('bounceThreshold','0.5'),('enableExternalForcesEveryIteration','True')]:
        assert fields[('/physicsScene','physxScene:'+key)]==val
    for a in attrs:
        if a['name'] in ['physxArticulation:sleepThreshold','physxArticulation:stabilizationThreshold']:assert a['value']=='0.0'
    for key in ['physics:staticFriction','physics:dynamicFriction']:assert fields[('/World/Material',key)]=='0.5'
    assert result['complete'] and result['whole_batch_finite'] and result['actual_duration_s']==12 and result['finite_check_interval_s']==.0005
    assert read(D/'native_error_audit.json')['errors']==[]
    assert rb['dt_s']==.0005 and rb['solver']=='TGS' and rb['gpu_dynamics'] is False
    assert th['tasking']==th['physics']==2 and th['torch_threads']==1
    assert all(i['position']==64 and i['velocity']==16 for i in iters)
    backend=np.load(D/'backend.npz');kp=np.array([pd[n][0] for n in rb['joint_names']],np.float32);kd=np.array([pd[n][1] for n in rb['joint_names']],np.float32)
    np.testing.assert_array_equal(backend['kp'],np.tile(kp,(3,1)));np.testing.assert_array_equal(backend['kd'],np.tile(kd,(3,1)))
    if reference_pd is None:reference_pd=(kp,kd,rb['joint_names'])
    else:
        assert reference_pd[2]==rb['joint_names'];np.testing.assert_array_equal(kp,reference_pd[0]);np.testing.assert_array_equal(kd,reference_pd[1])
    z=np.load(D/'trajectory.npz');t=z['t'];pose=z['pose'];assert pose.shape==(1201,3,251,7) and np.allclose(np.diff(t),.01) and t[-1]==12
    assert all(np.isfinite(z[k]).all() for k in ['pose','q','v','body_velocity'])
    c=np.load(D/'cluster_contacts.npz');assert c['force'].shape[:2]==(1200,3) and np.isfinite(c['force']).all()
    timing=read(D/'benchmark_timing.json');assert timing['steps_completed']==24000
    rows=read(D/'manifest.json')['rows'];frame4=int(np.argmin(abs(t-4)))
    for e,cell in enumerate(job['cells']):
        mass=backend['masses'][e].reshape(-1);com=np.average(pose[:,e,:,:3],axis=1,weights=mass)
        delta=(com[-1]-com[frame4])*1000
        lo=pose[:,e,:,:2].min((0,1));hi=pose[:,e,:,:2].max((0,1))
        r=rows[e];half=.5 if not r['surface_file'] else .511
        cond=dict(**cell,worker=job['name'],local_slot=e,duration_s=12,analysis_window_s=[4,12],com_displacement_mm=delta.tolist(),com_net_xy_distance_mm=float(np.linalg.norm(delta[:2])),max_sampled_body_speed_m_s=float(np.linalg.norm(z['body_velocity'][:,e,:,:3],axis=-1).max()),max_sampled_joint_speed_rad_s=float(abs(z['v'][:,e]).max()),foot_contact_frames=int((c['force'][:,e].sum(1)>0).sum()),total_contact_frames=1200,min_body_center_xy_m=lo.tolist(),max_body_center_xy_m=hi.tolist(),body_centers_inside_terrain=bool((lo>-half).all() and (hi<half).all()),surface_file=r['surface_file'],surface_sha256=r['surface_sha256'])
        assert cond['body_centers_inside_terrain'] and cond['foot_contact_frames']>0
        conditions.append(cond)
    evidence.append(dict(worker=job['name'],run_directory=str(D),source_ids=job['source_ids'],trajectory_sha256=sha(D/'trajectory.npz'),backend_sha256=sha(D/'backend.npz'),steps=24000,wall_s=result['wall_s'],thread_settings=th,exact_complete_pd=True,native_errors=[],finite_every_step=True))
receipt=dict(schema='unified_demo_verification_v1',attempt=attempt,complete=True,physics_conditions=12,physics_steps_per_condition=24000,profile_sha256=campaign['profile_sha256'],fixed_pd_sha256=campaign['fixed_pd_sha256'],workers=evidence,conditions=conditions,interpretation='Fixed-PD replay of previously optimized kinematic amplitudes. No dynamic optimum or convergence claim.',historical_failures={'v1':'Windows hidden-process output handle failed before physics','v2':'Sandbox could not read pinned Warp 1.8.1; version assertion stopped before physics','v3':'NumPy integer origin rejected by USD vector constructor before physics','v4':'Completed requested 12-second episodes'})
(O/'simulation_verification.json').write_text(json.dumps(receipt,indent=2),encoding='utf8')
print(json.dumps(dict(complete=True,conditions=[{k:c[k] for k in ['wave','surface','com_displacement_mm','foot_contact_frames']} for c in conditions]),indent=2))

