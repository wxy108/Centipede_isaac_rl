"""Bounded failure capture: finish 2x, then one isolated repeat of failed 1.5x group.
Only adds state saving inside the existing boundary-failure branch. Physics untouched.
"""
from pathlib import Path
import os,sys,ast,json,time,subprocess,inspect,hashlib
import run_height_amplitude as runner
engine=runner.engine
SIM=runner.SIM;ROOT=runner.ROOT
read,dump,sha=runner.read,runner.dump,runner.sha
CAPTURE=SIM/'executed_source/failure_capture_functions.py'


def prepare():
    assert not CAPTURE.exists()
    src=SIM/'executed_source/adapted_functions.py';s=src.read_text(encoding='utf-8')
    old='''            common.write(run_directory / "BOUNDARY_FAILURE.json", dict(t=(step_index+1)*args.dt, minimum_body_center_margin_m=boundary_margin))
            raise RuntimeError("Body entered 15 mm crop-boundary guard; this preview cannot be accepted")'''
    new='''            failure_t=(step_index+1)*args.dt
            failure_pose=as_numpy(robot.data.body_link_pose_w)
            failure_pose[:,:,:3]-=offsets[:,None,:]
            failure_q=as_numpy(robot.data.joint_pos)
            failure_v=as_numpy(robot.data.joint_vel)
            failure_body_v=as_numpy(robot.data.body_link_vel_w)
            env_bounds=[]
            for ee in range(environment_count):
                pxyz=failure_pose[ee,:,:3]
                margins=[float(pxyz[:,0].min()-bx0),float(bx1-pxyz[:,0].max()),float(pxyz[:,1].min()-by0),float(by1-pxyz[:,1].max())]
                env_bounds.append(dict(environment_index=ee,wave=rows[ee]["wave"],margins_m=margins,
                    xyz_min_m=pxyz.min(0).tolist(),xyz_max_m=pxyz.max(0).tolist(),
                    max_abs_joint_velocity_rad_s=float(abs(failure_v[ee]).max()),
                    max_body_linear_speed_m_s=float(np.linalg.norm(failure_body_v[ee,:,:3].astype(float),axis=-1).max())))
            common.write(run_directory / "BOUNDARY_FAILURE.json", dict(t=failure_t, minimum_body_center_margin_m=boundary_margin, environments=env_bounds,
                saved_regular_frame_end_s=frame_times[-1], all_state_finite=bool(np.isfinite(failure_pose).all() and np.isfinite(failure_v).all())))
            np.savez_compressed(run_directory/"failure_state.npz",t=failure_t,pose=failure_pose,q=failure_q,v=failure_v,body_velocity=failure_body_v,body_names=body_names,joint_names=joint_names)
            np.savez_compressed(run_directory/"trajectory_partial.npz",t=frame_times,episode=frame_episodes,pose=body_poses,q=joint_positions,v=joint_velocities,body_velocity=body_velocities,body_names=body_names,joint_names=joint_names)
            raise RuntimeError("Body entered 15 mm crop-boundary guard; this preview cannot be accepted")'''
    assert s.count(old)==1;s=s.replace(old,new);ast.parse(s);CAPTURE.write_text(s,encoding='utf-8')
    dump(SIM/'FAILURE_CAPTURE_PLAN.json',dict(original_source_sha256=sha(src),capture_source_sha256=sha(CAPTURE),source_script_sha256=sha(__file__),
        trigger='Libo height1.5 low group boundary failure at 1.5555s with margin -1.279938m; original guard raised before final trajectory save',
        change='Save offending state and already-recorded frames before the same exception; no stepping, controller, PD, initial state, terrain or boundary rule changed',
        pending_batch='All four 2x jobs, preserving original three-condition slots and 4x2 worker allocation',
        bounded_diagnostic='One isolated three-condition rerun of Libo height1.5 low group with two threads; not an independent statistical replicate or adopted resource setting'))
    print('Failure-only capture prepared; original frozen source preserved.',flush=True)


def worker(job,attempt):
    receipt=read(SIM/'FAILURE_CAPTURE_PLAN.json');assert sha(CAPTURE)==receipt['capture_source_sha256']
    code=inspect.getsource(engine.worker)
    old="patch=SIM/'executed_source/adapted_functions.py'"
    new="patch=SIM/'executed_source/failure_capture_functions.py'"
    assert code.count(old)==1;code=code.replace(old,new)
    old_provenance="execution_sources=c['pins'])"
    assert code.count(old_provenance)==1
    code=code.replace(old_provenance,"execution_sources={**c['pins'],str(patch):sha(patch)})")
    exec(compile(code,str(Path(__file__)), 'exec'),engine.__dict__)
    engine.worker(job,attempt)


def launch_jobs(jobs,attempt):
    procs=[]
    for job in jobs:
        assert not (SIM/'runs'/attempt/job['name']).exists()
        path=SIM/f'{attempt}_{job["name"]}_launch.log'
        with path.open('wb') as f:
            p=subprocess.Popen([sys.executable,'-B',str(Path(__file__)),'worker','--job',job['name'],'--attempt',attempt],
                cwd=ROOT,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        procs.append((job['name'],p));time.sleep(2)
    while any(p.poll() is None for _,p in procs):
        dump(SIM/'recovery_status.json',dict(stage=attempt,complete=False,jobs=[dict(job=n,pid=p.pid,exit=p.poll()) for n,p in procs]));time.sleep(15)
    return [dict(job=n,attempt=attempt,pid=p.pid,exit=p.returncode) for n,p in procs]


def run():
    engine.check();c=read(SIM/'campaign.json');started=time.time()
    pending=[j for j in c['jobs'] if j['height_scale']==2]
    states=launch_jobs(pending,'v2_capture')
    # Exactly one reproduction of the prior fault, preserving all three slots.
    target=[j for j in c['jobs'] if j['name']=='libo_height_x1p50_low']
    states+=launch_jobs(target,'v2_capture')
    dump(SIM/'recovery_status.json',dict(complete=True,jobs=states,elapsed_s=time.time()-started,
                                      meaning='All pending conditions attempted; per-job success/failure retained, complete does not imply all passed'))
    print(json.dumps(states,indent=2),flush=True)


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run','worker']);p.add_argument('--job');p.add_argument('--attempt');a=p.parse_args()
    if a.mode=='prepare':prepare()
    elif a.mode=='run':run()
    else:worker(a.job,a.attempt)
