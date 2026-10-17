import os
for key in ['OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','NUMEXPR_NUM_THREADS']:
    os.environ[key]='1'
os.environ['PXR_WORK_THREAD_LIMIT']='2'
import sys,json,hashlib,traceback,argparse
from pathlib import Path
from types import SimpleNamespace
W=Path(__file__).resolve().parent
NI=Path('C:/Users/wxy22/Documents/Codex/2026-09-15/ni')
CON=Path('C:/Users/wxy22/Documents/Codex/2026-09-11/con')
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
ap=argparse.ArgumentParser();ap.add_argument('job');ap.add_argument('--attempt',default='v1');ap.add_argument('--check',action='store_true');opt=ap.parse_args()
assert not sys.flags.optimize
campaign=read(W/'campaign.json');job=next(j for j in campaign['jobs'] if j['name']==opt.job)
assert sha(Path(campaign['profile']))==campaign['profile_sha256']
profile=read(Path(campaign['profile']));pdp=Path(profile['fixed_pd']['path']);assert sha(pdp)==campaign['fixed_pd_sha256']
pd=read(pdp)['absolute_joint_pd'];rows=read(W/'rows.json');assert sha(W/'rows.json')==campaign['rows_sha256']
assert all(r['absolute_joint_pd']==pd and not r['family_multipliers'] for r in rows)
for p,h in campaign['execution_and_input_pins'].items():assert sha(Path(p))==h,p
args=argparse.Namespace(**job['args']);assert sha(Path(args.experiment_plan))==args.experiment_plan_sha256
plan=read(Path(args.experiment_plan));assert args.dt==plan['dt_s']==profile['physics']['timestep_s']==.0005
assert args.position_iterations==64 and args.velocity_iterations==16 and args.duration==12.
assert plan['benchmark_source_ids']==job['source_ids'] and len(job['source_ids'])==3
if opt.check:print('Preflight passed:',opt.job);sys.exit(0)
D=W/'runs'/opt.attempt/opt.job;D.mkdir(parents=True,exist_ok=False)
runtime=SimpleNamespace(args=args,experiment=plan,rows=rows,input_receipt=dict(input_rows_sha256=sha(W/'rows.json'),profile_sha256=campaign['profile_sha256'],fixed_pd_sha256=campaign['fixed_pd_sha256'],full_table_exact=True),run_directory=D,execution_sources={str(p):sha(p) for p in [* (W/'unified_trial').glob('*.py'),Path(__file__)]})
log=(D/'worker.log').open('w',encoding='utf8');sys.stdout.flush();sys.stderr.flush();os.dup2(log.fileno(),sys.stdout.fileno());os.dup2(log.fileno(),sys.stderr.fileno())
active=dict(pid=os.getpid(),job=opt.job,status='starting',command=sys.argv)
def status(): (D/'active.json').write_text(json.dumps(active,indent=2),encoding='utf8')
status();exitcode=1
try:
    os.environ['WARP_CACHE_PATH']=str(D/'warp_cache')
    sys.path.insert(0,str(CON/'open_loop_review_2026-09-16'))
    sys.path.insert(0,str(NI/'work/runtime_speed_2026-09-17/deps'))
    import warp as wp
    assert wp.__version__=='1.8.1';wp.init()
    from isaaclab.app import AppLauncher
    launcher=AppLauncher(args);assert launcher.app is not None
    import torch
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    sys.path.insert(0,str(CON/'terrain_surface_library_2026-09-15'))
    from unified_trial import simulation
    active['status']='running';status();simulation.run_trial(runtime)
    result=read(D/'results.json');assert result['complete'] and result['whole_batch_finite'] and result['actual_duration_s']==12.
    errors=read(D/'native_error_audit.json')['errors'];assert not errors,errors
    active['status']='finished';exitcode=0
except BaseException:
    active['status']='failed';traceback.print_exc()
finally:
    active['exit_code']=exitcode;status();sys.stdout.flush();sys.stderr.flush();log.flush()
os._exit(exitcode)
