"""Only the two human-approved matched-layout endpoint controls; no stochastic dispatch."""
from pathlib import Path
import sys,os,time,json,subprocess,traceback
import psutil
ROOT=Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
sys.path.insert(0,str(ROOT))
import run_contact_erasure_layout_controls as ctrl
SIM=ctrl.SIM
for proc in psutil.process_iter(['pid','name']):
    if proc.info['pid']==os.getpid() or 'python' not in (proc.info['name'] or '').lower():continue
    try:cmd=proc.cmdline()
    except (psutil.AccessDenied,psutil.NoSuchProcess):continue
    if any('contact_erasure' in str(x) for x in cmd) and 'worker' in cmd:raise RuntimeError('Native contact worker still active; no extra launch')
assert not (SIM/'supervisor.json').exists()
c=ctrl.read(SIM/'campaign.json')
assert c['physical_instance_count']==20 and c['condition_count']==10 and c['independent_random_samples']==0
for path,h in c['pins'].items():assert ctrl.sha(path)==h,path
started=time.time();procs=[]
guard='No stochastic native workers active. Stochastic dispatcher remains paused for runtime-scope choice. Only two user-approved p0/p1 matched-layout jobs; original interrupted supervisor remains preserved.'
try:
    for j in c['jobs']:
        name=j['name'];assert not (SIM/'runs/v1'/name).exists()
        with (SIM/f'{name}_launch.log').open('wb') as log:
            p=subprocess.Popen([sys.executable,'-B',str(ROOT/'run_contact_erasure_layout_controls.py'),'worker','--job',name],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        procs.append((name,p));time.sleep(2)
    while True:
        cur=[dict(job=name,pid=p.pid,exit=p.poll()) for name,p in procs]
        ctrl.dump(SIM/'supervisor.json',dict(complete=False,pid=os.getpid(),current=cur,started_unix=started,elapsed_s=time.time()-started,guard=guard))
        if all(p.poll() is not None for _,p in procs):break
        time.sleep(10)
    assert all(p.returncode==0 for _,p in procs),cur
    for name,p in procs:
        d=SIM/'runs/v1'/name
        er=ctrl.read(d/'ERASURE_RESULT.json')
        assert er['complete'] and er['deleted_contact_violations']==0
        assert ctrl.read(d/'ERASURE_FIXED_PLANT.json')['passed']
        assert not ctrl.read(d/'native_error_audit.json')['errors']
    ctrl.dump(SIM/'supervisor.json',dict(complete=True,pid=os.getpid(),finished=cur,current=[],started_unix=started,elapsed_s=time.time()-started,guard=guard,physical_instance_count=20,unique_endpoint_count=10,new_independent_random_samples=0))
    print('Approved matched endpoints finished',flush=True)
except BaseException:
    ctrl.dump(SIM/'CONTROL_FAILURE.json',dict(error=traceback.format_exc(),elapsed_s=time.time()-started,current=[dict(job=n,pid=p.pid,exit=p.poll()) for n,p in procs],guard=guard));raise