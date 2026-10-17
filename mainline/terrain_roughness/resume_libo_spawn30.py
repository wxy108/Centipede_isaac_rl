"""Continue only unfinished authorized conditions; original native worker unchanged."""
from pathlib import Path
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[key]='1'
os.environ['PXR_WORK_THREAD_LIMIT']='2'
import sys,json,time,copy,subprocess,hashlib
import psutil
ROOT=Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
sys.path.insert(0,str(ROOT))
import run_libo_spawn30 as runner
SIM=runner.SIM
read,dump,sha=runner.read,runner.dump,runner.sha

def owned_alive(pid):
    try:
        p=psutil.Process(pid)
        return p.is_running() and any(s in ' '.join(p.cmdline()) for s in ('run_libo_spawn30.py','resume_libo_spawn30.py'))
    except (psutil.NoSuchProcess,psutil.AccessDenied):return False

def resume():
    prior=read(SIM/'supervisor.json');assert not prior['complete']
    assert not owned_alive(read(SIM/'DISPATCH.json')['pid'])
    assert not any(owned_alive(r['pid']) for r in prior.get('current',[]))
    runner.check();campaign=read(SIM/'campaign.json');jobs={j['name']:j for j in campaign['jobs']}
    # Preserve all pre-interruption receipts before updating the live index.
    history=SIM/'dispatch_resume_history';history.mkdir(exist_ok=True)
    number=len(list(history.glob('supervisor_before_*.json')))+1
    before=history/f'supervisor_before_{number:02d}.json'
    before.write_bytes((SIM/'supervisor.json').read_bytes())
    (history/f'dispatch_before_{number:02d}.json').write_bytes((SIM/'DISPATCH.json').read_bytes())
    (history/f'resume_source_{number:02d}.py').write_bytes(Path(__file__).read_bytes())
    started=prior['started_unix'];finished=copy.deepcopy(prior['finished']);external=[]
    known={(r['attempt'],r['job']) for r in finished}
    for item in prior.get('current',[]):
        if (item['attempt'],item['job']) in known:continue
        item=copy.deepcopy(item);d=SIM/'runs'/item['attempt']/item['job']
        a=read(d/'active.json');ids=a['source_ids']
        if item.get('exit')==0 or (a.get('exit_code')==0 and (d/'results.json').exists()):
            assert read(d/'results.json')['complete'] and not read(d/'native_error_audit.json')['errors']
            item.update(exit=0,status='finished',reconciled_after_dispatch_interruption=True)
        elif (d/'BOUNDARY_FAILURE.json').exists() or (d/'NUMERICAL_FAILURE.json').exists():
            item.update(exit=item.get('exit') or 1,status='failed',reconciled_after_dispatch_interruption=True)
        else:
            assert not (d/'results.json').exists()
            receipt=dict(classification='external_interruption_unscored',source_ids=ids,
                         observed_unix=time.time(),last_active_status=a.get('status'),
                         evidence='Owned dispatcher and worker absent; no complete result or native quality-failure receipt.',
                         cause='Unknown; observed after conversation interruption.',
                         physics_or_amplitude_changes=False,reexecute_same_conditions=True)
            dump(d/'EXTERNAL_INTERRUPTION.json',receipt)
            item.update(exit=-999,status='externally_interrupted',external_interruption=True)
            # Same complete worker layout for this interrupted, non-quality-failed job.
            keep=None if ids==jobs[item['job']]['source_ids'] else ids
            external.append((item['job'],f'interruption_resume{number:02d}',keep))
        finished.append(item)
    dump(SIM/'DISPATCH.json',dict(pid=os.getpid(),status='resumed',runner=str(Path(__file__)),
         resumed_unix=time.time(),original_started_unix=started,new_condition_count=600,
         scope=campaign['scope'],previous_receipt=str(history/f'dispatch_before_{number:02d}.json')))
    queue=[]
    # Reconstruct only outstanding nonoffender peers, never retry a numerical offender.
    completed=set();offended=set()
    for item in finished:
        ids=item.get('keep') or jobs[item['job']]['source_ids']
        if item['exit']==0:completed.update((item['job'],i) for i in ids)
        elif not item.get('external_interruption'):
            peers,off=runner.original.failure_peers(SIM/'runs'/item['attempt']/item['job'],ids)
            offended.update((item['job'],i) for i in off)
    for item in finished:
        if item['exit']==0 or item.get('external_interruption'):continue
        ids=item.get('keep') or jobs[item['job']]['source_ids']
        peers,off=runner.original.failure_peers(SIM/'runs'/item['attempt']/item['job'],ids)
        pending=[i for i in peers if (item['job'],i) not in completed|offended]
        if pending and not any(q[0]==item['job'] and set(q[2])==set(pending) for q in queue):
            queue.append((item['job'],f'peer_resume{number:02d}_round1',pending))
    def batch(entries,label):
        procs=[]
        assert len(entries)<=4
        for name,attempt,keep in entries:
            assert not (SIM/'runs'/attempt/name).exists()
            cmd=[sys.executable,'-B',str(ROOT/'run_libo_spawn30.py'),'worker','--job',name,'--attempt',attempt]
            if keep is not None:cmd+=['--keep',','.join(map(str,keep))]
            with (SIM/f'{attempt}_{name}_launch.log').open('wb') as log:
                p=subprocess.Popen(cmd,cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            procs.append((name,attempt,keep,p));time.sleep(2)
        while True:
            current=[]
            for name,attempt,keep,p in procs:
                try:a=read(SIM/'runs'/attempt/name/'active.json')
                except (FileNotFoundError,json.JSONDecodeError):a={}
                current.append(dict(job=name,attempt=attempt,keep=keep,pid=p.pid,exit=p.poll(),status=a.get('status')))
            dump(SIM/'supervisor.json',dict(complete=False,batch=label,started_unix=started,elapsed_s=time.time()-started,
                 finished=finished,current=current,resume_number=number,original_interruption_receipt=str(before)))
            if all(p.poll() is not None for *_,p in procs):break
            time.sleep(10)
        finished.extend(current)
        for item in current:
            if item['exit']!=0:
                ids=item['keep'] or jobs[item['job']]['source_ids']
                peers,off=runner.original.failure_peers(SIM/'runs'/item['attempt']/item['job'],ids)
                item['offender_source_ids']=off;item['interrupted_peer_source_ids']=peers
                if peers:queue.append((item['job'],f'peer_resume{number:02d}_after_{item["attempt"]}',peers))
        print(json.dumps(dict(batch=label,worker_attempts=len(finished),elapsed_s=time.time()-started,pending_peers=len(queue))),flush=True)
    if external:batch(external,f'external_interruption_resume{number:02d}')
    attempted_formal={r['job'] for r in finished if r['attempt']=='v1'}
    for b in campaign['batches']:
        entries=[(name,'v1',None) for name in b['jobs'] if name not in attempted_formal]
        if entries:batch(entries,b['name'])
    recovery_round=0
    while queue:
        recovery_round+=1;assert recovery_round<=3
        current=queue[:];queue.clear()
        for i in range(0,len(current),4):batch(current[i:i+4],f'peer_recovery_resume{number:02d}_{recovery_round}_{i//4}')
    dump(SIM/'supervisor.json',dict(complete=True,all_attempts_accounted_for=True,started_unix=started,
         elapsed_s=time.time()-started,finished=finished,current=[],resume_number=number,
         original_interruption_receipt=str(before),meaning='All600 authorized conditions accounted for; native failures retained, external unscored attempts preserved, no new independent samples.'))
    print('AUTHORIZED600 ATTEMPTS FINISHED; no further physics queued',flush=True)

if __name__=='__main__':resume()
