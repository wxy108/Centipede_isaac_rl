"""Resume only unscored light280 inputs after an external process interruption."""
from pathlib import Path
import os,sys,json,time,argparse,subprocess,traceback
ROOT=Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
sys.path.insert(0,str(ROOT))
import run_contact_erasure_light_v2 as impl
import psutil
SIM=impl.SIM;BASE=impl.BASE;CONTROL=BASE/'layout_controls'
read,sha,dump=impl.read,impl.sha,impl.dump
STATUS=SIM/'continuation_supervisor.json'

def inventory():
    c=read(SIM/'campaign.json');assert c['condition_count']==280 and c['mask_seeds']==[0,1]
    for path,h in c['pins'].items():assert sha(path)==h,path
    old=read(SIM/'supervisor.json')
    for pid in [old['pid']]+[x['pid'] for x in old['current']]:
        if psutil.pid_exists(pid):
            cmd=' '.join(psutil.Process(pid).cmdline())
            assert 'run_contact_erasure_light_v2.py' not in cmd,'Original light process still running'
    jobs={j['name']:j for j in c['jobs']};finished=[x for x in old['finished'] if x.get('reused')]
    complete={name:set() for name in jobs};failed={name:set() for name in jobs};external=[]
    for parent in sorted((SIM/'runs').iterdir()):
        if not parent.is_dir():continue
        for d in sorted(parent.iterdir()):
            if not d.is_dir() or not (d/'active.json').exists():continue
            a=read(d/'active.json');name=d.name;assert name in jobs
            ids=a['source_ids'];er=read(d/'ERASURE_RESULT.json') if (d/'ERASURE_RESULT.json').exists() else None
            if (d/'ERASURE_FAILURE.json').exists():raise RuntimeError('Native erasure failure requires diagnosis, not restart')
            result=read(d/'results.json') if (d/'results.json').exists() else None
            item=dict(job=name,attempt=parent.name,keep=ids,pid=a['pid'],status=a['status'],run_path=str(d))
            if result is not None and result['complete'] and er is not None and er['complete']:
                assert not read(d/'native_error_audit.json')['errors'] and er['deleted_contact_violations']==0
                item.update(exit=0,status='finished');complete[name].update(ids)
            elif (d/'BOUNDARY_FAILURE.json').exists() or (d/'NUMERICAL_FAILURE.json').exists():
                peers,off=impl.parent.failure_peers(d,ids);failed[name].update(off)
                item.update(exit=1,offender_source_ids=off,interrupted_peer_source_ids=peers)
            elif a['status']=='running' or a['status']=='starting':
                assert not psutil.pid_exists(a['pid'])
                item.update(exit=None,status='external_interruption',external_interruption=True)
                external.append(item)
            else:raise RuntimeError('Unclassified attempt '+str(d))
            finished.append(item)
    pending=[]
    for j in c['jobs']:
        if j['pilot_only'] or 'reuse_path' in j:continue
        assert not (complete[j['name']] & failed[j['name']])
        for source_id in j['source_ids']:
            if source_id not in complete[j['name']]|failed[j['name']]:pending.append((j['name'],source_id))
    return c,jobs,finished,pending,external,old

def run():
    assert not STATUS.exists(),'Do not duplicate continuation'
    c,jobs,finished,pending,external,old=inventory();started=time.time()
    dump(SIM/'CONTINUATION_INPUT.json',dict(scope='Existing light280 only; full4060 and rough expansion held',pending_source_ids=pending,
        external_interrupted_attempts=external,unchanged_native_worker_source=str(ROOT/'run_contact_erasure_light_v2.py'),
        unchanged_native_worker_sha256=sha(ROOT/'run_contact_erasure_light_v2.py'),scheduler_source_sha256=sha(__file__),
        recovery='FourCPUworkers x2threads,oneactualremainingcondition perworker. No offender retry or newmaskseed. Layout change explicitlyreported.',
        previous_observed_active_elapsed_s=old['elapsed_s'],new_started_unix=started))
    (SIM/'continuation_source.py').write_bytes(Path(__file__).read_bytes())
    def write(phase,current,complete=False,**extra):
        dump(STATUS,dict(complete=complete,pid=os.getpid(),phase=phase,started_unix=started,elapsed_s=time.time()-started,
            previous_observed_active_elapsed_s=old['elapsed_s'],finished=finished,current=current,external_interruption_count=len(external),**extra))
    try:
        for block in range(0,len(pending),4):
            procs=[]
            for name,i in pending[block:block+4]:
                attempt=f'isolated_resume01_s{i:02d}';assert not (SIM/'runs'/attempt/name).exists()
                log=(SIM/f'{attempt}_{name}_launch.log').open('wb')
                p=subprocess.Popen([sys.executable,'-B',str(ROOT/'run_contact_erasure_light_v2.py'),'worker','--job',name,'--attempt',attempt,'--keep',str(i)],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
                log.close();procs.append((name,attempt,i,p));time.sleep(2)
            while True:
                current=[dict(job=name,attempt=attempt,keep=[i],pid=p.pid,exit=p.poll()) for name,attempt,i,p in procs]
                write(f'isolated_recovery_{block//4:03d}',current)
                if all(p.poll() is not None for *_,p in procs):break
                time.sleep(10)
            for name,attempt,i,p in procs:
                d=SIM/'runs'/attempt/name;a=read(d/'active.json')
                item=dict(job=name,attempt=attempt,keep=[i],pid=p.pid,exit=p.returncode,status=a['status'],run_path=str(d),isolated_recovery=True)
                if p.returncode!=0:
                    if (d/'ERASURE_FAILURE.json').exists():raise RuntimeError('Native erasure failure; halted')
                    peers,off=impl.parent.failure_peers(d,[i]);assert off==[i] and not peers
                    item.update(offender_source_ids=off,interrupted_peer_source_ids=[])
                finished.append(item)
            print(json.dumps(dict(block=block//4,resolved_remaining=min(block+4,len(pending)),remaining_total=len(pending),elapsed_s=time.time()-started)),flush=True)
        # The following20deterministic instances were explicitly approved bythe human.
        assert not (CONTROL/'supervisor.json').exists(),'Preserve already-started matchedcontrols'
        cc=read(CONTROL/'campaign.json');assert cc['physical_instance_count']==20 and cc['independent_random_samples']==0
        control_started=time.time();procs=[]
        for j in cc['jobs']:
            name=j['name'];log=(CONTROL/f'{name}_launch.log').open('wb')
            p=subprocess.Popen([sys.executable,'-B',str(ROOT/'run_contact_erasure_layout_controls.py'),'worker','--job',name],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            log.close();procs.append((name,p));time.sleep(2)
        while True:
            current=[dict(job=name,pid=p.pid,exit=p.poll()) for name,p in procs];write('matched_layout_controls',current,stochastic_complete=True)
            dump(CONTROL/'supervisor.json',dict(complete=False,pid=os.getpid(),current=current,started_unix=control_started,elapsed_s=time.time()-control_started,guard='All lightstochastic conditions resolved before controls; original stale interruptedsupervisor preserved'))
            if all(p.poll() is not None for _,p in procs):break
            time.sleep(10)
        assert all(p.returncode==0 for _,p in procs),current
        for name,p in procs:
            d=CONTROL/'runs/v1'/name
            assert read(d/'ERASURE_RESULT.json')['complete'] and not read(d/'native_error_audit.json')['errors']
        dump(CONTROL/'supervisor.json',dict(complete=True,pid=os.getpid(),finished=current,current=[],started_unix=control_started,elapsed_s=time.time()-control_started,physical_instance_count=20,unique_endpoint_count=10,new_independent_random_samples=0))
        write('all_light_physics_complete',[],True,stochastic_complete=True,matched_controls_complete=True,all280_accounted_for=True)
        print('All remaininglight inputs resolved; matched20instances complete; no full4060/rough expansion resumed',flush=True)
    except BaseException:
        dump(SIM/'CONTINUATION_FAILURE.json',dict(error=traceback.format_exc(),elapsed_s=time.time()-started,finished=finished))
        raise

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['inventory','run']);args=ap.parse_args()
    if args.mode=='inventory':
        c,jobs,finished,pending,external,old=inventory()
        print(json.dumps(dict(remaining=len(pending),external_interrupted_attempts=len(external),previous_observed_active_minutes=old['elapsed_s']/60,existing_attempts=len(finished))),flush=True)
    else:run()
