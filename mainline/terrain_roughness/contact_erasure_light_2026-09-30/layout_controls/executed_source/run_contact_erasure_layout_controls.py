"""Matched10slot deterministic endpoints only. Not independent random samples."""
from pathlib import Path
import os,sys,copy,json,subprocess,time,argparse
ROOT=Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
sys.path.insert(0,str(ROOT))
import run_contact_erasure_light_v2 as impl
ORIGINAL_SIM=impl.SIM
SIM=impl.BASE/'layout_controls';impl.SIM=SIM
read,sha,dump=impl.read,impl.sha,impl.dump

def prepare():
    assert not SIM.exists()
    c=read(ORIGINAL_SIM/'campaign.json');template=next(j for j in c['jobs'] if j['name']=='bin010_p010_b00')
    for p,h in c['pins'].items():assert sha(p)==h,p
    SIM.mkdir();snap=SIM/'executed_source';snap.mkdir()
    (snap/'adapted_functions.py').write_bytes((ORIGINAL_SIM/'executed_source/adapted_functions.py').read_bytes())
    (snap/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    jobs=[]
    for probability in [0.,1.]:
        j=copy.deepcopy(template);name=f'matched_endpoint_p{int(probability)}';j['name']=name
        for cell in j['cells']:cell.update(p=probability,erase_bin_s=.01)
        rows=read(j['args']['rows'])
        for e,row in enumerate(rows):row.update(id=name+f'_e{e:02d}',erasure=dict(p=probability,mask_seed=j['cells'][e]['mask_seed'],erase_bin_s=.01))
        rp=SIM/f'{name}_rows.json';dump(rp,rows);j['rows_sha256']=sha(rp)
        plan=read(j['args']['experiment_plan']);plan.update(run_name=name,rows_sha256=sha(rp),joint_cells=j['cells'],worker_filename=Path(__file__).name,
            campaign_note='Matched10slot deterministic endpoint layout control; two physical placements perwave, NOTtwo independent seeds or additional stochastic observations.')
        pp=SIM/f'{name}_plan.json';dump(pp,plan);j['args'].update(name=name,rows=str(rp),experiment_plan=str(pp),experiment_plan_sha256=sha(pp));jobs.append(j)
    c.update(jobs=jobs,batches=[],pins=dict(c['pins'],**{str(Path(__file__)):sha(__file__),str(snap/'adapted_functions.py'):sha(snap/'adapted_functions.py')}),
        schema='same10slot_deterministic_endpoint_control_v1',condition_count=10,physical_instance_count=20,independent_random_samples=0,
        purpose='Endpoints share stochastic10environment layout andwave/slot ordering. Existing3+2endpoints remain historicalidentity/channel controls; new pairedplacement controls eliminate layout mismatch without adding randomreplicates.')
    dump(SIM/'campaign.json',c);print('Prepared20physical deterministic instances/10unique endpoints; no extra randomseed',flush=True)

def run():
    assert read(ORIGINAL_SIM/'supervisor.json')['complete'],'Only run after light stochastic workers finish'
    assert not (SIM/'supervisor.json').exists()
    c=read(SIM/'campaign.json');started=time.time();procs=[]
    for j in c['jobs']:
        name=j['name'];log=(SIM/f'{name}_launch.log').open('wb')
        p=subprocess.Popen([sys.executable,'-B',str(Path(__file__)),'worker','--job',name],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW);log.close();procs.append((name,p));time.sleep(2)
    while True:
        current=[dict(job=name,pid=p.pid,exit=p.poll()) for name,p in procs]
        dump(SIM/'supervisor.json',dict(complete=False,pid=os.getpid(),current=current,started_unix=started,elapsed_s=time.time()-started))
        if all(p.poll() is not None for _,p in procs):break
        time.sleep(10)
    assert all(p.returncode==0 for _,p in procs),current
    for name,p in procs:
        er=read(SIM/'runs/v1'/name/'ERASURE_RESULT.json');assert er['complete'] and er['deleted_contact_violations']==0
        assert read(SIM/'runs/v1'/name/'ERASURE_FIXED_PLANT.json')['passed']
    dump(SIM/'supervisor.json',dict(complete=True,pid=os.getpid(),finished=current,current=[],started_unix=started,elapsed_s=time.time()-started,physical_instance_count=20,unique_endpoint_count=10,new_independent_random_samples=0))
    print('Matchedlayout controls complete; deterministic duplicates not statisticalseeds',flush=True)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['prepare','run','worker']);ap.add_argument('--job');a=ap.parse_args()
    if a.mode=='prepare':prepare()
    elif a.mode=='run':run()
    else:impl.worker(a.job,'v1')
