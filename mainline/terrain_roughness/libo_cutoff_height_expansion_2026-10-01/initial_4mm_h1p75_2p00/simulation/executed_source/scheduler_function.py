def run():
    import psutil
    check();c=read(SIM/'campaign.json');assert not (SIM/'runs/v1').exists()
    assert psutil.virtual_memory().available>10*1024**3
    started=time.time();finished=[];queue=[]
    def batch(entries,label):
        procs=[]
        for name,attempt,keep in entries:
            cmd=[sys.executable,'-B',str(Path(__file__)),'worker','--job',name,'--attempt',attempt,'--stage',EXPANSION_STAGE]
            if keep is not None:cmd+=['--keep',','.join(map(str,keep))]
            with (SIM/f'{attempt}_{name}_launch.log').open('wb') as log:
                p=subprocess.Popen(cmd,cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            procs.append((name,attempt,keep,p));time.sleep(2)
        while True:
            current=[]
            for name,attempt,keep,p in procs:
                d=SIM/'runs'/attempt/name
                try:a=read(d/'active.json')
                except(FileNotFoundError,json.JSONDecodeError):a={}
                current.append(dict(job=name,attempt=attempt,keep=keep,pid=p.pid,exit=p.poll(),status=a.get('status')))
            dump(SIM/'supervisor.json',dict(complete=False,batch=label,started_unix=started,elapsed_s=time.time()-started,finished=finished,current=current))
            if all(p.poll() is not None for *_,p in procs):break
            time.sleep(10)
        finished.extend(current)
        for item in current:
            if item['exit']!=0:
                j=next(j for j in c['jobs'] if j['name']==item['job']);ids=item['keep'] or j['source_ids']
                peers,off=failure_peers(SIM/'runs'/item['attempt']/item['job'],ids)
                item['offender_source_ids']=off;item['interrupted_peer_source_ids']=peers
                if peers:queue.append((item['job'],'peer'+str(len(ids)-len(peers))+f'_from_{item["attempt"]}',peers))
        print(json.dumps(dict(batch=label,attempted_groups=len(finished),elapsed_s=time.time()-started,peer_recoveries_pending=len(queue))),flush=True)
    for b in c['batches']:batch([(name,'v1',None) for name in b['jobs']],b['name'])
    recovery_round=0
    while queue:
        recovery_round+=1;assert recovery_round<=3
        current=queue[:];queue.clear()
        for i in range(0,len(current),4):batch(current[i:i+4],f'peer_recovery{recovery_round}_{i//4}')
    dump(SIM/'supervisor.json',dict(complete=True,all_attempts_accounted_for=True,started_unix=started,elapsed_s=time.time()-started,finished=finished,current=[],meaning='Complete means all300 planned observations attempted and peers recovered; divergent/boundary trials remain failed.'))
    print('CAMPAIGN ATTEMPTS FINISHED; no further physics queued',flush=True)
