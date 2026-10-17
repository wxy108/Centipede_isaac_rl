def check():
    c=read(SIM/'campaign.json')
    for p,h in c['pins'].items():assert sha(p)==h,p
    pd=read(MAIN/'pd_wave4_firm20.json')['absolute_joint_pd'];seen=set()
    for j in c['jobs']:
        args=j['args'];assert args['dt']==.0005 and args['duration']==6 and args['position_iterations']==64 and args['velocity_iterations']==16
        assert sha(args['rows'])==j['rows_sha256'] and sha(args['experiment_plan'])==args['experiment_plan_sha256']
        p=read(args['experiment_plan']);rows=read(args['rows']);assert p['benchmark_source_ids']==j['source_ids']
        for cell,i in zip(j['cells'],j['source_ids']):
            row=rows[i];assert row['absolute_joint_pd']==pd and not row['family_multipliers']
            reference=next(t for t in c['flat_cluster_cells'] if t['wave']==cell['wave'])
            assert all(cell[k]==reference[k] for k in ['A_deg','B_deg','body_factor','leg_factor'])
            sp=c['spawns'][row['spawn_index']];assert row['start_xy']==sp['start_xy_m'] and row['start_yaw_deg']==sp['start_yaw_deg']
            if row['terrain']=='rough':assert sha(row['surface_file'])==row['surface_sha256']
            key=(j['height_scale'],cell['wave'],cell['spawn_index']);assert key not in seen;seen.add(key)
    assert len(seen)==c["condition_count"]
    print('INPUT CHECKS PASS:full173PD, frozen A/B, rotated placement, collision envelope, timing, source hashes',flush=True)

def run():
    import psutil
    check();c=read(SIM/'campaign.json');assert not (SIM/'runs/v1').exists()
    assert psutil.virtual_memory().available>10*1024**3
    started=time.time();finished=[];queue=[]
    def batch(entries,label):
        procs=[]
        for name,attempt,keep in entries:
            cmd=[sys.executable,'-B',str(Path(__file__)),'worker','--job',name,'--attempt',attempt]
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
    dump(SIM/'supervisor.json',dict(complete=True,all_attempts_accounted_for=True,started_unix=started,elapsed_s=time.time()-started,finished=finished,current=[],meaning='Complete means all100 planned observations attempted and peers recovered; divergent/boundary trials remain failed.'))
    print('CAMPAIGN ATTEMPTS FINISHED; no further physics queued',flush=True)
