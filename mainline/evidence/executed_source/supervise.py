"""One authorized four-worker batch. Never retries or searches configurations."""
import subprocess,sys,time,json,os
from pathlib import Path
import psutil
W=Path(__file__).resolve().parent
attempt=sys.argv[1] if len(sys.argv)>1 else 'v1'
campaign=json.loads((W/'campaign.json').read_text());procs=[]
assert psutil.virtual_memory().available>9*1024**3,'Insufficient memory to launch the requested four workers'
started=time.time()
for job in campaign['jobs']:
    bootstrap=(W/(attempt+'_'+job['name']+'_launch.log')).open('wb')
    p=subprocess.Popen([sys.executable,str(W/'launch.py'),job['name'],'--attempt',attempt],cwd=W,stdin=subprocess.DEVNULL,stdout=bootstrap,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
    bootstrap.close()
    procs.append((job['name'],p))
    time.sleep(2)
while True:
    states=[]
    for name,p in procs:
        active=W/'runs'/attempt/name/'active.json'
        try:state=json.loads(active.read_text())
        except (FileNotFoundError,json.JSONDecodeError):state={}
        states.append(dict(job=name,pid=p.pid,exit=p.poll(),status=state.get('status'),rss_bytes=psutil.Process(p.pid).memory_info().rss if p.poll() is None else None))
    receipt=dict(attempt=attempt,started=started,elapsed_s=time.time()-started,available_memory_gib=psutil.virtual_memory().available/1024**3,workers=states,complete=all(p.poll()==0 for _,p in procs))
    (W/('supervisor_'+attempt+'.json')).write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt),flush=True)
    if all(p.poll() is not None for _,p in procs):break
    time.sleep(20)
sys.exit(0 if receipt['complete'] else 1)
