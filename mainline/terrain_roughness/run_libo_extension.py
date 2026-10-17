"""Extend the frozen paired-spawn campaign to Libo heights 1.25 and 1.5."""
from pathlib import Path
import os,sys,json,copy,ast,argparse,shutil
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:os.environ[key]='1'
os.environ['PXR_WORK_THREAD_LIMIT']='2'
ROOT=Path(__file__).resolve().parent;MAIN=ROOT.parent
sys.path.insert(0,str(ROOT))
import run_libo_repeats as original
import numpy as np
read,sha,dump=original.read,original.sha,original.dump
PARENT=ROOT/'libo_spawn_repeats';BASE=PARENT/'extension_1p25_1p5';SIM=BASE/'simulation';SCALES=[1.25,1.5]

# Reuse the original worker and scheduler. Only the campaign path and count text differ.
original.OUT=BASE;original.SIM=SIM;original.__file__=str(Path(__file__).resolve())
tree=ast.parse((ROOT/'run_libo_repeats.py').read_text(encoding='utf-8'))
functions={n.name:ast.get_source_segment((ROOT/'run_libo_repeats.py').read_text(encoding='utf-8'),n) for n in tree.body if isinstance(n,ast.FunctionDef)}
check_source=functions['check'].replace('assert len(seen)==205','assert len(seen)==c["condition_count"]')
assert check_source!=functions['check']
run_source=functions['run'].replace('all205 planned observations','all100 planned observations')
assert run_source!=functions['run']
exec(compile(check_source+'\n\n'+run_source,str(Path(__file__).resolve()),'exec'),original.__dict__)

def check():
    c=read(SIM/'campaign.json');parent=read(PARENT/'simulation/campaign.json')
    assert c['spawns']==parent['spawns'] and c['flat_cluster_cells']==parent['flat_cluster_cells']
    assert c['profile']==parent['profile'] and c['fixed_pd_sha256']==parent['fixed_pd_sha256']
    assert c['collision_bounds']==parent['collision_bounds'] and c['condition_count']==100 and c['new_flat_condition_count']==0
    assert sha(SIM/'executed_source/adapted_functions.py')==sha(PARENT/'simulation/executed_source/adapted_functions.py')
    original.check()

def prepare():
    assert not SIM.exists(),'Preserve prepared extension'
    assert read(PARENT/'simulation/supervisor.json')['complete'],'Parent campaign must be finished'
    parent=read(PARENT/'simulation/campaign.json')
    for p,h in parent['pins'].items():assert sha(p)==h,p
    terrain=read(ROOT/'height_amplitude_4mm/REPORT.json')
    BASE.mkdir(exist_ok=False);SIM.mkdir();snap=SIM/'executed_source';snap.mkdir()
    shutil.copy2(PARENT/'simulation/executed_source/adapted_functions.py',snap/'adapted_functions.py')
    (snap/'scheduler_functions.py').write_text(check_source+'\n\n'+run_source+'\n',encoding='utf-8')
    template_rows=read(next(j for j in parent['jobs'] if j['height_scale']==1)['args']['rows'])
    jobs=[];surfaces=[]
    for scale in SCALES:
        tr=next(t for t in terrain['terrains'] if t['site']=='libo' and t['height_scale']==scale)
        assert sha(tr['path'])==tr['sha256'];surfaces.append(tr)
        with np.load(tr['path']) as z:h=z['height'];dx=float(z['dx_m'])
        rows=copy.deepcopy(template_rows);initials=[];level='height_x'+f'{scale:.2f}'.replace('.','p')
        for i,row in enumerate(rows):
            sid=row['spawn_index'];sp=parent['spawns'][sid]
            row.update(id=f'{level}_s{sid:02d}_w{row["wave"]:g}',height_scale=scale,surface_file=tr['path'],surface_sha256=tr['sha256'])
            pose=np.array(row['matched_initial_root_pose'],float);pose[:2]=sp['start_xy_m'];ang=np.deg2rad(sp['start_yaw_deg'])
            pose[3:]=[np.cos(ang/2),0,0,np.sin(ang/2)]
            ny,nx=h.shape;ix=int(pose[0]/dx+(nx-1)/2);iy=int(pose[1]/dx+(ny-1)/2);rad=int(np.ceil(.075/dx))
            pose[2]+=h[max(0,iy-rad):min(ny,iy+rad+1),max(0,ix-rad):min(nx,ix+rad+1)].max()+.002
            initials.append(pose.tolist())
            allowed={'id','height_scale','surface_file','surface_sha256'}
            assert all(row[k]==template_rows[i][k] for k in row if k not in allowed)
        rp=SIM/f'{level}_rows.json';dump(rp,rows)
        for sid in range(10):
            for group in ['low','high']:
                template=next(j for j in parent['jobs'] if j['height_scale']==1 and j['spawn_index']==sid and j['group']==group)
                j=copy.deepcopy(template);name=f'{level}_s{sid:02d}_{group}'
                selected=copy.deepcopy(j['cells'])
                for cell in selected:cell.update(height_scale=scale,surface=level,fixed_pair_angle_rms_deg=tr['fixed_pair_angle_rms_deg'])
                plan=read(template['args']['experiment_plan']);plan.update(run_name=name,rows_sha256=sha(rp),planned_initial_pose=initials,joint_cells=selected,worker_filename=Path(__file__).name,
                    campaign_note='User authorized100 additional Libo observations at1.25/1.5; same ten spawns; no amplitude/PD/physics retuning',
                    quality_protocol=dict(duration_s=6.,comparison='Same10spawns as parent, two additional terrain heights, five fixed waves',only_focal_poses_recorded=False))
                pp=SIM/f'{name}_plan.json';dump(pp,plan)
                j.update(name=name,level=level,height_scale=scale,cells=selected,rows_sha256=sha(rp))
                j['args'].update(rows=str(rp),name=name,experiment_plan=str(pp),experiment_plan_sha256=sha(pp))
                jobs.append(j)
    batches=[]
    for pair in range(5):
        order=SCALES if pair%2==0 else SCALES[::-1]
        for scale in order:
            names=[j['name'] for sid in [pair*2,pair*2+1] for group in ['low','high'] for j in jobs if j['height_scale']==scale and j['spawn_index']==sid and j['group']==group]
            assert len(names)==4;batches.append(dict(name=f'pair{pair:02d}_height{scale:g}',jobs=names))
    pins=dict(parent['pins']);extra=[Path(__file__),PARENT/'simulation/campaign.json',snap/'adapted_functions.py',snap/'scheduler_functions.py']
    pins.update({str(p):sha(p) for p in extra});pins.update({t['path']:t['sha256'] for t in surfaces})
    for p in [Path(__file__),ROOT/'run_libo_repeats.py']:(snap/p.name).write_bytes(p.read_bytes())
    c=copy.deepcopy(parent);c.update(schema='libo_paired_spawn_extension_v1',jobs=jobs,batches=batches,pins=pins,height_scales=SCALES,
        condition_count=100,rough_condition_count=100,new_flat_condition_count=0,parent_campaign=str(PARENT/'simulation/campaign.json'),parent_campaign_sha256=sha(PARENT/'simulation/campaign.json'),
        additional_surfaces=surfaces,scope='100 newly authorized paired Libo observations; existing205 parent conditions not rerun; same control and numerical standard')
    dump(SIM/'campaign.json',c)
    check();print('PREPARED100: two existing terrain surfaces, identical ten spawns, five waves,10 four-worker batches',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','check','run','worker']);p.add_argument('--job');p.add_argument('--attempt',default='v1');p.add_argument('--keep');a=p.parse_args()
    if a.mode=='prepare':prepare()
    elif a.mode=='check':check()
    elif a.mode=='run':check();original.run()
    else:original.worker(a.job,a.attempt,None if a.keep is None else list(map(int,a.keep.split(','))))
