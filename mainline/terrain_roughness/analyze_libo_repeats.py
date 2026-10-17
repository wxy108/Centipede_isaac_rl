"""Paired spawn statistics; retain every complete stuck/negative-motion trial."""
from pathlib import Path
import os
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
import json,hashlib,ast,argparse,shutil
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent;MAIN=ROOT.parent;BASE=ROOT/'libo_spawn_repeats';SIM=BASE/'simulation';OUT=BASE/'results'
WAVES=[1.5,2,2.5,3,3.5];COLORS=['#2474b5','#df7c13','#26915a','#bd4545','#8862b2']
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,r):Path(p).write_text(json.dumps(r,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')

def validate_startup(d,n):
    rb=read(d/'readback.json');plant=read(d/'matched_plant_check.json');threads=read(d/'thread_settings.json')
    assert rb['dt_s']==.0005 and rb['solver']=='TGS' and not rb['gpu_dynamics']
    assert threads['tasking']==threads['physics']==2 and threads['torch_threads']==1 and read(d/'articulation_binding.json')['row_order_verified']
    assert plant['passed'] and plant['complete_uniform_173_joint_stiffness'] and plant['complete_uniform_173_joint_damping']
    assert all(x['position']==64 and x['velocity']==16 for x in read(d/'iteration_readback.json'))
    offsets=read(d/'native_contact_offsets.json')
    np.testing.assert_allclose(offsets['contact'],.00002,atol=1e-10,rtol=0);np.testing.assert_allclose(offsets['rest'],0.,atol=1e-12,rtol=0)
    pd=read(MAIN/'pd_wave4_firm20.json')['absolute_joint_pd'];jn=rb['joint_names'];assert len(jn)==173 and set(jn)==set(pd)
    with np.load(d/'backend.npz') as z:
        kp=z['kp'];kd=z['kd'];mass=z['masses'].astype(float);ex=np.array([pd[k] for k in jn],dtype=kp.dtype)
        np.testing.assert_array_equal(kp,np.repeat(ex[None,:,0],n,axis=0));np.testing.assert_array_equal(kd,np.repeat(ex[None,:,1],n,axis=0))
    return mass

def extract_run(d,n,expected_yaw=None):
    mass=validate_startup(d,n);result=read(d/'results.json')
    assert result['complete'] and result['finite'] and result['whole_batch_finite'] and result['actual_duration_s']>=6
    assert not read(d/'native_error_audit.json')['errors']
    with np.load(d/'trajectory.npz') as z:
        mask=z['t']<=6+1e-9;times=z['t'][mask];pose=z['pose'][mask].astype(float);names=z['body_names'].tolist()
        q=z['q'][mask];v=z['v'][mask];vel=z['body_velocity'][mask]
        assert np.isfinite(pose).all() and np.isfinite(q).all() and np.isfinite(v).all() and np.isfinite(vel).all()
    assert pose.shape==(601,n,251,7);np.testing.assert_allclose(times,np.arange(601)*.01,atol=1e-12,rtol=0)
    fields=read(d/'effective_fields_after_pd.json')['attributes'];cf={a['path'].rsplit('/',1)[-1]:a for a in fields if a['name']=='physics:centerOfMass'}
    assert set(cf)==set(names) and all(x['authored'] for x in cf.values())
    local=np.array([ast.literal_eval(cf[k]['value']) for k in names],float)
    quat=pose[...,3:];norm=np.linalg.norm(quat,axis=-1,keepdims=True);assert np.max(abs(norm-1))<3e-6;quat/=norm
    uv=np.cross(quat[...,1:],local);rotated=local+2*(quat[...,:1]*uv+np.cross(quat[...,1:],uv))
    subset=quat[[0,300,600]];chk=Rotation.from_quat(subset[...,[1,2,3,0]].reshape(-1,4)).apply(np.broadcast_to(local,subset.shape[:-1]+(3,)).reshape(-1,3))
    assert np.max(abs(chk-rotated[[0,300,600]].reshape(-1,3)))<1e-14
    com=((pose[...,:3]+rotated)*mass[None,:,:,None]).sum(2)/mass.sum(1)[None,:,None]
    f=pose[0,:,names.index('link_body_0'),:2]-pose[0,:,names.index('link_body_20'),:2];f/=np.linalg.norm(f,axis=-1,keepdims=True)
    if expected_yaw is not None:
        yaw=np.deg2rad(expected_yaw);np.testing.assert_allclose(f,np.tile([-np.cos(yaw),-np.sin(yaw)],(n,1)),atol=2e-6,rtol=0)
    left=np.stack([-f[:,1],f[:,0]],axis=1);delta=(com[...,:2]-com[0,:,:2])*1000
    fwd=(delta*f[None]).sum(2);lateral=(delta*left[None]).sum(2)
    # Independent projection through each spawn's rotated initial coordinate frame.
    if expected_yaw is not None:
        ref=-delta[...,0]*np.cos(yaw)-delta[...,1]*np.sin(yaw)
        np.testing.assert_allclose(fwd,ref,atol=.002,rtol=0)
    return dict(forward=fwd,lateral=lateral,com=com,forward_xy=f,shared=(names,local,mass[0]),
        maximum_joint_speed_rad_s=np.max(abs(v),axis=(0,2)),maximum_body_speed_m_s=np.max(np.linalg.norm(vel[...,:3].astype(float),axis=-1),axis=(0,2)),wall_s=result['wall_s'])

def implementation_comparison():
    previous=ROOT/'height_amplitude_4mm/simulation/executed_source/failure_capture_functions.py'
    current=SIM/'executed_source/adapted_functions.py'
    def functions(path):return {n.name:n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n,ast.FunctionDef)}
    old,new=functions(previous),functions(current)
    checks={name:ast.dump(old[name])==ast.dump(new[name]) for name in ['_create_environments','_load_commands']}
    def control_prefix(fn):
        loop=next(n for n in fn.body if isinstance(n,ast.For) and isinstance(n.target,ast.Name) and n.target.id=='step_index')
        stop=next(i for i,n in enumerate(loop.body) if isinstance(n,ast.If) and 'isfinite' in ast.dump(n.test))
        return ast.dump(ast.Module(body=loop.body[:stop],type_ignores=[]))
    checks['physics_control_feedback_loop_prefix']=control_prefix(old['run_trial'])==control_prefix(new['run_trial'])
    assert all(checks.values()),checks
    return dict(reference_path=str(previous),reference_sha256=sha(previous),executed_path=str(current),executed_sha256=sha(current),AST_unchanged=checks,
        declared_changes='New input rows/spawns; full-PD plant verification independent of historical row index; shape-envelope guard and failure-state recording; scheduling and peer-only recovery.')

def analyze():
    c=read(SIM/'campaign.json');sv=read(SIM/'supervisor.json');assert sv['complete']
    for p,h in c['pins'].items():assert sha(p)==h,p
    jobs={j['name']:j for j in c['jobs']};cells=[];failed={};series={};sources={str(SIM/'campaign.json'):sha(SIM/'campaign.json')};shared=None;runs=[]
    for record in sv['finished']:
        j=jobs[record['job']];d=SIM/'runs'/record['attempt']/j['name'];a=read(d/'active.json');ids=a['source_ids']
        source_cells={x['source_id']:x for x in j['cells']};selected=[source_cells[i] for i in ids]
        validate_startup(d,len(ids))
        contract=read(d/'input_contract_validation.json')
        assert contract['full_table_exact'] and contract['flat_cluster_amplitudes_unchanged']
        assert contract['fixed_pd_sha256']==c['fixed_pd_sha256'] and contract['profile_sha256']==sha(MAIN/'trial_profile.json')
        provenance=read(d/'implementation_provenance.json')
        for path,h in provenance['execution_sources'].items():assert sha(path)==h,path
        amp=read(d/'amplitude_modulation.json');assert amp['other_targets_bitexact'] and amp['limits_passed'] and amp['cells']==selected
        for file in ['active.json','backend.npz','readback.json','matched_plant_check.json','amplitude_modulation.json','implementation_provenance.json']:
            if (d/file).exists():sources[str(d/file)]=sha(d/file)
        if record['exit']!=0:
            if (d/'BOUNDARY_FAILURE.json').exists():
                failure=read(d/'BOUNDARY_FAILURE.json');off=failure['offender_source_ids'];reason='boundary_guard'
                for env in failure['environments']:
                    if env['source_id'] in off and env['max_body_linear_speed_m_s']>1e6:reason='state_divergence'
            else:
                failure=read(d/'NUMERICAL_FAILURE.json');off=[ids[i] for i in failure['environments']];reason='nonfinite_state'
            for i in off:
                cell=source_cells[i];failed[(j['height_scale'],cell['wave'],cell['spawn_index'])]=dict(cell,status=reason,run_path=str(d),failure=failure)
            runs.append(dict(job=j['name'],attempt=record['attempt'],source_ids=ids,complete=False,path=str(d),reason=reason,native_startup_PD_exact=True,startup_config_verified=True,native_final_error_audit_available=(d/'native_error_audit.json').exists(),offender_source_ids=off,peer_source_ids=[i for i in ids if i not in off]))
            for file in ['BOUNDARY_FAILURE.json','NUMERICAL_FAILURE.json','trajectory_partial.npz','failure_state.npz','trajectory.npz','worker.log']:
                if (d/file).exists():sources[str(d/file)]=sha(d/file)
            continue
        n=len(ids);yaw=c['spawns'][j['spawn_index']]['start_yaw_deg'];ex=extract_run(d,n,yaw)
        if shared is None:shared=ex['shared']
        assert shared[0]==ex['shared'][0];np.testing.assert_array_equal(shared[1],ex['shared'][1]);np.testing.assert_array_equal(shared[2],ex['shared'][2])
        guard=read(d/'collision_guard.json');assert guard['minimum_all_step_collision_margin_m']>.015
        manifest=read(d/'manifest.json')['rows'];sp=c['spawns'][j['spawn_index']]
        assert all(row['start_xy']==sp['start_xy_m'] and row['start_yaw_deg']==yaw for row in manifest)
        sources[str(d/'trajectory.npz')]=sha(d/'trajectory.npz')
        for e,cell in enumerate(selected):
            fwd=ex['forward'][:,e];lat=ex['lateral'][:,e]
            out=dict(cell,status='complete',run=j['name'],attempt=record['attempt'],run_path=str(d),environment_index=e,
                start_xy_m=sp['start_xy_m'],start_yaw_deg=yaw,forward_xy=ex['forward_xy'][e].tolist(),
                forward_0_6_mm=float(fwd[-1]),forward_4_6_mm=float(fwd[-1]-fwd[400]),lateral_0_6_mm=float(lat[-1]),
                maximum_joint_speed_rad_s=float(ex['maximum_joint_speed_rad_s'][e]),maximum_body_speed_m_s=float(ex['maximum_body_speed_m_s'][e]))
            cells.append(out);series[f"h{cell['height_scale']:g}_w{cell['wave']:g}_s{cell['spawn_index']:02d}"]=fwd
        runs.append(dict(job=j['name'],attempt=record['attempt'],source_ids=ids,complete=True,path=str(d),native_PD_exact=True,minimum_all_step_collision_margin_m=guard['minimum_all_step_collision_margin_m'],wall_s=ex['wall_s']))
    keys={(x['height_scale'],x['wave'],x['spawn_index']) for x in cells};assert len(keys)==len(cells) and not(keys & set(failed))
    assert len(cells)+len(failed)==205
    flat=[x for x in cells if x['height_scale']==0];assert len(flat)==5
    legacy=[]
    old=read(MAIN/'campaign.json')
    for job in [j for j in old['jobs'] if j['name'].startswith('flat')]:
        d=MAIN/'runs/v4'/job['name'];ex=extract_run(d,3,0.)
        sources[str(d/'trajectory.npz')]=sha(d/'trajectory.npz')
        for e,cell in enumerate(job['cells']):
            if cell['wave']>3.5:continue
            new=next(x for x in flat if x['wave']==cell['wave'])
            legacy.append(dict(wave=cell['wave'],old_forward_0_6_mm=float(ex['forward'][-1,e]),new_forward_0_6_mm=new['forward_0_6_mm'],
                new_minus_old_mm=new['forward_0_6_mm']-float(ex['forward'][-1,e]),old_heading_deg=0,new_heading_deg=90,
                limitation='Translation and orientation both changed; this is a control comparison, not isolated numerical yaw sensitivity.'))
    summaries=[]
    for scale in c['height_scales']:
        for wave in WAVES:
            rows=[x for x in cells if x['height_scale']==scale and x['wave']==wave];fails=[x for x in failed.values() if x['height_scale']==scale and x['wave']==wave]
            assert len(rows)+len(fails)==10
            flatref=next(x for x in flat if x['wave']==wave)
            summary=dict(height_scale=scale,wave=wave,fixed_pair_angle_rms_deg=next(x for x in c['jobs'] if x['height_scale']==scale)['cells'][0]['fixed_pair_angle_rms_deg'],
                n_valid=len(rows),n_failed=len(fails),failure_fraction=len(fails)/10,spawn_indices=[x['spawn_index'] for x in rows],flat_reference_0_6_mm=flatref['forward_0_6_mm'])
            for key in ['forward_0_6_mm','forward_4_6_mm','lateral_0_6_mm']:
                vals=np.array([x[key] for x in rows]);summary[key]=dict(mean=float(vals.mean()) if len(vals) else None,std=float(vals.std(ddof=1)) if len(vals)>1 else None,min=float(vals.min()) if len(vals) else None,max=float(vals.max()) if len(vals) else None,values=vals.tolist())
            summary['mean_change_from_rotated_flat_mm']=None if not rows else summary['forward_0_6_mm']['mean']-flatref['forward_0_6_mm']
            summaries.append(summary)
    paired=[]
    for wave in WAVES:
        ref={x['spawn_index']:x for x in cells if x['height_scale']==.25 and x['wave']==wave}
        for scale in c['height_scales'][1:]:
            values=[x['forward_0_6_mm']-ref[x['spawn_index']]['forward_0_6_mm'] for x in cells if x['height_scale']==scale and x['wave']==wave and x['spawn_index'] in ref]
            paired.append(dict(wave=wave,height_scale=scale,reference_height_scale=.25,n_pairs=len(values),mean_difference_mm=float(np.mean(values)) if values else None,std_difference_mm=float(np.std(values,ddof=1)) if len(values)>1 else None,values_mm=values))
    return dict(complete=True,condition_count=205,rough_condition_count=200,new_flat_condition_count=5,valid_condition_count=len(cells),failed_condition_count=len(failed),
        worker_attempt_count=len(sv['finished']),condition_execution_count=sum(len(x['source_ids']) for x in runs),peer_condition_reexecutions=sum(len(x['source_ids']) for x in runs)-205,
        cells=cells,failed_cells=list(failed.values()),runs=runs,summaries=summaries,paired_height_comparisons=paired,flat_controls=flat,legacy_flat_comparison=legacy,
        spawns=c['spawns'],height_scales=c['height_scales'],waves=WAVES,roughness_definition=c['roughness_definition'],supervisor_wall_s=sv['elapsed_s'],source_hashes=sources,implementation_comparison=implementation_comparison(),
        displacement_definition='Mass-weighted COM including authored local offsets; XY translation projected on EACH trial initial tail-to-head direction. Lateral projection also saved.',
        acceptance='Every complete finite, native-error-free six-second run retained, including stuck, zero or reverse forward motion. No performance filtering.',
        statistics='Descriptive mean and sample standard deviation ddof=1 across ten preselected location/heading conditions; failed/incomplete cases have no six-second value.',
        limitations=['One fixed Libo layout; spatial conditions are not independent terrain realizations.','Positions conditional on heading-specific safe corridor based on known flat motion; not uniform over the whole terrain.','CPGphase0 fixed; position and heading vary together, so their separate variance contributions are not identified.','Six seconds includes settling; final two seconds is supplementary, not verified steady state.','One new flat run per wave plus historical reference; no flat variance estimate and no ten-way paired rough/flat contrast.','Failure-conditioned means describe completed cases; inspect failure fractions and all raw points.','Interrupted peers were reexecuted in partial batches with only remaining real conditions; native environment indices reindexed and per-worker environment count changed. Same spawn, commands, PD, physics, two threads and collision isolation retained; these are not additional samples.']),series

def plots(r):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    images=[]
    for key,name,title,ylabel in [
        ('forward_0_6_mm','mean_std_vs_roughness.png','Libo: paired spawn observations, mean and standard deviation','Forward displacement, 0-6 s (mm)'),
        ('forward_4_6_mm','mean_std_4_6_vs_roughness.png','Libo: displacement during the final two seconds','Forward displacement, 4-6 s (mm)')]:
        fig,ax=plt.subplots(figsize=(12,7));fig.subplots_adjust(left=.10,right=.98,bottom=.23,top=.78)
        fig.suptitle(title,fontsize=17,weight='bold',y=.965)
        for wave,color in zip(WAVES,COLORS):
            sums=sorted([s for s in r['summaries'] if s['wave']==wave],key=lambda s:s['height_scale'])
            xx=[s['fixed_pair_angle_rms_deg'] for s in sums];yy=[s[key]['mean'] for s in sums];std=[s[key]['std'] for s in sums]
            ax.errorbar(xx,yy,yerr=std,fmt='-o',color=color,capsize=4,lw=2,label=f'Wave {wave:g}: mean +/- SD')
            for summary in sums:
                ax.scatter([summary['fixed_pair_angle_rms_deg']]*len(summary[key]['values']),summary[key]['values'],s=20,color=color,alpha=.40,zorder=3)
                if summary['n_valid']<10 and summary[key]['mean'] is not None:
                    ax.annotate(f'n={summary["n_valid"]}/10',(summary['fixed_pair_angle_rms_deg'],summary[key]['mean']),xytext=(6,-15),textcoords='offset points',fontsize=9,color=color)
            flat=next(x for x in r['flat_controls'] if x['wave']==wave)
            ax.scatter([0],[flat[key]],marker='D',s=55,color=color,zorder=4)
        ax.axhline(0,color='#67737d',lw=.8,alpha=.6);ax.grid(alpha=.16);ax.set(xlabel='Fixed 38-foot / 703-pair angle RMS (deg)',ylabel=ylabel)
        fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.55,.91),ncol=3,frameon=False,fontsize=10)
        fig.text(.10,.145,'Points: all complete trials, including stuck or reverse motion. Bars: sample SD (ddof=1). Flat diamonds: one control per wave.',fontsize=9.5,color='#52616f')
        fig.text(.10,.09,'Rough heights x0.25, x0.5, x0.75, x1; ten shared (position, heading) conditions. Cutoff 4 mm; grid 0.5 mm.',fontsize=10,color='#52616f')
        fig.text(.10,.04,f'Complete observations {r["valid_condition_count"]}/205; failed {r["failed_condition_count"]}. Frozen gait A/B, full173 PD, 0.5 ms physics / 1 ms CPG.',fontsize=9.5,color='#52616f')
        path=OUT/name;fig.savefig(path,dpi=160,facecolor='white');plt.close(fig)
        with Image.open(path) as im:im.load()
        images.append(dict(path=str(path),sha256=sha(path)))
    fig,ax=plt.subplots(figsize=(12,7));fig.subplots_adjust(left=.10,right=.98,bottom=.20,top=.80)
    fig.suptitle('Libo: forward displacement vs. wave number',fontsize=18,weight='bold',y=.965)
    palette=['#235789','#3f8d8d','#94b447','#d79138']
    for scale,color in zip(r['height_scales'],palette):
        rows=sorted([x for x in r['summaries'] if x['height_scale']==scale],key=lambda x:x['wave'])
        ax.errorbar([x['wave'] for x in rows],[x['forward_0_6_mm']['mean'] for x in rows],yerr=[x['forward_0_6_mm']['std'] for x in rows],fmt='-o',lw=2,capsize=4,color=color,label=f'Height x{scale:g}, mean +/- SD')
    flat=sorted(r['flat_controls'],key=lambda x:x['wave']);ax.plot([x['wave'] for x in flat],[x['forward_0_6_mm'] for x in flat],'--D',lw=1.5,color='#575f68',label='Flat: one rotated control per wave')
    ax.set(xlabel='Wave number',ylabel='Forward displacement, 0-6 s (mm)');ax.set_xticks(WAVES,[f'{x:g}' for x in WAVES]);ax.grid(alpha=.17);ax.axhline(0,color='#67737d',lw=.8,alpha=.6)
    fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.55,.915),ncol=3,frameon=False,fontsize=10)
    fig.text(.10,.11,'Each projection follows that trial initial headward axis; all terrain heights use the same ten spawn conditions.',fontsize=10,color='#52616f')
    fig.text(.10,.045,'Six-second actual Isaac CPU PhysX observations. SD describes location/heading variation on one fixed Libo layout.',fontsize=10,color='#52616f')
    path=OUT/'mean_std_vs_wave_number.png';fig.savefig(path,dpi=160,facecolor='white');plt.close(fig)
    with Image.open(path) as im:im.load()
    images.append(dict(path=str(path),sha256=sha(path)))
    return images

if __name__=='__main__':
    assert not OUT.exists(),'Preserve completed analysis'
    r,series=analyze();OUT.mkdir();shutil.copy2(__file__,OUT/'analysis_source.py')
    r['analysis_source_sha256']=sha(__file__);r['figures']=plots(r)
    np.savez_compressed(OUT/'forward_timeseries.npz',t_s=np.arange(601)/100,**series)
    dump(OUT/'RESULTS.json',r)
    print(json.dumps(dict(valid=r['valid_condition_count'],failed=r['failed_condition_count'],elapsed_physics_s=r['supervisor_wall_s'])),flush=True)
