"""User-defined one-point omission and unified markers; preserve all raw evidence."""
from pathlib import Path
import os
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
import json,hashlib,shutil,copy,argparse,time
import numpy as np
import analyze_libo_repeats as previous
ROOT=Path(__file__).resolve().parent;MAIN=ROOT.parent;BASE=ROOT/'libo_spawn_repeats'
EXT=BASE/'extension_1p25_1p5';SIM=EXT/'simulation';OUT=BASE/'trimmed_results'
WAVES=previous.WAVES;COLORS=previous.COLORS
read,sha,dump=previous.read,previous.sha,previous.dump

def extension():
    c=read(SIM/'campaign.json');sv=read(SIM/'supervisor.json');assert sv['complete']
    for p,h in c['pins'].items():assert sha(p)==h,p
    parent=read(BASE/'simulation/campaign.json');assert c['spawns']==parent['spawns'] and c['profile']==parent['profile']
    assert sha(SIM/'executed_source/adapted_functions.py')==sha(BASE/'simulation/executed_source/adapted_functions.py')
    jobs={j['name']:j for j in c['jobs']};cells=[];failures={};runs=[];series={};sources={str(SIM/'campaign.json'):sha(SIM/'campaign.json')}
    for rec in sv['finished']:
        j=jobs[rec['job']];d=SIM/'runs'/rec['attempt']/j['name'];ids=read(d/'active.json')['source_ids'];source={x['source_id']:x for x in j['cells']};selected=[source[i] for i in ids]
        previous.validate_startup(d,len(ids));contract=read(d/'input_contract_validation.json')
        assert contract['full_table_exact'] and contract['flat_cluster_amplitudes_unchanged'] and contract['fixed_pd_sha256']==c['fixed_pd_sha256'] and contract['profile_sha256']==c['profile_sha256']
        amp=read(d/'amplitude_modulation.json');assert amp['other_targets_bitexact'] and amp['limits_passed'] and amp['cells']==selected
        for p,h in read(d/'implementation_provenance.json')['execution_sources'].items():assert sha(p)==h,p
        for name in ['active.json','backend.npz','readback.json','matched_plant_check.json','input_contract_validation.json','implementation_provenance.json','amplitude_modulation.json']:
            sources[str(d/name)]=sha(d/name)
        if rec['exit']!=0:
            if (d/'BOUNDARY_FAILURE.json').exists():
                f=read(d/'BOUNDARY_FAILURE.json');off=f['offender_source_ids'];why={x['source_id']:('state_divergence' if x['max_body_linear_speed_m_s']>1000 or x['max_abs_joint_velocity_rad_s']>1e6 else 'boundary_guard') for x in f['environments'] if x['source_id'] in off}
            else:
                f=read(d/'NUMERICAL_FAILURE.json');off=[ids[i] for i in f['environments']];why={i:'nonfinite_state' for i in off}
            for i in off:
                cell=source[i];key=(cell['height_scale'],cell['wave'],cell['spawn_index']);assert key not in failures
                failures[key]=dict(cell,status=why[i],run_path=str(d),failure=f)
            runs.append(dict(job=j['name'],attempt=rec['attempt'],source_ids=ids,complete=False,path=str(d),offender_source_ids=off,peer_source_ids=[i for i in ids if i not in off],native_startup_verified=True,native_final_error_audit_available=(d/'native_error_audit.json').exists()))
            for name in ['BOUNDARY_FAILURE.json','NUMERICAL_FAILURE.json','trajectory_partial.npz','failure_state.npz','trajectory.npz','worker.log']:
                if (d/name).exists():sources[str(d/name)]=sha(d/name)
            continue
        sp=c['spawns'][j['spawn_index']];ex=previous.extract_run(d,len(ids),sp['start_yaw_deg'])
        manifest=read(d/'manifest.json')['rows'];assert all(x['start_xy']==sp['start_xy_m'] and x['start_yaw_deg']==sp['start_yaw_deg'] for x in manifest)
        guard=read(d/'collision_guard.json');assert guard['minimum_all_step_collision_margin_m']>.015
        sources[str(d/'trajectory.npz')]=sha(d/'trajectory.npz')
        for e,cell in enumerate(selected):
            fwd=ex['forward'][:,e];lat=ex['lateral'][:,e]
            cells.append(dict(cell,status='complete',run=j['name'],attempt=rec['attempt'],run_path=str(d),environment_index=e,
                start_xy_m=sp['start_xy_m'],start_yaw_deg=sp['start_yaw_deg'],forward_xy=ex['forward_xy'][e].tolist(),
                forward_0_6_mm=float(fwd[-1]),forward_4_6_mm=float(fwd[-1]-fwd[400]),lateral_0_6_mm=float(lat[-1]),
                maximum_joint_speed_rad_s=float(ex['maximum_joint_speed_rad_s'][e]),maximum_body_speed_m_s=float(ex['maximum_body_speed_m_s'][e])))
            series[f"h{cell['height_scale']:g}_w{cell['wave']:g}_s{cell['spawn_index']:02d}"]=fwd
        runs.append(dict(job=j['name'],attempt=rec['attempt'],source_ids=ids,complete=True,path=str(d),native_PD_exact=True,minimum_all_step_collision_margin_m=guard['minimum_all_step_collision_margin_m'],wall_s=ex['wall_s']))
    keys={(x['height_scale'],x['wave'],x['spawn_index']) for x in cells};assert len(keys)==len(cells) and not(keys & set(failures)) and len(cells)+len(failures)==100
    for h in c['height_scales']:
        for w in WAVES:assert sum(x['height_scale']==h and x['wave']==w for x in cells)+sum(x['height_scale']==h and x['wave']==w for x in failures.values())==10
    r=dict(complete=True,condition_count=100,rough_condition_count=100,new_flat_condition_count=0,valid_condition_count=len(cells),failed_condition_count=len(failures),
        cells=cells,failed_cells=list(failures.values()),runs=runs,waves=WAVES,height_scales=c['height_scales'],spawns=c['spawns'],source_hashes=sources,
        worker_attempt_count=len(runs),condition_execution_count=sum(len(x['source_ids']) for x in runs),peer_condition_reexecutions=sum(len(x['source_ids']) for x in runs)-100,
        supervisor_wall_s=sv['elapsed_s'],adapted_physics_source_bitexact_parent=True,raw_no_performance_filtering=True,
        guard_failure_classification='Only already-guard-stopped cases: body linear speed >1000 m/s or joint speed >1e6 rad/s labels obvious nonphysical state divergence. No velocity threshold applied to complete trials or outlier selection.')
    d=EXT/'results';assert not d.exists(),'Preserve completed raw extension analysis';d.mkdir();shutil.copy2(__file__,d/'analysis_source.py')
    np.savez_compressed(d/'forward_timeseries.npz',t_s=np.arange(601)/100,**series);dump(d/'RESULTS.json',r)
    return r

def stats(rows,key):
    a=np.array([r[key] for r in rows],float)
    return dict(n=len(rows),mean=float(a.mean()) if len(a) else None,std=float(a.std(ddof=1)) if len(a)>1 else None,values=a.tolist(),spawn_indices=[r['spawn_index'] for r in rows])

def omit_one(rows):
    if not rows:return [],None,None
    median=float(np.median([x['forward_0_6_mm'] for x in rows]))
    chosen=sorted(rows,key=lambda x:(-abs(x['forward_0_6_mm']-median),x['spawn_index']))[0]
    retained=[r for r in rows if r['spawn_index']!=chosen['spawn_index']]
    assert len(retained)==len(rows)-1
    return retained,dict(chosen,selection_median_0_6_mm=median,absolute_distance_from_median_mm=abs(chosen['forward_0_6_mm']-median)),median

def summarize(include_extension):
    parent=read(BASE/'results/RESULTS.json');assert parent['complete'] and parent['figure_review']['passed']
    parts=[parent];sources={str(BASE/'results/RESULTS.json'):sha(BASE/'results/RESULTS.json')}
    if include_extension:
        p=EXT/'results/RESULTS.json';parts.append(read(p) if p.exists() else extension());sources[str(p)]=sha(p)
    cells=sum([x['cells'] for x in parts],[]);failed=sum([x['failed_cells'] for x in parts],[]);heights=sorted({x['height_scale'] for x in cells+failed if x['height_scale']>0})
    flat=sorted([x for x in cells if x['height_scale']==0],key=lambda x:x['wave']);assert len(flat)==5
    summaries=[];excluded=[];retained=[]
    for h in heights:
        for w in WAVES:
            rows=sorted([x for x in cells if x['height_scale']==h and x['wave']==w],key=lambda x:x['spawn_index']);bad=[x for x in failed if x['height_scale']==h and x['wave']==w]
            assert len(rows)+len(bad)==10;keep,drop,median=omit_one(rows);retained.extend(keep)
            if drop:excluded.append(drop)
            roughness=next(x for x in cells+failed if x['height_scale']==h and x['wave']==w)['fixed_pair_angle_rms_deg']
            summaries.append(dict(height_scale=h,wave=w,fixed_pair_angle_rms_deg=roughness,n_complete_before=len(rows),n_failed=len(bad),n_excluded=int(drop is not None),n_retained=len(keep),excluded_spawn_index=None if drop is None else drop['spawn_index'],
                forward_0_6_mm=stats(keep,'forward_0_6_mm'),forward_4_6_mm=stats(keep,'forward_4_6_mm'),raw_forward_0_6_mm=stats(rows,'forward_0_6_mm')))
    paired=[]
    for w in WAVES:
        ref={x['spawn_index']:x for x in retained if x['height_scale']==.25 and x['wave']==w}
        for h in heights[1:]:
            points=[dict(spawn_index=x['spawn_index'],forward_0_6_mm=x['forward_0_6_mm']-ref[x['spawn_index']]['forward_0_6_mm']) for x in retained if x['height_scale']==h and x['wave']==w and x['spawn_index'] in ref]
            paired.append(dict(wave=w,height_scale=h,reference_height_scale=.25,**stats(points,'forward_0_6_mm')))
    return dict(complete=include_extension,extension_complete=include_extension,planned_total_condition_count=305,plotted_condition_count=sum(x['condition_count'] for x in parts),raw_complete_count=len(cells),failed_condition_count=len(failed),excluded_for_statistics_count=len(excluded),retained_rough_count=len(retained),
        retained_total_count=len(retained)+5,height_scales=heights,waves=WAVES,flat_controls=flat,summaries=summaries,excluded_observations=excluded,failed_cells=failed,paired_height_comparisons=paired,
        raw_result_sources=sources,extension_supervisor_wall_s=None if len(parts)==1 else parts[1]['supervisor_wall_s'],
        selection_rule='For each rough height x wave group omit exactly one COMPLETE 0-6s observation farthest from the group median; ties choose lowest spawn index. This same retained subset is used for 0-6s and4-6s. Flat single observations retained; no flat SD estimated. No claim of statistically proven outliers.',
        source_data_policy='All original complete observations and numerical/boundary failures preserved in raw result files. User-requested omission only affects this derived display/statistics.',
        limitations=parent['limitations']+['One-point omission is a requested descriptive sensitivity view, not evidence that the omitted valid trial is physically wrong. It can remove a legitimate stuck or reverse-motion observation. Per-group omission changes paired sample intersections.'])

def pale(color):
    import matplotlib.colors as mc
    return mc.to_rgba(color,.30)

def plots(r):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    images=[]
    for key,name,title,ylabel in [
        ('forward_0_6_mm','mean_std_vs_roughness.png','Libo: forward displacement vs terrain roughness','Forward displacement, 0-6 s (mm)'),
        ('forward_4_6_mm','mean_std_4_6_vs_roughness.png','Libo: forward displacement during the final two seconds','Forward displacement, 4-6 s (mm)')]:
        fig,ax=plt.subplots(figsize=(12,7));fig.subplots_adjust(left=.10,right=.98,bottom=.23,top=.79);fig.suptitle(title,fontsize=17,weight='bold',y=.965)
        for w,color in zip(WAVES,COLORS):
            rows=sorted([x for x in r['summaries'] if x['wave']==w],key=lambda x:x['height_scale']);flat=next(x for x in r['flat_controls'] if x['wave']==w)
            xx=[0.]+[x['fixed_pair_angle_rms_deg'] for x in rows];yy=[flat[key]]+[np.nan if x[key]['mean'] is None else x[key]['mean'] for x in rows]
            sd=[np.nan]+[np.nan if x[key]['std'] is None else x[key]['std'] for x in rows]
            ax.errorbar(xx,yy,yerr=sd,fmt='none',ecolor=pale(color),elinewidth=1.6,capsize=4,capthick=1.4,zorder=2)
            ax.plot(xx,yy,'-o',color=color,lw=2,ms=6,label=f'Wave {w:g}',zorder=3)
        ax.axhline(0,color='#67737d',lw=.8,alpha=.5);ax.grid(alpha=.14);ax.set(xlabel='Fixed 38-foot / 703-pair angle RMS (deg)',ylabel=ylabel)
        fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.54,.91),ncol=5,frameon=False,fontsize=10)
        footers(fig,r)
        p=OUT/name;fig.savefig(p,dpi=160,facecolor='white');plt.close(fig)
        with Image.open(p) as im:im.load()
        images.append(dict(path=str(p),sha256=sha(p)))
    fig,ax=plt.subplots(figsize=(12,7));fig.subplots_adjust(left=.10,right=.98,bottom=.23,top=.77);fig.suptitle('Libo: forward displacement vs wave number',fontsize=17,weight='bold',y=.965)
    palette=['#575f68','#235789','#3f8d8d','#94b447','#d79138','#bd4545','#8862b2']
    for h,color in zip([0.,*r['height_scales']],palette):
        if h==0:rows=sorted(r['flat_controls'],key=lambda x:x['wave']);yy=[x['forward_0_6_mm'] for x in rows];sd=[np.nan]*len(rows)
        else:
            rows=sorted([x for x in r['summaries'] if x['height_scale']==h],key=lambda x:x['wave']);yy=[np.nan if x['forward_0_6_mm']['mean'] is None else x['forward_0_6_mm']['mean'] for x in rows];sd=[np.nan if x['forward_0_6_mm']['std'] is None else x['forward_0_6_mm']['std'] for x in rows]
        xx=[x['wave'] for x in rows];ax.errorbar(xx,yy,yerr=sd,fmt='none',ecolor=pale(color),elinewidth=1.6,capsize=4,capthick=1.4,zorder=2);ax.plot(xx,yy,'-o',color=color,lw=2,ms=6,label=f'Height x{h:g}',zorder=3)
    ax.set(xlabel='Wave number',ylabel='Forward displacement, 0-6 s (mm)');ax.set_xticks(WAVES);ax.axhline(0,color='#67737d',lw=.8,alpha=.5);ax.grid(alpha=.14)
    fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.54,.915),ncol=4,frameon=False,fontsize=10);footers(fig,r)
    p=OUT/'mean_std_vs_wave_number.png';fig.savefig(p,dpi=160,facecolor='white');plt.close(fig)
    with Image.open(p) as im:im.load()
    images.append(dict(path=str(p),sha256=sha(p)));return images

def footers(fig,r):
    fig.text(.10,.145,'Circles joined by lines: means. Pale error bars: sample SD. One farthest-from-median point omitted per rough group.',fontsize=9.2,color='#52616f')
    fig.text(.10,.09,'Selection uses 0-6 s; the same cases are retained for 4-6 s. Height x0 is the single flat reference; no SD estimated.',fontsize=9.2,color='#52616f')
    state='Six rough heights complete' if r['extension_complete'] else 'Heights x1.25 / x1.5 currently running; shown here: x0.25 / x0.5 / x0.75 / x1'
    fig.text(.10,.04,f'{state}. Frozen A/B, full173 PD, 4 mm cutoff; 0.5 ms physics / 1 ms CPG.',fontsize=9.0,color='#52616f')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--final',action='store_true');a=p.parse_args()
    if (OUT/'SUMMARY.json').exists():assert not read(OUT/'SUMMARY.json')['extension_complete'],'Preserve completed derived summary'
    r=summarize(a.final);OUT.mkdir(exist_ok=True);shutil.copy2(__file__,OUT/'analysis_source.py');r['analysis_source_sha256']=sha(__file__);r['figures']=plots(r);dump(OUT/'SUMMARY.json',r)
    print(json.dumps(dict(extension_complete=r['extension_complete'],raw_complete=r['raw_complete_count'],excluded=r['excluded_for_statistics_count'],retained=r['retained_total_count'],failed=r['failed_condition_count'])),flush=True)
