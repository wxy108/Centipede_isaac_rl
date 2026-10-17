"""Analysis of actual saved Isaac erasure outcomes and existing rough data; no physics."""
from pathlib import Path
import os,json,hashlib,itertools
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
import numpy as np
ROOT=Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
BASE=ROOT/'contact_erasure_light_2026-09-30';SIM=BASE/'simulation_v2';OUT=BASE/'results'
import sys
sys.path.insert(0,str(ROOT))
from analyze_libo_repeats import extract_run,validate_startup
WAVES=[1.5,2,2.5,3,3.5];BINS=[.01,.02,.05];PS=[i/10 for i in range(11)]
COLORS=['#2474b5','#df7c13','#26915a','#bd4545','#8862b2']
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,x):Path(p).write_text(json.dumps(x,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
def meanstd(values):
    values=[float(v) for v in values]
    return dict(mean=float(np.mean(values)) if values else None,std=float(np.std(values,ddof=1)) if len(values)>1 else None,values=values)
def key(cell):return (0. if cell['p'] in [0,1] else cell['erase_bin_s'],cell['p'],cell['wave'],cell['mask_seed'])

def invert(grid,values,target):
    """All roots of adjacent valid piecewise-linear segments, with plateaus explicit."""
    roots=[]
    for i,(p,y) in enumerate(zip(grid,values)):
        if y is not None and abs(y-target)<1e-10:roots.append([p,p])
        if i==len(grid)-1 or y is None or values[i+1] is None:continue
        v=values[i+1];q=grid[i+1]
        if abs(v-y)<1e-12:
            if abs(y-target)<1e-10:roots.append([p,q])
        elif (target-y)*(target-v)<=0:
            t=p+(target-y)/(v-y)*(q-p);roots.append([float(t),float(t)])
    roots.sort();merged=[]
    for interval in roots:
        if merged and interval[0]<=merged[-1][1]+1e-9:merged[-1][1]=max(merged[-1][1],interval[1])
        else:merged.append(interval)
    valid=[v for v in values if v is not None]
    if not valid:status='no_valid_response'
    elif not merged:status='out_of_range' if target<min(valid) or target>max(valid) else 'no_interpolable_match_due_to_missing_levels'
    elif len(merged)>1 or merged[0][1]-merged[0][0]>1e-9:status='multiple_solutions'
    else:status='unique_interpolated_solution'
    return dict(target=float(target),status=status,solution_intervals=merged,unique_p=merged[0][0] if status=='unique_interpolated_solution' else None,response_extent=[min(valid),max(valid)] if valid else None)

def analyze():
    c=read(SIM/'campaign.json');sv=read(SIM/'supervisor.json');assert sv['complete']
    for p,h in c['pins'].items():assert sha(p)==h,p
    assert read(SIM/'CHANNEL_GATE.json')['passed']
    jobs={j['name']:j for j in c['jobs']};completed={};failed={};runs=[];sources={str(SIM/'campaign.json'):sha(SIM/'campaign.json')};refs={};footnames=None
    archive=ROOT/'_archive/contact_erasure_full_2026-09-30';relocation=read(archive/'ARCHIVE_STATUS.json')
    def relocated(path):
        path=str(Path(path))
        if path.startswith(relocation['old_prefix']):return Path(relocation['new_prefix']+path[len(relocation['old_prefix']):])
        if path==relocation['old_runner']:return Path(relocation['archived_runner'])
        if path==relocation['old_channel']:return Path(relocation['archived_channel'])
        return Path(path)
    archived_pins=read(archive/'simulation/campaign.json')['pins']
    for path,h in archived_pins.items():
        actual=relocated(path);assert sha(actual)==h,str(actual);sources[str(actual)]=h
    for j in c['jobs'][:2]:
        d=Path(j['reuse_path'])
        with np.load(d/'erasure_contacts.npz') as z:
            names=z['foot_names'].tolist();assert names==[f'foot_{i//2}_{i%2}' for i in range(38)]
            footnames=names
            for e,cell in enumerate(j['cells']):refs[cell['wave']]=np.asarray(z['foot_normal_force'][:,e])>1e-8
    # Archived rough support references must use exactly the same feet/400 instants.
    rough=read(BASE/'rough_inputs/ROUGH_INPUTS.json')
    flatold=read(ROOT/'libo_spawn_repeats/results/RESULTS.json')['flat_controls']
    for cell in flatold:
        with np.load(Path(cell['run_path'])/'cluster_contacts.npz') as z:
            assert z['foot_names'].tolist()==footnames
            np.testing.assert_array_equal(refs[cell['wave']][4019::20],z['force'][200:,cell['environment_index']]>1e-8)
    for record in sv['finished']:
        j=jobs[record['job']];d=Path(record.get('run_path',SIM/'runs'/record['attempt']/j['name']))
        ids=read(d/'active.json')['source_ids'];selected=[next(x for x in j['cells'] if x['source_id']==i) for i in ids]
        validate_startup(d,len(ids));er=read(d/'ERASURE_RESULT.json');assert er['deleted_contact_violations']==0
        contract=read(d/'input_contract_validation.json');assert contract['full_table_exact'] and contract['flat_cluster_amplitudes_unchanged']
        assert contract['fixed_pd_sha256']==c['fixed_pd_sha256'] and contract['profile_sha256']==c['profile_sha256']
        for path,h in read(d/'implementation_provenance.json')['execution_sources'].items():
            actual=relocated(path)
            if str(actual) not in sources:assert sha(actual)==h,str(actual);sources[str(actual)]=h
            else:assert sources[str(actual)]==h,str(actual)
        amp=read(d/'amplitude_modulation.json');assert amp['other_targets_bitexact'] and amp['limits_passed'] and amp['cells']==selected
        for f in ['active.json','trajectory.npz','trajectory_partial.npz','ERASURE_RESULT.json','ERASURE_FIXED_PLANT.json','BOUNDARY_FAILURE.json','NUMERICAL_FAILURE.json','backend.npz','readback.json','amplitude_modulation.json','erasure_contacts.npz']:
            if (d/f).exists():sources[str(d/f)]=sha(d/f)
        samples=er['restore_samples'];depth=[x['max_native_depth_m']*1000 for x in samples];force=[x['peak_normal_N'] for x in samples]
        rr=dict(job=j['name'],attempt=record['attempt'],path=str(d),n_actual_environments=len(ids),source_ids=ids,diagnostic_only=j['pilot_only'],reused=record.get('reused',False),complete=record['exit']==0,
            restoration_diagnostic=dict(n_samples=len(samples),max_native_depth_mm=max(depth) if depth else None,p95_native_depth_mm=float(np.percentile(depth,95)) if depth else None,peak_normal_force_N=max(force) if force else None),wall_s=read(d/'results.json')['wall_s'] if (d/'results.json').exists() else None)
        runs.append(rr)
        if j['pilot_only']:continue
        if record['exit']!=0:
            if (d/'BOUNDARY_FAILURE.json').exists():
                f=read(d/'BOUNDARY_FAILURE.json');off=f['offender_source_ids'];reason='boundary_guard'
                if max(x['max_body_linear_speed_m_s'] for x in f['environments'] if x['source_id'] in off)>1e6:reason='state_divergence'
                t=f['t']
            else:
                f=read(d/'NUMERICAL_FAILURE.json');off=[ids[i] for i in f['environments']];reason='nonfinite_state';t=f['time_s']
            for cell in selected:
                if cell['source_id'] in off:failed[key(cell)]=dict(cell,status=reason,failure_t_s=t,run_path=str(d),native_final_error_audit_available=(d/'native_error_audit.json').exists())
            continue
        assert er['complete'];ex=extract_run(d,len(ids),90)
        if not record.get('reused',False):
            plant=read(d/'ERASURE_FIXED_PLANT.json');assert plant['passed'] and len(plant['checks'])>=3
        with np.load(d/'erasure_contacts.npz') as z:
            t=z['t'];np.testing.assert_allclose(t,(np.arange(12000)+1)*.0005,atol=1e-12,rtol=0)
            feet=z['foot_normal_force'];assert feet.shape==(12000,len(ids),38)
            for e,cell in enumerate(selected):
                eligible=refs[cell['wave']][4000:];observed=feet[4000:,e]>1e-8
                eligibility10=eligible[19::20];observation10=observed[19::20];assert len(eligibility10)==400
                value=dict(cell,status='complete',run_path=str(d),environment_index=e,forward_2_6_mm=float(ex['forward'][-1,e]-ex['forward'][200,e]),forward_0_6_mm=float(ex['forward'][-1,e]),
                    support_missing_0p5ms=float((eligible&~observed).sum()/eligible.sum()),support_missing_10ms=float((eligibility10&~observation10).sum()/eligibility10.sum()),
                    eligible_flat_support_samples_10ms=int(eligibility10.sum()),missing_eligible_samples_10ms=int((eligibility10&~observation10).sum()),loaded_foot_time_fraction_10ms=float(observation10.mean()),
                    actual_erased_fraction=er['schedules'][e]['actual_erased_time_fraction'],minimum_collision_margin_m=read(d/'collision_guard.json')['minimum_all_step_collision_margin_m'],
                    maximum_body_speed_m_s=float(ex['maximum_body_speed_m_s'][e]),mask_sha256=er['schedules'][e]['mask_sha256'],uniform_sha256=er['schedules'][e]['uniform_sha256'])
                assert key(cell) not in completed;completed[key(cell)]=value
    assert len(completed)+len(failed)==280 and not(set(completed)&set(failed))
    summaries=[]
    for b,p,w in itertools.product(BINS,PS,WAVES):
        cells=[x for x in completed.values() if x['wave']==w and x['p']==p and (p in [0,1] or x['erase_bin_s']==b)]
        fails=[x for x in failed.values() if x['wave']==w and x['p']==p and x['erase_bin_s']==b]
        assert len(cells)+len(fails)==(1 if p in [0,1] else 2)
        summaries.append(dict(bin_s=b,p=p,wave=w,n_complete=len(cells),n_failed=len(fails),seed_ids=[x['mask_seed'] for x in cells],
            forward_2_6_mm=meanstd([x['forward_2_6_mm'] for x in cells]),support_missing_10ms=meanstd([x['support_missing_10ms'] for x in cells]),support_missing_0p5ms=meanstd([x['support_missing_0p5ms'] for x in cells])))
    smap={(x['bin_s'],x['p'],x['wave']):x for x in summaries};paired=[];crossings=[]
    for b,(a,w) in itertools.product(BINS,itertools.combinations(WAVES,2)):
        curve=[]
        for p in PS:
            aa={x['mask_seed']:x for x in completed.values() if x['wave']==a and x['p']==p and (p in [0,1] or x['erase_bin_s']==b)}
            bb={x['mask_seed']:x for x in completed.values() if x['wave']==w and x['p']==p and (p in [0,1] or x['erase_bin_s']==b)}
            seeds=sorted(set(aa)&set(bb));diff=[aa[s]['forward_2_6_mm']-bb[s]['forward_2_6_mm'] for s in seeds]
            row=dict(bin_s=b,wave_a=a,wave_b=w,p=p,n_pairs=len(seeds),paired_seeds=seeds,difference_a_minus_b_mm=meanstd(diff));paired.append(row);curve.append(row)
        for i in range(10):
            lo,hi=curve[i],curve[i+1];x,y=lo['difference_a_minus_b_mm']['mean'],hi['difference_a_minus_b_mm']['mean']
            if x is None or y is None or x*y>=0:continue
            means=[smap[(b,p,ww)]['forward_2_6_mm']['mean'] for p in [PS[i],PS[i+1]] for ww in [a,w]]
            crossings.append(dict(bin_s=b,wave_a=a,wave_b=w,p_bracket=[PS[i],PS[i+1]],paired_difference_endpoints_mm=[x,y],linear_candidate_p=float(PS[i]-x*(PS[i+1]-PS[i])/(y-x)),
                both_waves_positive_at_bracket_endpoints=all(v is not None and v>0 for v in means),n_pairs_at_endpoints=[lo['n_pairs'],hi['n_pairs']],confirmed=False,reason='Exploratory2seed paired sign change; no independent confirmation, missing outcomes can change sample composition.'))
    equivalents=[]
    for level in rough['summaries']:
        w=level['wave']
        for b in BINS:
            displacement=[smap[(b,p,w)]['forward_2_6_mm']['mean'] for p in PS]
            contacts=[smap[(b,p,w)]['support_missing_10ms']['mean'] for p in PS]
            equivalents.append(dict(height_scale=level['height_scale'],wave=w,bin_s=b,n_rough_complete=level['n_complete'],n_rough_failed=level['n_failed'],
                raw_contact_proxy=level['phase_matched_foot_support_missing_fraction'],rough_forward_2_6_mm=level['forward_2_6_mm'],
                contact_equivalent=invert(PS,contacts,level['phase_matched_foot_support_missing_fraction']['mean']),performance_equivalent=invert(PS,displacement,level['forward_2_6_mm']['mean'])))
    for x in rough['cells']:
        x['equivalence_by_bin']={}
        for b in BINS:
            x['equivalence_by_bin'][str(b)]=dict(contact=invert(PS,[smap[(b,p,x['wave'])]['support_missing_10ms']['mean'] for p in PS],x['phase_matched_foot_support_missing_fraction']),
                performance=invert(PS,[smap[(b,p,x['wave'])]['forward_2_6_mm']['mean'] for p in PS],x['forward_2_6_mm']))
    rough['performance_equivalent_status']='Computed descriptive piecewise-linear equivalent maps; ambiguous/unmatched values preserved, no new rough physics'
    rough['performance_equivalent_p']='See per-cell equivalence_by_bin; no universal scalar'
    r=dict(complete=True,planned_unique_conditions=280,complete_conditions=len(completed),failed_conditions=len(failed),reused_endpoint_conditions=10,diagnostic_conditions=1,
        unique_stochastic_conditions=270,stochastic_seeds=[0,1],full4060_status='archived/held by latest user',rough_expansion_status='held; existing-data analysis only',
        supervisor_wall_s=sv['elapsed_s'],worker_attempts=len(runs),condition_execution_count=sum(x['n_actual_environments'] for x in runs if not x['diagnostic_only']),
        reused_condition_execution_count=sum(x['n_actual_environments'] for x in runs if x['reused'] and not x['diagnostic_only']),
        cells=list(completed.values()),failed_cells=list(failed.values()),summaries=summaries,paired_wave_differences=paired,crossing_candidates=crossings,rough_equivalent_groups=equivalents,runs=runs,
        channel_gate=read(SIM/'CHANNEL_GATE.json'),fixed_pd_sha256=c['fixed_pd_sha256'],profile_sha256=c['profile_sha256'],frozen_amplitudes=c['flat_cluster_cells'],source_hashes=sources,
        displacement='All251body mass COM with authored local COM offsets projected on each fixed initial headward XY axis, signed2–6s; zero/reverse/stuck complete outcomes retained.',
        statistics='All complete stochastic observations, sampleSD ddof1 only where n=2; endpoints deterministicn1 shared acrossdurations. No outlier omission or failure-as-zero. No confidence interval/significance claim.',
        contact_equivalence='Same-wave p0 phase-matched support reference >1e-8N, natural swing excluded. Calibrate rough missing-support proxy on new flat response at MATCHED10ms instants;0.5ms flat metric also saved. Return every adjacent-valid-grid linear inverse root/plateau.',
        limitations=['Only2mask seeds; exploratory curves, SD and crossings are uncertain.','Physical/numerical failures are missing6s outcomes and can strongly bias survivor means; do not interpret as stuck.','Re-enabled deeply penetrating shapes may cause contact impulses; restoring collision is not the same as natural rough support loss.','Both equivalentp definitions are descriptive; phase shifts, sideways support, body contacts and slip can make them differ.','Archived rough data have400foot samples2–6s, not continuous0.5ms contact history.','Do not bridge missingp levels when inverting response; multi-valued/out-of-range maps are explicit.','Same mask seeds are paired across wave/p/durations; differentduration draws share a time-indexed uniform bank but are not identical masks.','Peer recovery uses only remainingactual inputs and reindexed partialbatches; repeats do not count as newseeds.','Rough statistics here retain all completed records; older user-requested per-group trimming figures remain preserved.'])
    return r,rough

def plots(r):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import to_rgba
    from PIL import Image
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    files=[]
    for metric,name,title,ylabel in [('forward_2_6_mm','displacement_vs_p.png','Flat contact erasure: displacement response','Signed forward displacement, 2-6 s (mm)'),('support_missing_10ms','support_missing_vs_p.png','Flat contact erasure: expected-support loss','Missing expected loaded-foot fraction (10 ms samples)')]:
        fig,axes=plt.subplots(1,3,figsize=(16,5.5),sharey=True);fig.subplots_adjust(left=.065,right=.985,bottom=.22,top=.80,wspace=.12)
        for ax,b in zip(axes,BINS):
            for w,col in zip(WAVES,COLORS):
                rows=sorted([x for x in r['summaries'] if x['bin_s']==b and x['wave']==w],key=lambda x:x['p']);yy=[x[metric]['mean'] if x[metric]['mean'] is not None else np.nan for x in rows]
                err=[x[metric]['std'] if x[metric]['std'] is not None else 0 for x in rows]
                ax.errorbar(PS,yy,yerr=err,fmt='-o',lw=1.7,ms=4,color=col,ecolor=to_rgba(col,.25),capsize=3,label=f'Wave {w:g}')
                for row in rows:
                    if row['p'] not in [0,1] and row['n_complete']<2:ax.annotate(str(row['n_complete'])+'/2',(row['p'],row[metric]['mean'] if row[metric]['mean'] is not None else 0),xytext=(0,5),textcoords='offset points',fontsize=7,color=col)
            ax.set(xlabel='Fixed erasure probability p',title=f'Hold duration {b*1000:g} ms',xlim=(-.025,1.025));ax.grid(alpha=.17);ax.axhline(0,color='#52616f',lw=.7)
        axes[0].set_ylabel(ylabel);fig.suptitle(title,fontsize=17,y=.975)
        fig.legend(*axes[0].get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.52,.925),ncol=5,frameon=False)
        fig.text(.065,.115,'Interior p: 2 shared mask seeds; pale bars: sample SD. p=0/1: shared deterministic endpoints, same circular marks.',fontsize=9,color='#52616f')
        fig.text(.065,.065,f'Completed {r["complete_conditions"]}/280; failures {r["failed_conditions"]} excluded from displacement (never zero). Annotations show surviving n/2.',fontsize=9,color='#52616f')
        fig.text(.065,.02,'Frozen flat-cluster A/B and173-joint wave4PD; CPU TGS64/16; physics0.5ms / CPG1ms. No confirmation or causal claim.',fontsize=9,color='#52616f')
        path=OUT/name;fig.savefig(path,dpi=150,facecolor='white');plt.close(fig);files.append(path)
    fig,axes=plt.subplots(1,3,figsize=(16,5.1),sharey=True);fig.subplots_adjust(bottom=.22,top=.84,left=.065,right=.98)
    for ax,b in zip(axes,BINS):
        for w,col in zip(WAVES,COLORS):
            rows=sorted([x for x in r['summaries'] if x['bin_s']==b and x['wave']==w],key=lambda x:x['p']);ax.plot(PS,[x['n_failed']/ (1 if x['p'] in [0,1] else 2) for x in rows],'-o',lw=1.6,ms=4,color=col,label=f'Wave {w:g}')
        ax.set(xlabel='Fixed p',title=f'{b*1000:g} ms',ylim=(-.03,1.05));ax.grid(alpha=.18)
    axes[0].set_ylabel('Failure fraction among planned conditions');fig.suptitle('Numerical / boundary failures are separate outcomes',fontsize=16)
    fig.legend(*axes[0].get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.52,.925),ncol=5,frameon=False)
    fig.text(.065,.085,'Only 2 interior seeds. Finite but explosive states are classified as state divergence; they are not zero forward motion.',fontsize=9)
    fig.text(.065,.035,'Inspect this figure together with survivor displacement curves. Full280 registry and raw failure states are preserved.',fontsize=9)
    path=OUT/'failure_fraction_vs_p.png';fig.savefig(path,dpi=150,facecolor='white');plt.close(fig);files.append(path)
    fig,axes=plt.subplots(2,3,figsize=(16,9),sharex=True,sharey=True);fig.subplots_adjust(left=.065,right=.985,bottom=.15,top=.88,hspace=.20)
    for col,b in enumerate(BINS):
        for row,method in enumerate(['contact_equivalent','performance_equivalent']):
            ax=axes[row,col]
            for w,color in zip(WAVES,COLORS):
                rr=sorted([x for x in r['rough_equivalent_groups'] if x['bin_s']==b and x['wave']==w],key=lambda x:x['height_scale']);xx=[x['height_scale'] for x in rr];yy=[x[method]['unique_p'] if x[method]['unique_p'] is not None else np.nan for x in rr];ax.plot(xx,yy,'-o',ms=4,lw=1.3,color=color,label=f'Wave {w:g}')
                for x in rr:
                    eq=x[method]
                    if eq['status']=='multiple_solutions':
                        for lo,hi in eq['solution_intervals']:
                            ax.plot([x['height_scale']]*2,[lo,hi],color=color,alpha=.5,lw=2);ax.scatter([x['height_scale']],[lo],marker='x',color=color,s=28)
            ax.set(title=f'{"Contact" if row==0 else "Performance"} equivalent / {b*1000:g} ms',ylim=(-.025,1.025));ax.grid(alpha=.17)
            if row==1:ax.set_xlabel('Existing Libo height multiplier (fixed4mm cutoff)')
            if col==0:ax.set_ylabel('Equivalent fixed p')
    fig.suptitle('Existing rough terrain: two descriptive erasure equivalents',fontsize=17,y=.98);fig.legend(*axes[0,0].get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.52,.94),ncol=5,frameon=False)
    fig.text(.065,.085,'Circles: unique linear inverse of group mean; crosses / segments: all alternative roots or plateaus. Unmatched points omitted.',fontsize=9)
    fig.text(.065,.048,'No interpolation across missing erasure levels. Rough data: all complete shared spawns, no new physics; no new outlier omission.',fontsize=9)
    fig.text(.065,.013,'Contact proxy excludes normal swing using same-wave flat support phase; phase shifts and slip are not proven Bernoulli deletion.',fontsize=9)
    path=OUT/'rough_equivalent_erasure.png';fig.savefig(path,dpi=150,facecolor='white');plt.close(fig);files.append(path)
    for path in files:
        with Image.open(path) as im:im.load()
    return [dict(path=str(p),sha256=sha(p)) for p in files]

if __name__=='__main__':
    assert not OUT.exists(),'Preserve finished analysis'
    r,rough=analyze();OUT.mkdir();r['figures']=plots(r);r['analysis_source_sha256']=sha(__file__)
    (OUT/'analysis_source.py').write_bytes(Path(__file__).read_bytes());dump(OUT/'RESULTS.json',r);dump(OUT/'ROUGH_EQUIVALENTS.json',rough)
    print(json.dumps(dict(complete=r['complete_conditions'],failed=r['failed_conditions'],crossing_candidates=len(r['crossing_candidates']),wall_minutes=r['supervisor_wall_s']/60)),flush=True)
