"""Merge900 Libo attempts and5 existing flat controls; preserve raw failures."""
from pathlib import Path
import os
for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[name] = '1'
import sys, json, hashlib, copy, argparse, collections
import numpy as np

ROOT = Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
MAIN = ROOT.parent
BASE = ROOT / 'libo_spawn_repeats'
EXT = BASE / 'extension_spawn30_2026-09-30'
SIM = EXT / 'simulation'
OUT = EXT / 'results'
sys.path.insert(0, str(ROOT))
import analyze_libo_repeats as prior
WAVES = [1.5, 2., 2.5, 3., 3.5]
HEIGHTS = [.25, .5, .75, 1., 1.25, 1.5]
COLORS = ['#2474b5', '#df7c13', '#26915a', '#bd4545', '#8862b2']
METRICS = ['instantaneous', 'cycle', 'pooled']
LABELS = ['A. Per 10 ms snapshot', 'B. Per 1 s CPG cycle', 'C. Whole-window ratio']
read = prior.read
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def dump(path, value): Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')
def stats(rows, field):
    values = [float(row[field]) for row in rows]
    return dict(n=len(values), mean=float(np.mean(values)) if values else None,
                std=float(np.std(values, ddof=1)) if len(values)>1 else None,
                values=values, spawn_indices=[row['spawn_index'] for row in rows])


def collect_new():
    campaign, supervisor = read(SIM/'campaign.json'), read(SIM/'supervisor.json')
    assert supervisor['complete'] and supervisor['all_attempts_accounted_for']
    for path, digest in campaign['pins'].items(): assert sha(path)==digest, path
    jobs = {j['name']:j for j in campaign['jobs']}
    cells, failures, runs, series = [], {}, [], {}
    sources = {str(SIM/'campaign.json'):sha(SIM/'campaign.json'), str(SIM/'supervisor.json'):sha(SIM/'supervisor.json')}
    for record in supervisor['finished']:
        job = jobs[record['job']]
        d = SIM/'runs'/record['attempt']/job['name']
        ids = read(d/'active.json')['source_ids']
        source_cells = {c['source_id']:c for c in job['cells']}
        selected = [source_cells[i] for i in ids]
        prior.validate_startup(d,len(ids))
        contract = read(d/'input_contract_validation.json')
        assert contract['full_table_exact'] and contract['flat_cluster_amplitudes_unchanged']
        assert contract['profile_sha256']==campaign['profile_sha256'] and contract['fixed_pd_sha256']==campaign['fixed_pd_sha256']
        amp = read(d/'amplitude_modulation.json')
        assert amp['other_targets_bitexact'] and amp['limits_passed'] and amp['cells']==selected
        execution = read(d/'implementation_provenance.json')['execution_sources']
        assert all(execution.get(path)==digest for path,digest in campaign['pins'].items())
        for path,digest in execution.items():
            if path not in campaign['pins']: assert sha(path)==digest, path
        for name in ['active.json','backend.npz','readback.json','matched_plant_check.json','input_contract_validation.json','implementation_provenance.json','amplitude_modulation.json']:
            sources[str(d/name)]=sha(d/name)
        if record['exit']!=0:
            if record.get('external_interruption'):
                failure=read(d/'EXTERNAL_INTERRUPTION.json')
                assert failure['classification']=='external_interruption_unscored' and failure['source_ids']==ids
                offenders=[];reasons={}
            elif (d/'BOUNDARY_FAILURE.json').exists():
                failure=read(d/'BOUNDARY_FAILURE.json')
                offenders=failure['offender_source_ids']
                reasons={e['source_id']:('state_divergence' if e['max_body_linear_speed_m_s']>1000 or e['max_abs_joint_velocity_rad_s']>1e6 else 'boundary_guard')
                         for e in failure['environments'] if e['source_id'] in offenders}
            else:
                failure=read(d/'NUMERICAL_FAILURE.json')
                offenders=[ids[i] for i in failure['environments']]
                reasons={i:'nonfinite_state' for i in offenders}
            for source_id in offenders:
                cell=source_cells[source_id]
                key=(cell['height_scale'],cell['wave'],cell['spawn_index'])
                assert key not in failures
                failures[key]=dict(cell,status=reasons[source_id],run_path=str(d),failure=failure)
            for name in ['BOUNDARY_FAILURE.json','NUMERICAL_FAILURE.json','EXTERNAL_INTERRUPTION.json','trajectory_partial.npz','failure_state.npz','worker.log']:
                if (d/name).exists(): sources[str(d/name)]=sha(d/name)
            runs.append(dict(job=job['name'],attempt=record['attempt'],path=str(d),source_ids=ids,complete=False,
                             offender_source_ids=offenders,peer_source_ids=[i for i in ids if i not in offenders],native_startup_verified=True,
                             external_interruption=bool(record.get('external_interruption'))))
            continue
        spawn=campaign['spawns'][job['spawn_index']]
        extracted=prior.extract_run(d,len(ids),spawn['start_yaw_deg'])
        guard=read(d/'collision_guard.json')
        assert guard['minimum_all_step_collision_margin_m']>.015
        manifest=read(d/'manifest.json')['rows']
        assert all(r['start_xy']==spawn['start_xy_m'] and r['start_yaw_deg']==spawn['start_yaw_deg'] for r in manifest)
        for name in ['trajectory.npz','cluster_contacts.npz','collision_guard.json','native_error_audit.json','effective_fields_after_pd.json']:
            sources[str(d/name)]=sha(d/name)
        for environment,cell in enumerate(selected):
            fwd=extracted['forward'][:,environment]
            lateral=extracted['lateral'][:,environment]
            cells.append(dict(cell,status='complete',run=job['name'],attempt=record['attempt'],run_path=str(d),environment_index=environment,
                              start_xy_m=spawn['start_xy_m'],start_yaw_deg=spawn['start_yaw_deg'],forward_xy=extracted['forward_xy'][environment].tolist(),
                              forward_0_6_mm=float(fwd[-1]),forward_2_6_mm=float(fwd[-1]-fwd[200]),forward_4_6_mm=float(fwd[-1]-fwd[400]),
                              lateral_0_6_mm=float(lateral[-1]),maximum_joint_speed_rad_s=float(extracted['maximum_joint_speed_rad_s'][environment]),
                              maximum_body_speed_m_s=float(extracted['maximum_body_speed_m_s'][environment])))
            series[f"h{cell['height_scale']:g}_w{cell['wave']:g}_s{cell['spawn_index']:02d}"]=fwd
        runs.append(dict(job=job['name'],attempt=record['attempt'],path=str(d),source_ids=ids,complete=True,
                         native_PD_exact=True,minimum_all_step_collision_margin_m=guard['minimum_all_step_collision_margin_m'],wall_s=extracted['wall_s']))
    keys={(c['height_scale'],c['wave'],c['spawn_index']) for c in cells}
    assert len(keys)==len(cells) and not(keys & set(failures))
    assert keys | set(failures)=={(h,w,s) for h in HEIGHTS for w in WAVES for s in range(10,30)}
    return cells,list(failures.values()),runs,series,sources,campaign,supervisor


def add_contact_metrics(cells,sources):
    cache={}
    def load(cell):
        path=Path(cell['run_path'])/'cluster_contacts.npz'
        if str(path) not in cache:
            with np.load(path) as z:
                assert z['foot_names'].tolist()==[f'foot_{i//2}_{i%2}' for i in range(38)]
                t,force=z['t'].copy(),z['force'].copy()
            np.testing.assert_allclose(t,np.arange(1,601)/100,atol=1e-12,rtol=0)
            assert np.isfinite(force).all()
            cache[str(path)]=(t,force)
            sources[str(path)]=sha(path)
        t,force=cache[str(path)]
        return t,force[:,cell['environment_index']]
    flat={c['wave']:c for c in cells if c['height_scale']==0}
    assert set(flat)==set(WAVES)
    refs={w:load(flat[w]) for w in WAVES}
    for cell in cells:
        t,force=load(cell)
        tf,ff=refs[cell['wave']]
        np.testing.assert_array_equal(t,tf)
        expected=ff>1e-8
        actual=force>1e-8
        missing=expected & ~actual
        gained=~expected & actual
        e,m=expected.sum(1),missing.sum(1)
        assert np.all(m<=e)
        cell['contact_windows']={}
        for label,start in [('0_6',0),('2_6',2),('4_6',4)]:
            ix=t>start+1e-10
            valid=ix & (e>0)
            cycles=[]
            for j in range(start,6):
                select=(t>j+1e-10)&(t<=j+1+1e-10)
                assert int(select.sum())==100 and e[select].sum()>0
                cycles.append(float(m[select].sum()/e[select].sum()))
            metrics={'instantaneous':float(np.mean(m[valid]/e[valid])), 'cycle':float(np.mean(cycles)),
                     'pooled':float(m[ix].sum()/e[ix].sum())}
            assert all(0<=v<=1 for v in metrics.values())
            cell['contact_windows'][label]=dict(loss_percent={k:100*v for k,v in metrics.items()},
                expected_samples=int(e[ix].sum()),missing_samples=int(m[ix].sum()),valid_snapshots=int(valid.sum()),
                skipped_zero_expected_snapshots=int((ix & (e==0)).sum()),
                gained_foot_time_fraction=float(gained[ix].mean()),
                flat_force_weighted_missing=float((ff[ix]*missing[ix]).sum()/ff[ix].sum()),
                cycle_missing_fractions=cycles)
        if cell['height_scale']==0: assert all(v==0 for v in cell['contact_windows']['0_6']['loss_percent'].values())


def summarize(cells,failures):
    summaries,omissions,retained=[],[],[]
    for h in HEIGHTS:
        for w in WAVES:
            rows=sorted([c for c in cells if c['height_scale']==h and c['wave']==w],key=lambda c:c['spawn_index'])
            bad=[c for c in failures if c['height_scale']==h and c['wave']==w]
            assert len(rows)+len(bad)==30
            assert len(rows)>1
            median=float(np.median([c['forward_0_6_mm'] for c in rows]))
            drop=sorted(rows,key=lambda c:(-abs(c['forward_0_6_mm']-median),c['spawn_index']))[0]
            keep=[c for c in rows if c['spawn_index']!=drop['spawn_index']]
            retained.extend(keep)
            omissions.append(dict(height_scale=h,wave=w,spawn_index=drop['spawn_index'],run_path=drop['run_path'],
                                  forward_0_6_mm=drop['forward_0_6_mm'],group_median_0_6_mm=median,
                                  absolute_distance_from_median_mm=abs(drop['forward_0_6_mm']-median)))
            loss={}
            for window in ['0_6','2_6','4_6']:
                loss[window]={k:stats([dict(spawn_index=c['spawn_index'],value=c['contact_windows'][window]['loss_percent'][k]) for c in keep],'value') for k in METRICS}
            summaries.append(dict(height_scale=h,wave=w,fixed_pair_angle_rms_deg=rows[0]['fixed_pair_angle_rms_deg'],
                                  n_planned=30,n_complete_before=len(rows),n_failed=len(bad),n_excluded=1,n_retained=len(keep),
                                  excluded_spawn_index=drop['spawn_index'],forward_0_6_mm=stats(keep,'forward_0_6_mm'),
                                  forward_2_6_mm=stats(keep,'forward_2_6_mm'),forward_4_6_mm=stats(keep,'forward_4_6_mm'),
                                  raw_forward_0_6_mm=stats(rows,'forward_0_6_mm'),contact_loss_percent=loss))
    paired=[]
    for h in HEIGHTS:
        for label,data in [('raw',cells),('trimmed',retained)]:
            a={c['spawn_index']:c for c in data if c['height_scale']==h and c['wave']==2.5}
            b={c['spawn_index']:c for c in data if c['height_scale']==h and c['wave']==3}
            common=sorted(set(a)&set(b))
            points=[dict(spawn_index=s,value=b[s]['forward_0_6_mm']-a[s]['forward_0_6_mm']) for s in common]
            paired.append(dict(height_scale=h,selection=label,wave_a=2.5,wave_b=3,
                               difference_wave3_minus_wave2p5_mm=stats(points,'value'),wave3_positive_difference_count=sum(p['value']>0 for p in points)))
    return summaries,omissions,retained,paired


def plots(report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MultipleLocator,PercentFormatter
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11.5,'svg.fonttype':'none'})
    groups=report['summaries']
    flat={c['wave']:c for c in report['flat_controls']}
    outputs=[]
    def save(fig,name):
        for suffix in ['.png','.svg']:
            path=OUT/(name+suffix)
            fig.savefig(path,dpi=170,facecolor='white')
            outputs.append(dict(path=str(path),sha256=sha(path)))
        plt.close(fig)
    def axes_style(ax):
        ax.axhline(0,color='#818891',lw=.8,alpha=.45,zorder=0)
        ax.grid(alpha=.16)
        for side in ['top','right']: ax.spines[side].set_visible(False)
    footer=f"30 planned shared spawns per rough group; {report['retained_total_count']} retained complete trials. Pale bars: sample SD."
    for window,title in [('0_6','Forward displacement, 0–6 s'),('4_6','Forward displacement, 4–6 s')]:
        fig,ax=plt.subplots(figsize=(11.5,6.8));fig.subplots_adjust(left=.11,right=.97,bottom=.22,top=.8)
        for wave,color in zip(WAVES,COLORS):
            rows=sorted([g for g in groups if g['wave']==wave],key=lambda g:g['height_scale'])
            field='forward_'+window+'_mm'
            xx=[0]+[g['fixed_pair_angle_rms_deg'] for g in rows]
            yy=[flat[wave][field]]+[g[field]['mean'] for g in rows]
            sd=[0]+[g[field]['std'] or 0 for g in rows]
            ax.errorbar(xx,yy,yerr=sd,fmt='none',ecolor=color,alpha=.25,capsize=3,zorder=1)
            ax.plot(xx,yy,'-o',color=color,lw=2,ms=5.5,label=f'Wave {wave:g}',zorder=3)
        axes_style(ax)
        ax.set(xlabel='Shared fixed 38-foot / 703-pair angle RMS (deg)',ylabel=title+' (mm)')
        fig.suptitle('Libo 4 mm: displacement vs. terrain roughness',y=.96,fontsize=17)
        fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.54,.91),ncol=5,frameon=False)
        fig.text(.11,.115,footer,fontsize=10)
        fig.text(.11,.055,'One farthest-from-median complete trial omitted per combined rough group; unchanged amplitudes, PD and physics.',fontsize=9.5)
        save(fig,'mean_std_vs_roughness' if window=='0_6' else 'mean_std_4_6_vs_roughness')
    fig,ax=plt.subplots(figsize=(11.5,6.8));fig.subplots_adjust(left=.11,right=.97,bottom=.22,top=.77)
    palette=['#575f68','#235789','#3f8d8d','#94b447','#d79138','#bd4545','#8862b2']
    for height,color in zip([0,*HEIGHTS],palette):
        if height==0:
            yy=[flat[w]['forward_0_6_mm'] for w in WAVES];sd=[0]*5
        else:
            rows=sorted([g for g in groups if g['height_scale']==height],key=lambda g:g['wave'])
            yy=[g['forward_0_6_mm']['mean'] for g in rows];sd=[g['forward_0_6_mm']['std'] or 0 for g in rows]
        ax.errorbar(WAVES,yy,yerr=sd,fmt='none',ecolor=color,alpha=.25,capsize=3,zorder=1)
        ax.plot(WAVES,yy,'-o',color=color,lw=2,ms=5.5,label=f'Height x{height:g}',zorder=3)
    axes_style(ax);ax.set(xlabel='Wave number',ylabel='Forward displacement, 0–6 s (mm)');ax.set_xticks(WAVES)
    fig.suptitle('Libo 4 mm: displacement vs. wave number',y=.965,fontsize=17)
    fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.54,.915),ncol=4,frameon=False)
    fig.text(.11,.115,footer,fontsize=10);fig.text(.11,.055,'Same retained trial sets as the terrain-roughness and contact-loss plots.',fontsize=10)
    save(fig,'mean_std_vs_wave_number')
    top=max(250,10*np.ceil(max([flat[w]['forward_0_6_mm'] for w in WAVES]+[g['forward_0_6_mm']['mean']+(g['forward_0_6_mm']['std'] or 0) for g in groups])/10)+10)
    bottom=min(-20,10*np.floor(min(g['forward_0_6_mm']['mean']-(g['forward_0_6_mm']['std'] or 0) for g in groups)/10)-10)
    def draw_loss(ax,metric,comparison=False):
        for wave,color in zip(WAVES,COLORS):
            rows=sorted([g for g in groups if g['wave']==wave],key=lambda g:g['height_scale'])
            xx=[0]+[g['contact_loss_percent']['0_6'][metric]['mean'] for g in rows]
            yy=[flat[wave]['forward_0_6_mm']]+[g['forward_0_6_mm']['mean'] for g in rows]
            xsd=[0]+[g['contact_loss_percent']['0_6'][metric]['std'] or 0 for g in rows]
            ysd=[0]+[g['forward_0_6_mm']['std'] or 0 for g in rows]
            ax.errorbar(xx,yy,xerr=xsd,yerr=ysd,fmt='none',ecolor=color,alpha=.23,capsize=2.5,zorder=1)
            ax.plot(xx,yy,'-o',color=color,lw=2,ms=4.6 if comparison else 5.5,label=f'Wave {wave:g}',zorder=3)
        axes_style(ax)
        ax.set(xlim=(-1.5,100),ylim=(bottom,top),xlabel='Expected-support loss (%)')
        ax.xaxis.set_major_locator(MultipleLocator(20 if comparison else 10))
        ax.xaxis.set_major_formatter(PercentFormatter(xmax=100,decimals=0))
    for metric,label in zip(METRICS,LABELS):
        fig,ax=plt.subplots(figsize=(10.7,6.9));fig.subplots_adjust(left=.11,right=.955,bottom=.23,top=.8)
        draw_loss(ax,metric);ax.set_ylabel('Forward COM displacement, 0–6 s (mm)')
        fig.suptitle(label+': displacement vs. contact-loss portion',y=.96,fontsize=16)
        fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.54,.91),ncol=5,frameon=False)
        fig.text(.11,.125,footer,fontsize=10)
        fig.text(.11,.06,'Same 0–6 s window and retained trials in both axes; phase-matched reference from the corresponding flat gait.',fontsize=9.5)
        save(fig,'displacement_vs_'+metric+'_loss_portion')
    fig,axs=plt.subplots(1,3,figsize=(18.5,6.7),sharey=True);fig.subplots_adjust(left=.055,right=.975,bottom=.22,top=.72,wspace=.12)
    weights=['Equal weight per valid snapshot','Equal weight per 1 s cycle','Weight by expected contact samples']
    for ax,metric,label,weight in zip(axs,METRICS,LABELS,weights):
        draw_loss(ax,metric,True);ax.set_title(label+'\n'+weight,fontsize=11.5,pad=13)
    axs[0].set_ylabel('Forward COM displacement (mm)')
    fig.suptitle('Libo 4 mm, 30 shared spawns: three contact-loss metrics',y=.965,fontsize=17)
    fig.legend(*axs[0].get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.91),ncol=5,frameon=False)
    fig.text(.055,.115,footer+' Same 0–6 s window.',fontsize=10.5)
    fig.text(.055,.055,'A: average instantaneous fractions. B: pool each nominal 1 s cycle, then average. C: pool the whole window.',fontsize=10.5)
    save(fig,'loss_portion_metrics_comparison')
    return outputs


def analyze():
    new,failed_new,runs,series,sources,campaign,supervisor=collect_new()
    dispatch=read(SIM/'DISPATCH.json');gap_s=0.
    history=SIM/'dispatch_resume_history'
    snapshots=sorted(history.glob('supervisor_before_*.json')) if history.exists() else []
    for index,path in enumerate(snapshots,1):
        before=read(path);next_dispatch=history/f'dispatch_before_{index+1:02d}.json'
        after=read(next_dispatch) if next_dispatch.exists() else dispatch
        gap_s+=max(0.,after['resumed_unix']-before['started_unix']-before['elapsed_s'])
        sources[str(path)]=sha(path)
    sources[str(SIM/'DISPATCH.json')]=sha(SIM/'DISPATCH.json')
    old_cells,old_failed=[],[]
    for result_path,series_path in [(BASE/'results/RESULTS.json',BASE/'results/forward_timeseries.npz'),
                                  (BASE/'extension_1p25_1p5/results/RESULTS.json',BASE/'extension_1p25_1p5/results/forward_timeseries.npz')]:
        old=read(result_path);assert old['complete']
        sources[str(result_path)]=sha(result_path);sources[str(series_path)]=sha(series_path)
        with np.load(series_path) as z:
            for cell in old['cells']:
                cell=copy.deepcopy(cell)
                key=f"h{cell['height_scale']:g}_w{cell['wave']:g}_s{cell['spawn_index']:02d}"
                assert key in z.files and key not in series
                fwd=z[key].copy();assert fwd.shape==(601,) and np.isfinite(fwd).all()
                np.testing.assert_allclose(fwd[-1],cell['forward_0_6_mm'],atol=1e-10,rtol=0)
                cell['forward_2_6_mm']=float(fwd[-1]-fwd[200]);series[key]=fwd;old_cells.append(cell)
        old_failed.extend(old['failed_cells'])
    assert len(old_cells)==297 and len(old_failed)==8
    cells=old_cells+new;failures=old_failed+failed_new
    keys={(c['height_scale'],c['wave'],c['spawn_index']) for c in cells+failures}
    assert len(keys)==len(cells)+len(failures)==905
    add_contact_metrics(cells,sources)
    summaries,omissions,retained,paired=summarize(cells,failures)
    flat=sorted([c for c in cells if c['height_scale']==0],key=lambda c:c['wave'])
    report=dict(complete=True,planned_total_condition_count=905,rough_condition_count=900,preserved_flat_control_count=5,
                new_planned_condition_count=600,new_complete_count=len(new),new_failed_count=len(failed_new),
                raw_complete_count=len(cells),failed_condition_count=len(failures),excluded_for_statistics_count=len(omissions),
                retained_rough_count=len(retained),retained_total_count=len(retained)+5,heights=HEIGHTS,waves=WAVES,
                flat_controls=flat,cells=cells,failed_cells=failures,summaries=summaries,excluded_observations=omissions,
                paired_wave2p5_3=paired,new_runs=runs,source_hashes=sources,
                physical_campaign_wall_s=supervisor['elapsed_s']-gap_s,new_worker_attempts=len(runs),
                total_elapsed_wall_s=supervisor['elapsed_s'],unobserved_dispatch_gap_s=gap_s,
                timing_note='Active dispatch elapsed excludes observed gaps between last pre-interruption supervisor receipt and resumed dispatcher. This is not summed physics-step CPU time; interruption instant is not exactly known.',
                new_condition_executions=sum(len(run['source_ids']) for run in runs),
                additional_condition_executions=sum(len(run['source_ids']) for run in runs)-600,
                peer_reexecutions=sum(len(run['source_ids']) for run in runs if run['attempt'].startswith('peer')),
                external_interruption_reexecutions=sum(len(run['source_ids']) for run in runs if run['attempt'].startswith('interruption_resume')),
                externally_interrupted_unscored_attempts=sum(bool(run.get('external_interruption')) for run in runs),
                fixed_pd_sha256=campaign['fixed_pd_sha256'],profile_sha256=campaign['profile_sha256'],
                native_worker_bitexact_to_original=True,
                statistics='Per combined rough height×wave group,omit exactly one complete0–6s point farthest from median; tie lowestspawn index. Same retained set for all windows and all contact metrics. Sample SDddof1. No failure-as-zero; original artifacts unchanged.',
                contact_definitions={'instantaneous':'Equal mean of missingfeet/expectedflatfeet per valid10ms snapshot; expected=0 omitted.',
                                     'cycle':'Poolmissing/expected within each nominal1s interval,then average intervalfractions.',
                                     'pooled':'Totalmissing expected foot-time samples / total expected flat foot-time samples.'},
                contact_threshold_N=1e-8,contact_snapshot_dt_s=.01,
                limitations=['Same single Libo terrain realization and30 shared position/heading observations,not30independent terrains.',
                             'One flat reference perwave; no repeat-basedflatSD.',
                             'Contact loss uses38toes and fixedCPGclock; phasechanges count as differences,and forces do not prove anchoring.',
                             'Loss-axis intersections need not correspond to equalterrainconditions; no calibrated IID erasureprobability.',
                             'Numerical/guardfailures are missing6s scores; reported means are conditional on complete cases.',
                             'One-point omission is descriptive,can remove a realstuck/reverse outcome,and changes paired intersections.'],
                figure_review=dict(passed=False,status='awaiting_visual_inspection'),analysis_source_sha256=sha(__file__))
    OUT.mkdir(exist_ok=False)
    (OUT/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
    np.savez_compressed(OUT/'forward_timeseries.npz',t_s=np.arange(601)/100,**series)
    report['forward_timeseries_sha256']=sha(OUT/'forward_timeseries.npz')
    dump(OUT/'RESULTS.json',report)
    report['figures']=plots(report)
    dump(OUT/'RESULTS.json',report)
    print(json.dumps(dict(new_complete=len(new),new_failed=len(failed_new),total_complete=len(cells),total_failed=len(failures),
                         retained=report['retained_total_count'],physical_wall_min=report['physical_campaign_wall_s']/60,outputs=str(OUT)),ensure_ascii=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['analyze','plots'],default='analyze',nargs='?');args=parser.parse_args()
    if args.mode=='analyze': analyze()
    else:
        report=read(OUT/'RESULTS.json');report['figures']=plots(report);dump(OUT/'RESULTS.json',report)
