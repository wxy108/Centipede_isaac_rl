"""Analyze frozen native Libo cutoff/height stages; never launch physics.

Stage: --stage STAGE_OR_SIMULATION --out FRESH_RESULTS_DIRECTORY
Merge: --combine --stages RESULT_OR_STAGE ... --out FRESH_RESULTS_DIRECTORY
The five existing flat controls and all old 4 mm raw observations are reused.
"""
from pathlib import Path
import os
for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[name] = '1'
import sys, json, hashlib, copy, argparse, collections
import ast
import numpy as np

ROOT = Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
MAIN = ROOT.parent
PREVIOUS = ROOT / 'libo_spawn_repeats/extension_spawn30_2026-09-30/results'
SPAWN_CAMPAIGN = ROOT / 'libo_spawn_repeats/extension_spawn30_2026-09-30/simulation/campaign.json'
EXPANSION_BASE = ROOT / 'libo_cutoff_height_expansion_2026-10-01'
sys.path.insert(0, str(ROOT))
import analyze_libo_repeats as native

WAVES = [1.5, 2., 2.5, 3., 3.5]
COLORS = ['#2474b5', '#df7c13', '#26915a', '#bd4545', '#8862b2']
METRICS = ['instantaneous', 'cycle', 'pooled']
LABELS = ['A: per 10 ms snapshot', 'B: per 1 s CPG cycle', 'C: whole-window ratio']
STOP_RULE = dict(wave=3.5, window_s=[4., 6.], forward_threshold_mm=2.,
                 minimum_complete=24, minimum_stuck_fraction=.8, planned_spawn_count=30,
                 numerical_or_boundary_failures_count_as_stuck=False,
                 omission_applied=False,
                 authorization='User confirmed 2026-10-01: 采用这个判据（建议）')
read = native.read


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')


def cell_key(cell):
    cutoff=cell.get('cutoff_mm', 4)
    return (float(cutoff) if cutoff is not None else 0., float(cell['height_scale']), float(cell['wave']), int(cell['spawn_index']))


def series_key(cell):
    cutoff, height, wave, spawn = cell_key(cell)
    if height == 0:
        return f'flat_w{wave:g}'
    return f'c{cutoff:g}_h{height:g}_w{wave:g}_s{spawn:02d}'


def stats(rows, field):
    values = [float(row[field]) for row in rows]
    assert np.isfinite(values).all()
    return dict(n=len(values), mean=float(np.mean(values)) if values else None,
                std=float(np.std(values, ddof=1)) if len(values)>1 else None,
                min=min(values) if values else None, max=max(values) if values else None,
                values=values, spawn_indices=[row['spawn_index'] for row in rows])


def source(path, sources):
    sources[str(path)] = sha(path)


def stage_sim(path):
    path = Path(path)
    if not path.is_absolute():
        path = EXPANSION_BASE / path
    return path if (path/'campaign.json').is_file() else path/'simulation'


def collect_stage(path):
    sim = stage_sim(path)
    campaign, supervisor = read(sim/'campaign.json'), read(sim/'supervisor.json')
    assert supervisor['complete'] and supervisor['all_attempts_accounted_for']
    for path, digest in campaign['pins'].items():
        assert sha(path)==digest, path
    assert campaign['spawns']==read(SPAWN_CAMPAIGN)['spawns'], 'Shared 30-spawn definition changed'
    assert campaign['workers']==4 and campaign['threads_per_worker']==2
    assert campaign['environments_per_worker']==[3,2,3,2] and campaign['new_flat_condition_count']==0
    adapter=sim/'executed_source/adapted_functions.py'
    original_adapter=ROOT/'libo_spawn_repeats/simulation/executed_source/adapted_functions.py'
    assert sha(adapter)==sha(original_adapter), 'Native physics/control adapter changed'
    original_source=(ROOT/'run_libo_repeats.py').read_text(encoding='utf-8')
    original_worker=next(ast.get_source_segment(original_source,n) for n in ast.parse(original_source).body
                         if isinstance(n,ast.FunctionDef) and n.name=='worker')
    assert (sim/'executed_source/original_worker_function.py').read_text(encoding='utf-8')==original_worker+'\n'
    jobs = {j['name']:j for j in campaign['jobs']}
    expected = {cell_key(c):c for j in jobs.values() for c in j['cells']}
    assert len(expected)==sum(len(j['cells']) for j in jobs.values())
    assert all(c['height_scale']>0 and c['wave'] in WAVES for c in expected.values())
    planned_groups = collections.defaultdict(set)
    for cutoff, height, wave, spawn in expected:
        planned_groups[(cutoff,height,wave)].add(spawn)
    assert all(spawns==set(range(30)) for spawns in planned_groups.values())
    cells, failures, runs, series, sources = [], {}, [], {}, {}
    for p in [sim/'campaign.json', sim/'supervisor.json', SPAWN_CAMPAIGN]:
        source(p,sources)
    for record in supervisor['finished']:
        job = jobs[record['job']]
        d = sim/'runs'/record['attempt']/job['name']
        source_cells = {c['source_id']:c for c in job['cells']}
        if record.get('external_interruption') or (d/'EXTERNAL_INTERRUPTION.json').is_file():
            # An interrupted process can stop before App/startup binding finishes.
            # Existing receipts are evidence, not a scored run or verified startup.
            failure=read(d/'EXTERNAL_INTERRUPTION.json')
            assert failure['classification']=='external_interruption_unscored' and record['exit']!=0
            ids=failure['source_ids']
            assert set(ids)<=set(source_cells) and len(ids)==len(set(ids))
            if (d/'active.json').is_file():
                assert read(d/'active.json')['source_ids']==ids
            for name in ['EXTERNAL_INTERRUPTION.json','active.json','backend.npz','readback.json',
                         'matched_plant_check.json','thread_settings.json','iteration_readback.json',
                         'native_contact_offsets.json','input_contract_validation.json',
                         'implementation_provenance.json','amplitude_modulation.json',
                         'trajectory_partial.npz','failure_state.npz','worker.log']:
                if (d/name).is_file(): source(d/name,sources)
            runs.append(dict(job=job['name'],attempt=record['attempt'],path=str(d),source_ids=ids,
                             complete=False,offender_source_ids=[],peer_source_ids=ids,
                             native_startup_verified=False,external_interruption=True,
                             status='external_interruption_unscored',
                             receipt_note='Interrupted before full validation; existing receipts hashed only. No numerical-failure, PD-readback or six-second-score claim.'))
            continue
        ids = read(d/'active.json')['source_ids']
        selected = [source_cells[i] for i in ids]
        assert len(ids)==len(set(ids)) and set(ids)<=set(source_cells)
        native.validate_startup(d,len(ids))
        contract = read(d/'input_contract_validation.json')
        assert contract['full_table_exact'] and contract['flat_cluster_amplitudes_unchanged']
        assert contract['profile_sha256']==campaign['profile_sha256']
        assert contract['fixed_pd_sha256']==campaign['fixed_pd_sha256']
        amp = read(d/'amplitude_modulation.json')
        assert amp['other_targets_bitexact'] and amp['limits_passed'] and amp['cells']==selected
        execution = read(d/'implementation_provenance.json')['execution_sources']
        assert all(execution.get(p)==h for p,h in campaign['pins'].items())
        for p,h in execution.items():
            if p not in campaign['pins']:
                assert sha(p)==h, p
        for name in ['active.json','backend.npz','readback.json','matched_plant_check.json',
                     'thread_settings.json','iteration_readback.json','native_contact_offsets.json',
                     'input_contract_validation.json','implementation_provenance.json','amplitude_modulation.json']:
            source(d/name,sources)
        if record['exit']!=0:
            if (d/'BOUNDARY_FAILURE.json').is_file():
                failure=read(d/'BOUNDARY_FAILURE.json')
                offenders=failure['offender_source_ids']
                reasons={e['source_id']:('state_divergence' if e['max_body_linear_speed_m_s']>1000
                         or e['max_abs_joint_velocity_rad_s']>1e6 else 'boundary_guard')
                         for e in failure['environments'] if e['source_id'] in offenders}
                assert set(reasons)==set(offenders)
            else:
                assert (d/'NUMERICAL_FAILURE.json').is_file(), f'Unclassified infrastructure failure: {d}'
                failure=read(d/'NUMERICAL_FAILURE.json')
                offenders=[ids[i] for i in failure['environments']]
                reasons={i:'nonfinite_state' for i in offenders}
            assert offenders, f'Quality failure has no offending conditions: {d}'
            for source_id in offenders:
                cell=source_cells[source_id]; key=cell_key(cell)
                assert key not in failures, f'Offending condition was rerun: {key}'
                failures[key]=dict(cell,status=reasons[source_id],run_path=str(d),failure=failure)
            for name in ['BOUNDARY_FAILURE.json','NUMERICAL_FAILURE.json','EXTERNAL_INTERRUPTION.json',
                         'trajectory_partial.npz','failure_state.npz','worker.log']:
                if (d/name).is_file(): source(d/name,sources)
            runs.append(dict(job=job['name'],attempt=record['attempt'],path=str(d),source_ids=ids,
                             complete=False,offender_source_ids=offenders,
                             peer_source_ids=[i for i in ids if i not in offenders],native_startup_verified=True,
                             external_interruption=False))
            continue
        spawn=campaign['spawns'][job['spawn_index']]
        assert all(c['spawn_index']==job['spawn_index'] for c in selected)
        extracted=native.extract_run(d,len(ids),spawn['start_yaw_deg'])
        guard=read(d/'collision_guard.json')
        assert guard['minimum_all_step_collision_margin_m']>.015
        manifest=read(d/'manifest.json')['rows']
        assert len(manifest)==len(ids)
        assert all(r['start_xy']==spawn['start_xy_m'] and r['start_yaw_deg']==spawn['start_yaw_deg'] for r in manifest)
        for name in ['trajectory.npz','cluster_contacts.npz','collision_guard.json','native_error_audit.json',
                     'effective_fields_after_pd.json','results.json','manifest.json']:
            source(d/name,sources)
        for environment,cell in enumerate(selected):
            fwd=extracted['forward'][:,environment]
            lateral=extracted['lateral'][:,environment]
            result=dict(cell,status='complete',run=job['name'],attempt=record['attempt'],run_path=str(d),
                        environment_index=environment,start_xy_m=spawn['start_xy_m'],
                        start_yaw_deg=spawn['start_yaw_deg'],forward_xy=extracted['forward_xy'][environment].tolist(),
                        forward_0_6_mm=float(fwd[-1]),forward_2_6_mm=float(fwd[-1]-fwd[200]),
                        forward_4_6_mm=float(fwd[-1]-fwd[400]),lateral_0_6_mm=float(lateral[-1]),
                        maximum_joint_speed_rad_s=float(extracted['maximum_joint_speed_rad_s'][environment]),
                        maximum_body_speed_m_s=float(extracted['maximum_body_speed_m_s'][environment]))
            cells.append(result)
            assert series_key(cell) not in series
            series[series_key(cell)]=fwd
        runs.append(dict(job=job['name'],attempt=record['attempt'],path=str(d),source_ids=ids,complete=True,
                         native_PD_exact=True,minimum_all_step_collision_margin_m=guard['minimum_all_step_collision_margin_m'],
                         wall_s=extracted['wall_s']))
    keys={cell_key(c) for c in cells}
    assert len(keys)==len(cells) and not(keys & set(failures))
    assert keys | set(failures)==set(expected), 'Not every planned independent condition is accounted for'
    return cells,list(failures.values()),runs,series,sources,campaign,supervisor,list(expected.values()),sim


def preserved_previous():
    path=PREVIOUS/'RESULTS.json'; series_path=PREVIOUS/'forward_timeseries.npz'
    report=read(path)
    assert report['complete'] and report['native_worker_bitexact_to_original']
    assert sha(series_path)==report['forward_timeseries_sha256']
    cells=copy.deepcopy(report['cells']); failures=copy.deepcopy(report['failed_cells'])
    flat=sorted([c for c in cells if c['height_scale']==0],key=lambda c:c['wave'])
    assert len(flat)==5 and [c['wave'] for c in flat]==WAVES
    assert flat==report['flat_controls']
    sources={str(path):sha(path),str(series_path):sha(series_path)}
    series={}
    with np.load(series_path) as z:
        for cell in cells:
            key=f"h{cell['height_scale']:g}_w{cell['wave']:g}_s{cell['spawn_index']:02d}"
            fwd=z[key].copy()
            assert fwd.shape==(601,) and np.isfinite(fwd).all()
            np.testing.assert_allclose(fwd[-1],cell['forward_0_6_mm'],atol=1e-10,rtol=0)
            cell['forward_2_6_mm']=float(fwd[-1]-fwd[200])
            cell['cutoff_mm']=None if cell['height_scale']==0 else 4.
            series[series_key(cell)]=fwd
    for cell in failures: cell['cutoff_mm']=4.
    # Reused controls are exact prior flat observations, not new repeated samples.
    flat=sorted([c for c in cells if c['height_scale']==0],key=lambda c:c['wave'])
    cache={}
    for cell in flat:
        d=Path(cell['run_path']); n=len(read(d/'active.json')['source_ids'])
        cache.setdefault(str(d),None)
        if cache[str(d)] is None:
            cache[str(d)]=native.extract_run(d,n,cell['start_yaw_deg'])
        ex=cache[str(d)]['forward'][:,cell['environment_index']]
        np.testing.assert_allclose(ex,series[series_key(cell)],atol=1e-10,rtol=0)
        for name in ['trajectory.npz','backend.npz','readback.json','results.json','cluster_contacts.npz']:
            source(d/name,sources)
    return [c for c in cells if c['height_scale']>0], failures, flat, series, sources, report


def add_contact_metrics(cells, flat, sources):
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
            source(path,sources)
        t,force=cache[str(path)]
        return t,force[:,cell['environment_index']]
    refs={c['wave']:load(c) for c in flat}
    assert set(refs)==set(WAVES)
    for cell in [*flat,*cells]:
        t,force=load(cell); tf,ff=refs[cell['wave']]
        np.testing.assert_array_equal(t,tf)
        expected=ff>1e-8; actual=force>1e-8
        missing=expected & ~actual; gained=~expected & actual
        e,m=expected.sum(1),missing.sum(1)
        assert np.all(m<=e)
        cell['contact_windows']={}
        for label,start in [('0_6',0),('2_6',2),('4_6',4)]:
            ix=t>start+1e-10; valid=ix & (e>0); cycles=[]
            assert valid.any() and e[ix].sum()>0 and ff[ix].sum()>0
            for j in range(start,6):
                select=(t>j+1e-10)&(t<=j+1+1e-10)
                assert int(select.sum())==100 and e[select].sum()>0
                cycles.append(float(m[select].sum()/e[select].sum()))
            metrics=dict(instantaneous=float(np.mean(m[valid]/e[valid])),cycle=float(np.mean(cycles)),
                         pooled=float(m[ix].sum()/e[ix].sum()))
            assert all(0<=v<=1 for v in metrics.values())
            cell['contact_windows'][label]=dict(loss_percent={k:100*v for k,v in metrics.items()},
                expected_samples=int(e[ix].sum()),missing_samples=int(m[ix].sum()),valid_snapshots=int(valid.sum()),
                skipped_zero_expected_snapshots=int((ix & (e==0)).sum()),gained_foot_time_fraction=float(gained[ix].mean()),
                flat_force_weighted_missing=float((ff[ix]*missing[ix]).sum()/ff[ix].sum()),cycle_missing_fractions=cycles)
        if cell['height_scale']==0:
            assert all(v==0 for v in cell['contact_windows']['0_6']['loss_percent'].values())


def stop_judgments(cells, failures, planned):
    groups=sorted({cell_key(c)[:2] for c in planned})
    result=[]
    for cutoff,height in groups:
        rows=sorted([c for c in cells if cell_key(c)[:3]==(cutoff,height,3.5)],key=lambda c:c['spawn_index'])
        bad=[c for c in failures if cell_key(c)[:3]==(cutoff,height,3.5)]
        count=sum(c['forward_4_6_mm']<=STOP_RULE['forward_threshold_mm'] for c in rows)
        fraction=count/len(rows) if rows else None
        nplanned=sum(cell_key(c)[:3]==(cutoff,height,3.5) for c in planned)
        assert nplanned==30 and len(rows)+len(bad)==nplanned
        enough=len(rows)>=STOP_RULE['minimum_complete']
        result.append(dict(cutoff_mm=cutoff,height_scale=height,wave=3.5,n_planned=nplanned,
            n_complete=len(rows),n_incomplete=len(bad),n_raw_complete_at_or_below_threshold=count,
            raw_stuck_fraction=fraction,minimum_complete_met=enough,
            status='insufficient_complete_trials' if not enough else ('criterion_met' if fraction>=.8 else 'criterion_not_met'),
            stop_height_continuation=(not enough or fraction>=.8),
            stop_reason='insufficient_quality_for_stuck_judgment' if not enough else ('wave3p5_stuck_criterion' if fraction>=.8 else None),
            raw_forward_4_6_mm=stats(rows,'forward_4_6_mm'),rule=STOP_RULE))
    return result


def controller_judgements(judgments):
    return [dict(cutoff_mm=r['cutoff_mm'],height_scale=r['height_scale'],wave=3.5,n_planned=r['n_planned'],
                 n_complete=r['n_complete'],n_stalled_raw=r['n_raw_complete_at_or_below_threshold'],
                 stalled_fraction=r['raw_stuck_fraction'],eligible=r['minimum_complete_met'],
                 stop=r['status']=='criterion_met',
                 status=('insufficient_complete' if not r['minimum_complete_met']
                         else 'stalled' if r['status']=='criterion_met' else 'progressing')) for r in judgments]


def summarize(cells,failures,planned):
    summaries,omissions,retained=[],[],[]
    groups=sorted({cell_key(c)[:3] for c in planned})
    for cutoff,height,wave in groups:
        rows=sorted([c for c in cells if cell_key(c)[:3]==(cutoff,height,wave)],key=lambda c:c['spawn_index'])
        bad=[c for c in failures if cell_key(c)[:3]==(cutoff,height,wave)]
        inputs=[c for c in planned if cell_key(c)[:3]==(cutoff,height,wave)]
        assert len(inputs)==30 and len(rows)+len(bad)==30
        rms={float(c['fixed_pair_angle_rms_deg']) for c in inputs}
        assert len(rms)==1
        drop=None; median=None
        if rows:
            median=float(np.median([c['forward_0_6_mm'] for c in rows]))
            drop=sorted(rows,key=lambda c:(-abs(c['forward_0_6_mm']-median),c['spawn_index']))[0]
            omissions.append(dict(cutoff_mm=cutoff,height_scale=height,wave=wave,spawn_index=drop['spawn_index'],
                run_path=drop['run_path'],forward_0_6_mm=drop['forward_0_6_mm'],group_median_0_6_mm=median,
                absolute_distance_from_median_mm=abs(drop['forward_0_6_mm']-median)))
        keep=[c for c in rows if drop is None or c['spawn_index']!=drop['spawn_index']]
        retained.extend(keep)
        loss={}
        for window in ['0_6','2_6','4_6']:
            loss[window]={k:stats([dict(spawn_index=c['spawn_index'],value=c['contact_windows'][window]['loss_percent'][k])
                                    for c in keep],'value') for k in METRICS}
        summaries.append(dict(cutoff_mm=cutoff,height_scale=height,wave=wave,fixed_pair_angle_rms_deg=rms.pop(),
            n_planned=30,n_complete_before=len(rows),n_failed=len(bad),failure_counts=dict(collections.Counter(c['status'] for c in bad)),
            n_excluded=int(drop is not None),n_retained=len(keep),excluded_spawn_index=drop['spawn_index'] if drop else None,
            statistics_status='no_retained_observations' if not keep else ('mean_only_no_SD' if len(keep)==1 else 'mean_and_sample_SD'),
            forward_0_6_mm=stats(keep,'forward_0_6_mm'),forward_2_6_mm=stats(keep,'forward_2_6_mm'),
            forward_4_6_mm=stats(keep,'forward_4_6_mm'),raw_forward_0_6_mm=stats(rows,'forward_0_6_mm'),
            raw_forward_4_6_mm=stats(rows,'forward_4_6_mm'),contact_loss_percent=loss))
    paired=[]
    for cutoff,height in sorted({(c,h) for c,h,w in groups}):
        for label,data in [('raw',cells),('trimmed',retained)]:
            a={c['spawn_index']:c for c in data if cell_key(c)[:3]==(cutoff,height,2.5)}
            b={c['spawn_index']:c for c in data if cell_key(c)[:3]==(cutoff,height,3.)}
            common=sorted(set(a)&set(b))
            points=[dict(spawn_index=s,value=b[s]['forward_0_6_mm']-a[s]['forward_0_6_mm']) for s in common]
            paired.append(dict(cutoff_mm=cutoff,height_scale=height,selection=label,wave_a=2.5,wave_b=3.,
                difference_wave3_minus_wave2p5_mm=stats(points,'value'),wave3_positive_difference_count=sum(p['value']>0 for p in points)))
    return summaries,omissions,retained,paired


def plots(report,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import PercentFormatter
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'svg.fonttype':'none'})
    cutoffs=report['cutoffs_mm']; groups=report['summaries']; flat={c['wave']:c for c in report['flat_controls']}
    outputs=[]; n=len(cutoffs)
    def save(fig,name):
        for suffix in ['.png','.svg']:
            path=out/(name+suffix); fig.savefig(path,dpi=165,facecolor='white')
            outputs.append(dict(path=str(path),sha256=sha(path)))
        plt.close(fig)
    def panel(title,ylabel):
        fig,axs=plt.subplots(1,n,figsize=(max(10,6.2*n),6.8),sharey=True,squeeze=False)
        axs=axs[0]; fig.subplots_adjust(left=.075 if n==1 else .055,right=.98,bottom=.22,top=.74,wspace=.13)
        for ax,cutoff in zip(axs,cutoffs):
            ax.set_title(f'Nominal cutoff {cutoff:g} mm',pad=12)
            ax.axhline(0,color='#7d838b',lw=.8,alpha=.45,zorder=0); ax.grid(alpha=.16)
            for side in ['top','right']: ax.spines[side].set_visible(False)
        axs[0].set_ylabel(ylabel); fig.suptitle(title,y=.965,fontsize=17)
        return fig,axs
    def footer(fig,extra=''):
        fig.text(.075 if n==1 else .055,.115,
                 '30 shared spawns per rough group; one farthest-from-median complete 0–6 s observation omitted. Pale bars: sample SD.',fontsize=9.5)
        fig.text(.075 if n==1 else .055,.055,extra or 'Failures have no six-second score. Existing single flat control per wave has no repeat-based SD.',fontsize=9.5)
    def values(items,key):
        return [float('nan') if item[key]['mean'] is None else item[key]['mean'] for item in items]
    def sd(items,key):
        return [float('nan') if item[key]['std'] is None else item[key]['std'] for item in items]
    def legend(fig,ax):
        handles,labels=ax.get_legend_handles_labels()
        fig.legend(handles,labels,loc='upper center',bbox_to_anchor=(.52,.9),ncol=min(5,len(labels)),frameon=False)
    for xfield,xlabel,window,name in [
        ('fixed_pair_angle_rms_deg','Shared fixed 38-foot / 703-pair angle RMS (deg)','0_6','mean_std_vs_roughness'),
        ('fixed_pair_angle_rms_deg','Shared fixed 38-foot / 703-pair angle RMS (deg)','4_6','mean_std_4_6_vs_roughness'),
        ('height_scale','Terrain height multiplier','0_6','mean_std_vs_height')]:
        key='forward_'+window+'_mm'
        fig,axs=panel('Libo: displacement across cutoff and terrain roughness',f'Forward displacement, {window.replace("_","–")} s (mm)')
        for ax,cutoff in zip(axs,cutoffs):
            for wave,color in zip(WAVES,COLORS):
                rows=sorted([g for g in groups if g['cutoff_mm']==cutoff and g['wave']==wave],key=lambda g:g['height_scale'])
                xx=[0]+[g[xfield] for g in rows]; yy=[flat[wave][key]]+values(rows,key)
                ax.errorbar(xx,yy,yerr=[float('nan')]+sd(rows,key),fmt='none',ecolor=color,alpha=.25,capsize=3,zorder=1)
                ax.plot(xx,yy,'-o',color=color,lw=2,ms=5,label=f'Wave {wave:g}',zorder=3)
            ax.set_xlabel(xlabel)
        legend(fig,axs[0]); footer(fig); save(fig,name)
    heights=sorted({g['height_scale'] for g in groups})
    fig,axs=panel('Libo: displacement vs. wave number','Forward displacement, 0–6 s (mm)')
    palette=plt.get_cmap('viridis')(np.linspace(.12,.9,len(heights)))
    for ax,cutoff in zip(axs,cutoffs):
        ax.plot(WAVES,[flat[w]['forward_0_6_mm'] for w in WAVES],'-o',color='#575f68',lw=2,ms=5,label='Height x0')
        for height,color in zip(heights,palette):
            mapping={g['wave']:g for g in groups if g['cutoff_mm']==cutoff and g['height_scale']==height}
            if not mapping: continue
            assert set(mapping)==set(WAVES)
            rows=[mapping[w] for w in WAVES]
            ax.errorbar(WAVES,values(rows,'forward_0_6_mm'),yerr=sd(rows,'forward_0_6_mm'),fmt='none',ecolor=color,alpha=.25,capsize=3)
            ax.plot(WAVES,values(rows,'forward_0_6_mm'),'-o',color=color,lw=1.8,ms=4.5,label=f'Height x{height:g}')
        ax.set_xlabel('Wave number'); ax.set_xticks(WAVES)
    legend(fig,axs[0]); footer(fig,'Same retained observations as the roughness and contact-loss plots.'); save(fig,'mean_std_vs_wave_number')
    for metric,label in zip(METRICS,LABELS):
        fig,axs=panel(f'{label}: displacement vs. expected-support loss','Forward COM displacement, 0–6 s (mm)')
        for ax,cutoff in zip(axs,cutoffs):
            for wave,color in zip(WAVES,COLORS):
                rows=sorted([g for g in groups if g['cutoff_mm']==cutoff and g['wave']==wave],key=lambda g:g['height_scale'])
                losses=[g['contact_loss_percent']['0_6'][metric] for g in rows]
                xx=[0]+[float('nan') if loss['mean'] is None else loss['mean'] for loss in losses]
                xsd=[float('nan')]+[float('nan') if loss['std'] is None else loss['std'] for loss in losses]
                yy=[flat[wave]['forward_0_6_mm']]+values(rows,'forward_0_6_mm')
                ax.errorbar(xx,yy,xerr=xsd,yerr=[float('nan')]+sd(rows,'forward_0_6_mm'),fmt='none',ecolor=color,alpha=.23,capsize=3,zorder=1)
                ax.plot(xx,yy,'-o',color=color,lw=2,ms=5,label=f'Wave {wave:g}',zorder=3)
            ax.set(xlim=(-1.5,100),xlabel='Missing expected-support portion (%)')
            ax.xaxis.set_major_formatter(PercentFormatter(xmax=100,decimals=0))
        legend(fig,axs[0]); footer(fig,'Same 0–6 s window and retained observations on both axes; corresponding flat gait at the same CPG clock.'); save(fig,'displacement_vs_'+metric+'_loss_portion')
    fig,axs=panel('Libo: incomplete trials by height','Number of incomplete trials (all five waves)')
    failure_labels=[('state_divergence','State divergence','#bd4545'),('nonfinite_state','Nonfinite state','#df7c13'),('boundary_guard','Boundary guard','#8862b2')]
    max_failed=max(sum(g['n_failed'] for g in groups if g['cutoff_mm']==c and g['height_scale']==h)
                   for c,h in {(g['cutoff_mm'],g['height_scale']) for g in groups})
    for ax,cutoff in zip(axs,cutoffs):
        hh=sorted({g['height_scale'] for g in groups if g['cutoff_mm']==cutoff}); bottoms=np.zeros(len(hh))
        for status,label,color in failure_labels:
            yy=[sum(g['failure_counts'].get(status,0) for g in groups if g['cutoff_mm']==cutoff and g['height_scale']==h) for h in hh]
            ax.bar(hh,yy,width=.18,bottom=bottoms,color=color,alpha=.75,label=label); bottoms+=yy
        ax.set_xlabel('Terrain height multiplier'); ax.set_ylim(0,max(10,max_failed*1.15))
    legend(fig,axs[0]); footer(fig,'Each height has 150 planned independent conditions. Failures are excluded from displacement and stuck proportions.'); save(fig,'incomplete_condition_counts')
    fig,axs=panel('Raw wave 3.5 stopping criterion','Complete trials with final 2 s displacement ≤2 mm (%)')
    for ax,cutoff in zip(axs,cutoffs):
        rows=sorted([r for r in report['stop_judgments'] if r['cutoff_mm']==cutoff],key=lambda r:r['height_scale'])
        xx=[r['height_scale'] for r in rows]; yy=[float('nan') if r['raw_stuck_fraction'] is None else 100*r['raw_stuck_fraction'] for r in rows]
        ax.plot(xx,yy,'-o',color=COLORS[-1],lw=2,label='Raw complete trials')
        ax.axhline(80,color='#8f5656',ls='--',lw=1.2,label='80% threshold')
        for row,y in zip(rows,yy):
            if np.isfinite(y):
                ax.annotate(f'n={row["n_complete"]}',(row['height_scale'],y),xytext=(0,8),textcoords='offset points',ha='center',fontsize=9)
                if not row['minimum_complete_met']:
                    ax.scatter([row['height_scale']],[y],marker='x',s=65,color='#bd4545',zorder=4)
        ax.set(xlabel='Terrain height multiplier',ylim=(-3,108)); ax.yaxis.set_major_formatter(PercentFormatter(xmax=100,decimals=0))
    legend(fig,axs[0]); footer(fig,'No outlier omission for this decision. At least 24/30 complete trials required; red crosses indicate insufficient complete trials.'); save(fig,'raw_wave3p5_stopping_criterion')
    return outputs


def build_report(cells,failures,flat,planned,series,sources,metadata,out):
    cells=sorted(cells,key=cell_key); failures=sorted(failures,key=cell_key)
    keys={cell_key(c) for c in cells}; badkeys={cell_key(c) for c in failures}; plannedkeys={cell_key(c) for c in planned}
    assert len(keys)==len(cells) and len(badkeys)==len(failures) and not(keys & badkeys)
    assert keys | badkeys==plannedkeys
    add_contact_metrics(cells,flat,sources)
    summaries,omissions,retained,paired=summarize(cells,failures,planned)
    report=dict(metadata,complete=True,rough_condition_count=len(planned),preserved_flat_control_count=5,
        raw_complete_count=len(cells),failed_condition_count=len(failures),failure_counts=dict(collections.Counter(c['status'] for c in failures)),
        excluded_for_statistics_count=len(omissions),retained_rough_count=len(retained),retained_total_count=len(retained)+5,
        cutoffs_mm=sorted({float(c['cutoff_mm']) for c in planned}),heights=sorted({c['height_scale'] for c in planned}),waves=WAVES,
        flat_controls=flat,cells=cells,failed_cells=failures,summaries=summaries,excluded_observations=omissions,
        paired_wave2p5_3=paired,stop_rule=STOP_RULE,stop_judgments=stop_judgments(cells,failures,planned),
        source_hashes=sources,analysis_source_sha256=sha(__file__),
        statistics='Per cutoff×height×wave group, omit one completed 0–6 s point farthest from the group median; tie lowest spawn index. Empty groups omit none. A one-complete group retains none. Same retained observations for all windows and contact metrics; sample SD ddof=1 only with at least two retained points. Missing means/SD are null, never zeros.',
        displacement_definition='Mass-weighted COM including authored local offsets; XY displacement projected on EACH initial headward direction.',
        contact_definitions=dict(instantaneous='Equal mean of missing/expected feet per valid 10 ms snapshot; expected=0 omitted.',
            cycle='Pool missing/expected samples within each nominal 1 s CPG cycle, then average cycle fractions.',
            pooled='Total missing expected foot-time samples / total expected flat foot-time samples.'),
        contact_threshold_N=1e-8,contact_snapshot_dt_s=.01,
        frozen_native_checks=dict(full_173_PD=True,flat_cluster_amplitudes=True,cpu_TGS_64_16=True,
            physics_dt_s=.0005,CPG_command_hold_s=.001,threads_per_worker=2,library_threads=1,
            contact_offset_m=.00002,rest_offset_m=0.,all_step_collision_margin_m=.015,
            startup_and_final_native_validation='New completed runs independently checked; reused data preserved from validated previous report; reused flat controls re-extracted and checked.'),
        limitations=['One fixed Libo mother surface and 30 shared spatial/heading observations, not independent terrain realizations.',
            'Means and SD are conditional on complete numerical/boundary-safe six-second trials; no failure-as-zero substitution.',
            'One flat reference per wave; no repeat-based flat SD.',
            'Missing-contact measures compare 38 toes at the same CPG clock, so phase shifts contribute; this is not calibrated IID erasure probability.',
            'Contact-loss curve intersections may correspond to different terrain heights across waves.',
            'Nominal cutoff describes restored components; preserved legacy triangle interpolation can retain weak higher-frequency content.',
            'Stop rule is descriptive final-two-second displacement, not proof of a steady-state or permanent inability to progress.',
            'One-point omission may exclude a physical stuck or reverse outcome; raw stop decisions never use the omission.'],
        figure_review=dict(passed=False,status='awaiting_visual_inspection'))
    report['stop_judgements']=controller_judgements(report['stop_judgments'])
    out.mkdir(parents=True,exist_ok=False)
    (out/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
    np.savez_compressed(out/'forward_timeseries.npz',t_s=np.arange(601)/100,**series)
    report['forward_timeseries_sha256']=sha(out/'forward_timeseries.npz')
    dump(out/'RESULTS.json',report)
    report['figures']=plots(report,out)
    dump(out/'RESULTS.json',report)
    print(json.dumps(dict(results=str(out),rough_planned=len(planned),complete=len(cells),failed=len(failures),
                         retained=report['retained_total_count'],stop_judgments=report['stop_judgments']),ensure_ascii=False),flush=True)
    return report


def dispatch_timing(sim,supervisor,sources):
    elapsed=float(supervisor['elapsed_s'])
    dispatch_path=sim/'DISPATCH.json'
    dispatch=read(dispatch_path) if dispatch_path.is_file() else {}
    if dispatch_path.is_file(): source(dispatch_path,sources)
    history=sim/'dispatch_resume_history'
    snapshots=sorted(history.glob('supervisor_before_*.json')) if history.is_dir() else []
    gap_s=0.; known=True; gaps=[]
    for index,path in enumerate(snapshots,1):
        before=read(path); source(path,sources)
        next_path=history/f'dispatch_before_{index+1:02d}.json'
        after=read(next_path) if next_path.is_file() else dispatch
        if next_path.is_file(): source(next_path,sources)
        if 'resumed_unix' not in after or 'started_unix' not in before or 'elapsed_s' not in before:
            known=False; gaps.append(dict(before=str(path),observed_gap_s=None)); continue
        gap=max(0.,float(after['resumed_unix'])-float(before['started_unix'])-float(before['elapsed_s']))
        gap_s+=gap; gaps.append(dict(before=str(path),observed_gap_s=gap))
    # Absence of resume receipts cannot prove the size of a known interruption gap.
    if not snapshots and (dispatch.get('resumed_unix') is not None or supervisor.get('resume_count',0)):
        known=False
    return dict(total_elapsed_wall_s=elapsed,
                physical_campaign_wall_s=max(0.,elapsed-gap_s) if known else None,
                unobserved_dispatch_gap_s=gap_s if known else None,
                interruption_gap_receipts=gaps,interruption_gap_duration_known=known,
                timing_note='Active dispatch elapsed excludes observable interruption gaps between the last pre-interruption supervisor receipt and the next resumed dispatcher. This is not summed physics-step CPU time. The exact interruption instant is unknown; if receipts are insufficient the active estimate is null.')


def analyze_stage(path,out):
    cells,failed,runs,series,sources,campaign,supervisor,planned,sim=collect_stage(path)
    _,_,flat,oldseries,oldsources,old=preserved_previous()
    for cell in flat: series[series_key(cell)]=oldseries[series_key(cell)]
    sources.update(oldsources)
    assert campaign['fixed_pd_sha256']==old['fixed_pd_sha256'] and campaign['profile_sha256']==old['profile_sha256']
    timing=dispatch_timing(sim,supervisor,sources)
    metadata=dict(mode='stage',stage_simulation=str(sim),planned_total_condition_count=len(planned),
        new_planned_condition_count=len(planned),new_complete_count=len(cells),new_failed_count=len(failed),
        new_runs=runs,new_worker_attempts=len(runs),new_condition_executions=sum(len(r['source_ids']) for r in runs),
        additional_condition_executions=sum(len(r['source_ids']) for r in runs)-len(planned),
        external_unscored_attempts=sum(r.get('external_interruption',False) for r in runs),
        fixed_pd_sha256=campaign['fixed_pd_sha256'],profile_sha256=campaign['profile_sha256'],native_worker_bitexact_to_original=True,
        campaign_terrains=campaign.get('terrains'),planned_cells=planned,
        reused_previous_report=str(PREVIOUS/'RESULTS.json'))
    metadata.update(timing)
    return build_report(cells,failed,flat,planned,series,sources,metadata,out)


def result_path(path):
    path=Path(path)
    if not path.is_absolute():
        path=EXPANSION_BASE/path
    if path.name=='RESULTS.json': return path
    if (path/'RESULTS.json').is_file(): return path/'RESULTS.json'
    return path/'results/RESULTS.json'


def combine_stages(paths,out):
    cells,failed,flat,series,sources,old=preserved_previous()
    planned=[dict(c) for c in cells+failed]
    stages=[]
    for path in paths:
        p=result_path(path); report=read(p); assert report['complete'] and report['mode']=='stage'
        assert report['fixed_pd_sha256']==old['fixed_pd_sha256'] and report['profile_sha256']==old['profile_sha256']
        assert report['flat_controls']==flat
        ts=p.parent/'forward_timeseries.npz'
        assert sha(ts)==report['forward_timeseries_sha256']
        source(p,sources); source(ts,sources); sources.update(report['source_hashes'])
        cells.extend(copy.deepcopy(report['cells'])); failed.extend(copy.deepcopy(report['failed_cells']))
        planned.extend(copy.deepcopy(report['planned_cells']))
        with np.load(ts) as z:
            np.testing.assert_array_equal(z['t_s'],np.arange(601)/100)
            for cell in report['cells']:
                key=series_key(cell); assert key not in series, key
                fwd=z[key].copy(); assert fwd.shape==(601,) and np.isfinite(fwd).all()
                np.testing.assert_allclose(fwd[-1],cell['forward_0_6_mm'],atol=1e-10,rtol=0)
                series[key]=fwd
        stages.append(dict(results_path=str(p),sha256=sha(p),stage_simulation=report['stage_simulation'],
            planned=report['rough_condition_count'],complete=report['raw_complete_count'],failed=report['failed_condition_count'],
            elapsed_s=report['total_elapsed_wall_s'],active_dispatch_elapsed_s=report['physical_campaign_wall_s'],
            observed_gap_s=report['unobserved_dispatch_gap_s'],worker_attempts=report['new_worker_attempts']))
    metadata=dict(mode='combined',planned_total_condition_count=len(planned)+5,
        preserved_previous_report=str(PREVIOUS/'RESULTS.json'),preserved_previous_rough_attempts=900,
        new_planned_condition_count=sum(s['planned'] for s in stages),stages=stages,
        new_complete_count=sum(s['complete'] for s in stages),new_failed_count=sum(s['failed'] for s in stages),
        stage_elapsed_s_sum=sum(s['elapsed_s'] for s in stages),
        stage_active_dispatch_elapsed_s_sum=(sum(s['active_dispatch_elapsed_s'] for s in stages)
                                            if all(s['active_dispatch_elapsed_s'] is not None for s in stages) else None),
        timing_note='Sum of stage dispatcher elapsed times and separately observed active dispatch estimates; neither is summed per-environment physics CPU time. Unknown interruption gaps leave the active sum null.',
        fixed_pd_sha256=old['fixed_pd_sha256'],profile_sha256=old['profile_sha256'],native_worker_bitexact_to_original=True,
        planned_cells=planned)
    return build_report(cells,failed,flat,planned,series,sources,metadata,out)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['stage','combine'],nargs='?')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--stage',type=Path); mode.add_argument('--combine',action='store_true')
    parser.add_argument('--stages',type=Path,nargs='+'); parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    assert bool(args.stage) != bool(args.combine or args.mode=='combine'), 'Select stage or combine'
    assert args.mode is None or (args.mode=='stage' and args.stage) or (args.mode=='combine' and args.stages)
    assert not args.out.exists(), 'Preserve completed analysis: choose a fresh output directory'
    if args.stage:
        assert not args.stages
        analyze_stage(args.stage,args.out)
    else:
        assert args.stages
        combine_stages(args.stages,args.out)
