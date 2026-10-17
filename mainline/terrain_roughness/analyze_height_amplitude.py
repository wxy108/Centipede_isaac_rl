"""Account for 84 fixed-cutoff trials, validate completed trajectories and compare forward COM motion."""
from pathlib import Path
import os
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
import json,hashlib,ast,argparse,shutil
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from PIL import Image

ROOT=Path(__file__).resolve().parent;MAIN=ROOT.parent;BASE=ROOT/'height_amplitude_4mm';SIM=BASE/'simulation';OUT=BASE/'results'
WAVES=[1.5,2,2.5,3,3.5,4]
COLORS=['#2474b5','#df7c13','#26915a','#bd4545','#8862b2','#765441']
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,r):Path(p).write_text(json.dumps(r,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')


def validate_and_extract(attempt):
    campaign=read(SIM/'campaign.json');supervisor=read(SIM/f'supervisor_{attempt}.json')
    recovery=read(SIM/'recovery_status.json');assert recovery['complete']
    original={r['job']:r for r in supervisor['finished']+supervisor['current']}
    recovered={r['job']:r for r in recovery['jobs']}
    assert len(campaign['jobs'])==28 and campaign['condition_count']==84
    resolution={};incomplete=[];failed_runs=[]
    for job in campaign['jobs']:
        name=job['name'];old=original.get(name)
        selected=attempt if old and old['exit']==0 else 'v2_capture'
        status=old if selected==attempt else recovered[name]
        d=SIM/'runs'/selected/name
        active=read(d/'active.json');assert active['exit_code']==status['exit']
        resolution[name]=dict(path=str(d),attempt=selected,exit_code=status['exit'],complete=status['exit']==0)
    assert sum(r['complete'] for r in resolution.values())==26
    assert all((SIM/'runs'/r['attempt']/name/'active.json').exists() for name,r in resolution.items())
    for p,h in campaign['pins'].items():assert sha(p)==h,p
    pd=read(MAIN/'pd_wave4_firm20.json')['absolute_joint_pd'];terrain=read(BASE/'REPORT.json')
    terrain_lookup={(r['site'],r['height_scale']):r for r in terrain['terrains']}
    cells=[];runs=[];series={};shared=None;sources={str(SIM/'campaign.json'):sha(SIM/'campaign.json'),str(BASE/'REPORT.json'):sha(BASE/'REPORT.json')}
    for job in campaign['jobs']:
        d=Path(resolution[job['name']]['path']);complete=resolution[job['name']]['complete']
        for file in ['backend.npz','effective_fields_after_pd.json','amplitude_modulation.json']:
            sources[str(d/file)]=sha(d/file)
        rb=read(d/'readback.json');plant=read(d/'matched_plant_check.json')
        amp=read(d/'amplitude_modulation.json');threads=read(d/'thread_settings.json');binding=read(d/'articulation_binding.json')
        assert rb['dt_s']==.0005 and rb['solver']=='TGS' and not rb['gpu_dynamics']
        assert threads['tasking']==threads['physics']==2 and threads['torch_threads']==1
        assert binding['row_order_verified'] and plant['passed'] and plant['complete_uniform_173_joint_stiffness'] and plant['complete_uniform_173_joint_damping']
        assert all(r['position']==64 and r['velocity']==16 for r in read(d/'iteration_readback.json'))
        names=rb['joint_names'];assert len(names)==173 and set(names)==set(pd)
        with np.load(d/'backend.npz') as z:
            expected=np.array([pd[n] for n in names],dtype=z['kp'].dtype)
            np.testing.assert_array_equal(z['kp'],np.repeat(expected[None,:,0],3,axis=0))
            np.testing.assert_array_equal(z['kd'],np.repeat(expected[None,:,1],3,axis=0))
            mass=z['masses'].astype(float)
        assert amp['other_targets_bitexact'] and amp['limits_passed']
        for c,i in zip(amp['cells'],job['source_ids']):
            assert all(c[k]==campaign['flat_cluster_cells'][i][k] for k in ['wave','A_deg','B_deg','body_factor','leg_factor'])
            assert c['height_scale']==job['height_scale']
        tr=terrain_lookup[job['site'],job['height_scale']]
        manifest=read(d/'manifest.json');assert all(r['surface_sha256']==tr['sha256'] for r in manifest['rows'])
        if not complete:
            failure=read(d/'BOUNDARY_FAILURE.json')
            evidence_files=['BOUNDARY_FAILURE.json','failure_state.npz','trajectory_partial.npz','worker.log','implementation_provenance.json']
            for file in evidence_files:sources[str(d/file)]=sha(d/file)
            offenders=[e['environment_index'] for e in failure['environments'] if min(e['margins_m'])<=.015]
            assert len(offenders)==1
            offender=failure['environments'][offenders[0]]
            assert offender['max_body_linear_speed_m_s']>1e6
            with np.load(d/'trajectory_partial.npz') as z:
                assert abs(z['t'][-1]-failure['saved_regular_frame_end_s'])<1e-12
            old_failure=SIM/'runs'/attempt/job['name']/'BOUNDARY_FAILURE.json'
            reproduced=None
            if old_failure.exists():
                original_failure=read(old_failure);sources[str(old_failure)]=sha(old_failure)
                reproduced=all(failure[k]==original_failure[k] for k in ['t','minimum_body_center_margin_m'])
                assert reproduced
            failed_runs.append(dict(job=job['name'],path=str(d),native_PD_exact=True,startup_settings_passed=True,
                failure=failure,original_failure_path=str(old_failure) if old_failure.exists() else None,
                same_failure_time_and_margin_on_bounded_capture_repeat=reproduced,
                native_error_audit='Final audit unavailable: worker terminated at boundary guard; raw log preserved.'))
            for e,c in enumerate(amp['cells']):
                incomplete.append(dict(site=job['site'],height_scale=job['height_scale'],level=job['level'],wave=c['wave'],
                    run=job['name'],run_path=str(d),environment_index=e,status='state_divergence' if e in offenders else 'interrupted_by_peer_guard',
                    stop_time_s=failure['t'],last_saved_frame_s=failure['saved_regular_frame_end_s'],
                    fixed_pair_angle_rms_deg=tr['fixed_pair_angle_rms_deg'],height_Sq_mm=tr['height_Sq_mm'],
                    forward_0_6_mm=None,forward_4_6_mm=None,failure_evidence=str(d/'BOUNDARY_FAILURE.json')))
            continue
        result=read(d/'results.json')
        assert result['complete'] and result['finite'] and result['whole_batch_finite']
        assert result['actual_duration_s']==6 and result['finite_check_interval_s']==.0005
        assert not read(d/'native_error_audit.json')['errors']
        guard=read(d/'crop_guard.json');assert guard['passed'] and min(guard['saved_pose_margins_m'])>.015
        sources[str(d/'trajectory.npz')]=sha(d/'trajectory.npz')
        baseline_state_repeat=None
        with np.load(d/'trajectory.npz') as z:
            pose=z['pose'].astype(float);times=z['t'];bn=z['body_names'].tolist()
            if job['height_scale']==1:
                old_path=ROOT/'shortwave/simulation_v2/runs/v2'/f"{job['site']}_restore4_{job['group']}"/'trajectory.npz'
                sources[str(old_path)]=sha(old_path)
                with np.load(old_path) as old:
                    baseline_state_repeat=dict(prior_trajectory=str(old_path),equal_arrays={k:bool(np.array_equal(z[k],old[k])) for k in ['t','pose','q','v']},max_body_position_difference_m=float(np.max(abs(z['pose'][...,:3]-old['pose'][...,:3]))))
            assert pose.shape==(601,3,251,7) and np.isfinite(pose).all()
            assert np.isfinite(z['q']).all() and np.isfinite(z['v']).all()
        np.testing.assert_allclose(times,np.arange(601)*.01,atol=1e-12,rtol=0)
        fields=read(d/'effective_fields_after_pd.json')['attributes']
        cf={a['path'].rsplit('/',1)[-1]:a for a in fields if a['name']=='physics:centerOfMass'}
        assert set(cf)==set(bn) and all(a['authored'] for a in cf.values())
        local=np.array([ast.literal_eval(cf[n]['value']) for n in bn],float)
        assert np.isfinite(local).all() and np.all(mass>0)
        if shared is None:shared=(bn,local,mass[0])
        assert shared[0]==bn;np.testing.assert_array_equal(shared[1],local)
        np.testing.assert_array_equal(mass,np.broadcast_to(shared[2],mass.shape))
        quat=pose[...,3:];norm=np.linalg.norm(quat,axis=-1,keepdims=True);assert np.max(abs(norm-1))<2e-6;quat=quat/norm
        uv=np.cross(quat[...,1:],local);rotated=local+2*(quat[...,:1]*uv+np.cross(quat[...,1:],uv))
        subset=quat[[0,300,600]]
        chk=Rotation.from_quat(subset[...,[1,2,3,0]].reshape(-1,4)).apply(np.broadcast_to(local,subset.shape[:-1]+(3,)).reshape(-1,3))
        assert np.max(abs(chk-rotated[[0,300,600]].reshape(-1,3)))<1e-14
        com=((pose[...,:3]+rotated)*mass[None,:,:,None]).sum(2)/mass.sum(1)[None,:,None]
        forward=pose[0,:,bn.index('link_body_0'),:2]-pose[0,:,bn.index('link_body_20'),:2]
        forward/=np.linalg.norm(forward,axis=-1,keepdims=True)
        np.testing.assert_allclose(forward,np.tile([-1,0],(3,1)),atol=2e-6,rtol=0)
        fwd=((com[...,:2]-com[0,:,:2])*forward[None]).sum(2)*1000
        with np.load(d/'cluster_contacts.npz') as z:assert np.isfinite(z['force']).all()
        tr=terrain_lookup[job['site'],job['height_scale']]
        manifest=read(d/'manifest.json');assert all(r['surface_sha256']==tr['sha256'] for r in manifest['rows'])
        for e,c in enumerate(amp['cells']):
            cells.append(dict(site=job['site'],height_scale=job['height_scale'],level=job['level'],wave=c['wave'],run=job['name'],environment_index=e,
                fixed_pair_angle_rms_deg=tr['fixed_pair_angle_rms_deg'],height_Sq_mm=tr['height_Sq_mm'],A_deg=c['A_deg'],B_deg=c['B_deg'],
                forward_0_6_mm=float(fwd[-1,e]),forward_4_6_mm=float(fwd[-1,e]-fwd[400,e]),
                com_delta_xyz_mm=((com[-1,e]-com[0,e])*1000).tolist(),com_path_xy_mm=float(np.linalg.norm(np.diff(com[:,e,:2],axis=0),axis=1).sum()*1000),
                forward_axis_xy=forward[e].tolist(),surface_path=tr['path'],surface_sha256=tr['sha256']))
            series[f'{job["site"]}_{job["level"]}_w{c["wave"]:g}']=fwd[:,e]
        runs.append(dict(job=job['name'],height_scale=job['height_scale'],site=job['site'],trajectory_path=str(d/'trajectory.npz'),
                         trajectory_sha256=sources[str(d/'trajectory.npz')],native_PD_exact=True,native_error_count=0,thread_readback=threads,
                         minimum_body_center_margin_mm=1000*min(guard['saved_pose_margins_m']),wall_s=result['wall_s'],baseline_state_repeat=baseline_state_repeat))
    assert len(cells)==78 and len(incomplete)==6
    assert sum(c['status']=='state_divergence' for c in incomplete)==2
    prior=read(ROOT/'shortwave/displacement_roughness/RESULTS.json')
    repeat=[]
    for c in cells:
        ref=next(r for r in cells if r['site']==c['site'] and r['wave']==c['wave'] and r['height_scale']==1)
        c['change_from_height1_0_6_mm']=c['forward_0_6_mm']-ref['forward_0_6_mm']
        if c['height_scale']==1:
            old=next(r for r in prior['cells'] if r['site']==c['site'] and r['wave']==c['wave'] and r['level']=='restore4')
            repeat.append(dict(site=c['site'],wave=c['wave'],prior_forward_mm=old['forward_0_6_mm'],current_forward_mm=c['forward_0_6_mm'],
                               difference_mm=c['forward_0_6_mm']-old['forward_0_6_mm'],same_surface_sha256=True))
            assert c['surface_sha256']==next(t['surface_sha256'] for t in prior['roughness'] if t['site']==c['site'] and t['level']=='restore4')
    report=dict(complete=True,meaning='All planned conditions accounted for; incomplete conditions have no six-second displacement',all_conditions_completed=False,condition_count=84,completed_condition_count=len(cells),incomplete_cells=incomplete,failed_runs=failed_runs,run_resolution=resolution,physics_seconds_per_condition=6,attempts=[attempt,'v2_capture'],runs=runs,cells=cells,
                height_scales=campaign['height_scales'],roughness_definition=terrain['roughness_sampling'],baseline_repeat=repeat,
                displacement_definition='Native mass-weighted whole-body COM including authored local COM offsets, projected onto initial headward XY direction (~-X); net displacement, not path length',
                time_windows_s=[[0,6],[4,6]],source_hashes=sources,
                checks=dict(input_hashes=True,frozen_flat_cluster_amplitudes=True,all173_native_PD_exact=True,physics_0p5ms=True,
                            CPG_1ms_two_step_hold_source_unchanged=True,TGS64_16=True,main_batches_four_workers_two_threads=True,bounded_diagnostic_one_worker_two_threads=True,completed_runs_all_step_finite=True,
                            collision_isolation_source_unchanged=True,completed_runs_crop_interior_guard=True,completed_runs_native_error_free=True,completed_trajectories_saved=True,failed_state_and_partial_trajectories_preserved=True,
                            common_native_masses_and_local_COM=True,rotation_library_crosscheck=True),
                initialization_note=campaign['initialization'],physics_roi_m=campaign['terrain_roi_m'],supervisor_wall_s=supervisor['elapsed_s'],
                limitations=['78 complete six-second observations; two states diverged and four peers were stopped by their shared worker guard. No six-second value is assigned to those six conditions.',
                             'One planned run per condition; one bounded failure-capture repeat is diagnostic only, not an independent statistical sample. No error bars.',
                             '0-6 s includes initial settling; 4-6 s is a supplementary window, not proof of steady state.',
                             'Fixed-neutral roughness is a shared terrain descriptor, not a contact/anchoring measurement.',
                             'Scale=1 repeats all prior t/pose/q/v arrays bit for bit; this establishes reproducibility, not physical accuracy.',
                             'High-height failure root cause is unresolved; no PD, timing, solver, geometry or boundary changes were made to pass it.'])
    return report,series


def plots(report):
    cells=report['cells'];output=[]
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    for key,name,title,ylabel in [
        ('forward_0_6_mm','displacement_vs_roughness.png','Fixed 4 mm cutoff: forward displacement vs. terrain roughness','Forward displacement, 0-6 s (mm)'),
        ('forward_4_6_mm','displacement_4_6_vs_roughness.png','Fixed 4 mm cutoff: displacement during the final 2 seconds','Forward displacement, 4-6 s (mm)')]:
        fig,axs=plt.subplots(1,2,figsize=(13,6.5),sharey=True);fig.subplots_adjust(left=.085,right=.98,bottom=.21,top=.75,wspace=.13)
        fig.suptitle(title,fontsize=16,weight='bold',y=.965)
        for ax,site in zip(axs,['libo','narvik']):
            for wave,color in zip(WAVES,COLORS):
                rows=sorted([r for r in cells if r['site']==site and r['wave']==wave],key=lambda r:r['height_scale'])
                ax.plot([r['fixed_pair_angle_rms_deg'] for r in rows],[r[key] for r in rows],'-o',lw=2,color=color,label=f'Wave {wave:g}',markersize=5.5)
            ax.set(title=site.capitalize(),xlabel='Fixed-foot pair-angle RMS (deg)');ax.grid(alpha=.18)
        all_y=[c[key] for c in cells];pad=max(3,.07*np.ptp(all_y));axs[0].set_ylim(min(0,min(all_y))-pad,max(all_y)+pad);axs[0].set_ylabel(ylabel)
        fig.legend(*axs[0].get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.54,.91),ncol=6,frameon=False)
        fig.text(.085,.105,'Terrain height multipliers: 0.25, 0.5, 0.75, 1, 1.25, 1.5, 2. Same phases, XY layout and 0.5 mm grid.',fontsize=10,color='#52616f')
        fig.text(.085,.06,'Fixed 38-foot / 703-pair roughness; shared position + all-heading average. Robot amplitudes and PD unchanged.',fontsize=10,color='#52616f')
        fig.text(.085,.018,'Six missing Libo points at height x1.5/x2: 2 divergent states + 4 peer interruptions; not zero displacement.',fontsize=9,color='#9a352e')
        path=OUT/name;fig.savefig(path,dpi=160,facecolor='white');plt.close(fig)
        with Image.open(path) as im:im.load()
        output.append(dict(path=str(path),sha256=sha(path)))
    fig,axs=plt.subplots(1,2,figsize=(13,6.5),sharey=True);fig.subplots_adjust(left=.085,right=.98,bottom=.18,top=.73,wspace=.13)
    palette=plt.cm.viridis(np.linspace(.08,.9,7));fig.suptitle('Fixed 4 mm cutoff: forward displacement vs. wave number',fontsize=17,weight='bold',y=.965)
    for ax,site in zip(axs,['libo','narvik']):
        for scale,color in zip(report['height_scales'],palette):
            rows=sorted([r for r in cells if r['site']==site and r['height_scale']==scale],key=lambda r:r['wave'])
            ax.plot([r['wave'] for r in rows],[r['forward_0_6_mm'] for r in rows],'-o',lw=2,color=color,label=f'Height x{scale:g}',markersize=5.5)
        ax.set(title=site.capitalize(),xlabel='Wave number');ax.set_xticks(WAVES,[f'{w:g}' for w in WAVES]);ax.grid(alpha=.18)
    y=[c['forward_0_6_mm'] for c in cells];pad=max(3,.07*np.ptp(y));axs[0].set_ylim(min(0,min(y))-pad,max(y)+pad);axs[0].set_ylabel('Forward displacement, 0-6 s (mm)')
    fig.legend(*axs[0].get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.53,.9),ncol=4,frameon=False)
    fig.text(.085,.06,'Actual six-second Isaac CPU PhysX runs. Flat-cluster gait amplitudes and Sep 17 wave-4 PD fixed.',fontsize=10,color='#52616f')
    fig.text(.085,.018,'Libo x1.5/x2 have no six-second values for waves 1.5, 2, 2.5; only completed observations are plotted.',fontsize=9,color='#9a352e')
    path=OUT/'displacement_vs_wave_number.png';fig.savefig(path,dpi=160,facecolor='white');plt.close(fig)
    with Image.open(path) as im:im.load()
    output.append(dict(path=str(path),sha256=sha(path)))
    return output


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--attempt',default='v1');args=ap.parse_args()
    assert not OUT.exists(),'Do not overwrite completed analysis'
    report,series=validate_and_extract(args.attempt)
    OUT.mkdir();shutil.copy2(__file__,OUT/'analysis_source.py')
    report['analysis_source']=str(OUT/'analysis_source.py');report['analysis_source_sha256']=sha(__file__)
    np.savez_compressed(OUT/'forward_timeseries.npz',t_s=np.arange(601)/100,**series)
    report['figures']=plots(report);dump(OUT/'RESULTS.json',report)
    print('Accounted for 84 conditions: 78 completed, 2 divergent, 4 peer-interrupted; generated three figures.',flush=True)
    print(json.dumps({'max_scale1_repeat_difference_mm':max(abs(r['difference_mm']) for r in report['baseline_repeat']),
                     'minimum_boundary_margin_mm':min(r['minimum_body_center_margin_mm'] for r in report['runs'])},indent=2),flush=True)
