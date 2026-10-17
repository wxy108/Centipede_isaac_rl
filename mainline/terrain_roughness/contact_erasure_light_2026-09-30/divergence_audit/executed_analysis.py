from pathlib import Path
import json,sys,hashlib,time,collections
import numpy as np
ROOT=Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness');sys.path.insert(0,str(ROOT))
import continue_contact_erasure_light as inv
c,jobs,attempts,pending,external,old=inv.inventory()
BASE=ROOT/'contact_erasure_light_2026-09-30';OUT=BASE/'divergence_audit';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
complete=set();failed={};type_counts=collections.Counter();family_counts=collections.Counter()
for x in attempts:
    if x.get('reused'):continue
    if x['status']=='finished':complete.update((x['job'],sid) for sid in x['keep'])
    for sid in x.get('offender_source_ids',[]):
        key=(x['job'],sid);assert key not in failed
        d=Path(x['run_path']);kind='BOUNDARY_FAILURE.json' if (d/'BOUNDARY_FAILURE.json').exists() else 'NUMERICAL_FAILURE.json'
        f=read(d/kind);local=read(d/'active.json')['source_ids'].index(sid);e=next(q for q in f['environments'] if q['source_id']==sid)
        with np.load(d/'trajectory_partial.npz') as z:
            fastest=int(np.argmax(abs(z['v'][-1,local])));joint=str(z['joint_names'][fastest]);family_counts[joint.split('_')[0]]+=1
        type_counts[kind]+=1
        failed[key]=dict(job=key[0],source_id=sid,wave=jobs[key[0]]['cells'][sid]['wave'],bin_s=jobs[key[0]]['cells'][sid]['erase_bin_s'],p=jobs[key[0]]['cells'][sid]['p'],mask_seed=jobs[key[0]]['cells'][sid]['mask_seed'],failure_path=str(d/kind),failure_sha256=sha(d/kind),t_s=f['t'],max_body_linear_speed_m_s=e['max_body_linear_speed_m_s'],max_joint_speed_rad_s=e['max_abs_joint_velocity_rad_s'],last_recorded_fastest_joint=joint)
assert not complete&set(failed);assert len(complete)+len(failed)+len(pending)==270
p=BASE/'simulation/runs/v1/pilot_p050_bin010_w3';er=read(p/'ERASURE_RESULT.json')
with np.load(p/'trajectory_partial.npz') as z:
    tt=z['t'];vv=np.linalg.norm(z['body_velocity'][:,0,:,:3],axis=-1).max(-1);qv=abs(z['v'][:,0,:]).max(-1)
    keep=(tt>=2.80)&(tt<=2.89);timeline=[dict(t_s=float(t),max_body_speed_m_s=float(v),max_joint_speed_rad_s=float(q)) for t,v,q in zip(tt[keep],vv[keep],qv[keep])]
with np.load(p/'erasure_contacts.npz') as z:
    ft=z['t'];force=z['leg_normal_force'][:,0].sum(-1)+np.linalg.norm(z['chassis_normal_force'][:,0],axis=-1)
    ix=np.flatnonzero((ft>2.85)&(ft<=2.87));mi=ix[int(np.argmax(force[ix]))]
    peak=dict(t_s=float(ft[mi]),sum_leg_resultant_magnitudes_plus_chassis_resultant_N=float(force[mi]))
b=Path('X:/Projects/_archive/Codex/2026-09-11/con/round9_wave_amplitude_2026-09-14/trend_revision_2026-09-15')
mr=read(b/'IID_MAIN_REGISTRY.json');rest=read(b/'IID_RESTORATION_DIAGNOSTICS.json');profile=read(b.parent/'OPERATING_PROFILE.json')
report=dict(schema='contact_erasure_existing_data_divergence_audit_v1',created_unix=time.time(),scope='Read-only analysis of existing simulations. No new random trial, diagnostic simulation, gain/timestep/armature tuning, or rough expansion.',stochastic_snapshot=dict(planned=270,complete=len(complete),numerical_divergence=len(failed),unscored=len(pending),external_interrupted_attempts=len(external),not_completed_not_zero=True),failure_file_types=dict(type_counts),last_frame_fastest_joint_families=dict(family_counts),failed_conditions=list(failed.values()),minimum_offender_peak_body_v_m_s=min(v['max_body_linear_speed_m_s'] for v in failed.values()),pilot=dict(path=str(p),boundary_failure=read(p/'BOUNDARY_FAILURE.json')['environments'],timeline=timeline,contact_peak=peak,last_three_restoration_1ms_windows_have_zero_force=all(s['peak_normal_N']==0 for s in er['restore_samples'][-6:]),max_native_restore_depth_mm=max(s['max_native_depth_m'] for s in er['restore_samples'])*1000,channel_kept_mass_inertia_PD=True,restore_observation_limits='All robot contacts, first1ms; not exclusive to restored foot. Foot origin height is not shape clearance. Saved velocity stride10ms cannot identify the very first unstable0.5ms step.'),mujoco_round9=dict(registry=str(b/'IID_MAIN_REGISTRY.json'),registry_sha256=sha(b/'IID_MAIN_REGISTRY.json'),conditions=len(mr),complete=sum(q['complete'] for q in mr),warning_cases=sum(any(q['warnings']) for q in mr),max_restore_depth_mm=rest['max_depth_mm'],profile=profile['MuJoCo'],erasure='19bilateral pairs; independent0.5s clocks,250–500ms or longer bursts; NOT10/20/50ms independent38leg switching',added_armature=0),mujoco_august=dict(model='C:/Users/wxy22/Documents/Centipede_MUJOCO-main/experiments/independent_short_erasure_armature_1e4_2026-08-18/strong_independent_erasure/models/farms/centipede_reliability_refined.xml',timestep_s=.0005,integrator='RK4',added_armature_kg_m2=.0001),current_profile=dict(path=str(ROOT.parent/'trial_profile.json'),sha256=sha(ROOT.parent/'trial_profile.json'),dt_s=.0005,solver='CPU PhysX TGS64/16',added_armature=0,joint_velocity_limit_rad_s=1e6),inference='Rapidly switching contact constraints can excite lightly damped low-inertia tarsal joints. High joint speeds and subsequent free-flight growth are consistent with articulation integration/constraint instability. Existing observations do not establish whether restoration impulse, USD update side effects, timestep, solver coupling or another implementation fault is the initiating cause.',causal_status='unresolved; engine, timestep, schedule and controller differ from historical MuJoCo. No one-factor engine comparison.',queue_status='Remaining random jobs paused for runtime-scope choice. Only two already-human-approved matched deterministic endpoint controls may run. Full4060 and rough extension held.')
(OUT/'DIVERGENCE_AUDIT.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
(OUT/'executed_analysis.py').write_bytes(Path(__file__).read_bytes())
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
fig,axs=plt.subplots(3,1,figsize=(8,6),sharex=True,constrained_layout=True)
axs[0].plot(tt,vv,color='#367aa7',marker='o',ms=3);axs[0].set_ylabel('Max link speed\n(m/s)')
axs[1].plot(tt,qv,color='#d8782e',marker='o',ms=3);axs[1].set_yscale('log');axs[1].set_ylim(8,2500);axs[1].set_ylabel('Max joint speed\n(rad/s)')
axs[2].plot(ft,force,color='#7b609e',lw=1.2);axs[2].set_ylabel('Contact force proxy\n(N)');axs[2].set_xlabel('Simulation time (s)')
for ax in axs:
    ax.set_xlim(2.80,2.902);ax.grid(alpha=.2);ax.axvspan(2.87,2.90,color='#b8cbd8',alpha=.17)
    for t in np.arange(2.8,2.901,.01):ax.axvline(t,color='#c6c6c6',lw=.45,alpha=.35)
axs[0].set_title('Existing failed trial: wave 3, p=0.5, 10 ms hold, mask seed 0')
axs[0].text(.99,.93,'At 2.900 s: body speed 6.09e14 m/s\njoint speed 1.03e17 rad/s; numeric divergence',transform=axs[0].transAxes,ha='right',va='top',fontsize=9)
axs[2].text(.98,.86,'No native ground-contact force in the final 30 ms',transform=axs[2].transAxes,ha='right',fontsize=9)
fig.savefig(OUT/'pilot_divergence_timeline.png',dpi=160);plt.close(fig)
print(json.dumps(report['stochastic_snapshot']))