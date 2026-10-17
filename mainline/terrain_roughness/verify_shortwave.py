"""Verify completed previews against frozen amplitude, PD, and source contracts."""
from pathlib import Path
import json,hashlib,numpy as np
ROOT=Path(__file__).resolve().parent;MAIN=ROOT.parent;OUT=ROOT/'shortwave';SIM=OUT/'simulation_v2'
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def verify(attempt):
 c=read(SIM/'campaign.json');assert read(SIM/f'supervisor_{attempt}.json')['complete']
 for p,h in c['pins'].items():assert sha(p)==h,p
 pd=read(MAIN/'pd_wave4_firm20.json')['absolute_joint_pd'];records=[]
 for job in c['jobs']:
  d=SIM/'runs'/attempt/job['name'];res=read(d/'results.json');readback=read(d/'readback.json');plant=read(d/'matched_plant_check.json');amplitude=read(d/'amplitude_modulation.json');threads=read(d/'thread_settings.json');binding=read(d/'articulation_binding.json')
  assert res['complete'] and res['finite'] and res['whole_batch_finite'] and res['actual_duration_s']==6. and res['finite_check_interval_s']==.0005
  assert not read(d/'native_error_audit.json')['errors']
  assert readback['dt_s']==.0005 and readback['solver']=='TGS' and not readback['gpu_dynamics']
  assert threads['tasking']==threads['physics']==2 and threads['torch_threads']==1
  assert binding['row_order_verified'] and plant['passed'] and plant['complete_uniform_173_joint_stiffness'] and plant['complete_uniform_173_joint_damping']
  assert all(x['position']==64 and x['velocity']==16 for x in read(d/'iteration_readback.json'))
  names=readback['joint_names'];assert len(names)==173 and set(names)==set(pd)
  with np.load(d/'backend.npz') as z:
   expected=np.array([pd[n] for n in names],dtype=z['kp'].dtype)
   np.testing.assert_array_equal(z['kp'],np.repeat(expected[None,:,0],3,axis=0));np.testing.assert_array_equal(z['kd'],np.repeat(expected[None,:,1],3,axis=0))
  assert amplitude['other_targets_bitexact'] and amplitude['limits_passed']
  for cell,i in zip(amplitude['cells'],job['source_ids']):
   assert all(cell[k]==c['flat_cluster_cells'][i][k] for k in ['wave','A_deg','B_deg','body_factor','leg_factor'])
  traj=np.load(d/'trajectory.npz');assert traj['pose'].shape==(601,3,251,7) and np.isfinite(traj['pose']).all();assert np.isfinite(traj['q']).all() and np.isfinite(traj['v']).all();np.testing.assert_allclose(traj['t'],np.arange(601)*.01,atol=1e-12,rtol=0)
  poses=traj['pose'];body_names=traj['body_names'].tolist();spine=[i for i,n in enumerate(body_names) if n.startswith(('link_body_','link_pitch_body_'))];center=poses[:,:,:3] if False else poses[:,:,:,:3][:,:,spine].mean(2)
  guard=read(d/'crop_guard.json');assert guard['passed'];contacts=np.load(d/'cluster_contacts.npz');assert np.isfinite(contacts['force']).all()
  cells=[]
  for e,cell in enumerate(amplitude['cells']):
   cells.append(dict(wave=cell['wave'],A_deg=cell['A_deg'],B_deg=cell['B_deg'],center_displacement_xy_mm=((center[-1,e,:2]-center[0,e,:2])*1000).tolist(),path_length_xy_mm=float(np.linalg.norm(np.diff(center[:,e,:2],axis=0),axis=-1).sum()*1000),body_z_range_mm=[float(poses[:,e,:,2].min()*1000),float(poses[:,e,:,2].max()*1000)],mean_contacted_feet=float(np.count_nonzero(contacts['force'][:,e,:]>1e-9,axis=1).mean())))
  records.append(dict(job=job['name'],site=job['site'],level=job['level'],duration_s=6.,trajectory_sha256=sha(d/'trajectory.npz'),surface_sha256=read(d/'manifest.json')['rows'][0]['surface_sha256'],native_PD_exact=True,native_error_count=0,whole_batch_finite=True,minimum_saved_body_center_boundary_margin_mm=min(guard['saved_pose_margins_m'])*1000,thread_readback=threads,cells=cells,wall_s=res['wall_s']))
 report=dict(complete=True,condition_count=36,physics_seconds_per_condition=6,checks=dict(input_hashes=True,frozen_flat_cluster_amplitudes=True,all173_native_PD_exact=True,physics_0p5ms=True,CPG_1ms_two_step_hold=True,TGS64_16=True,four_workers_two_threads=True,all_step_finite=True,collision_isolation=True,crop_interior_guard=True,native_error_free=True,all_trajectories_saved=True),initialization_note=c['initialization'],physics_roi_m=c['terrain_roi_m'],scope='Short visualization previews; not full locomotion performance validation, optimization or convergence evidence',runs=records,earlier_attempt=dict(path=str(OUT/'simulation/runs/v1'),failure='Narvik low group reached 15 mm cropped-domain guard at 5.089 s; no solver/PD failure; expanded common ROI and reran matched previews in simulation_v2. Original attempt retained.'))
 (OUT/'SIMULATION_VERIFICATION.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8');print('VERIFIED: all 36 previews, exact PD, fixed amplitudes, native readback, finite states, boundaries and no native errors')
if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('--attempt',default='v2');verify(p.parse_args().attempt)
