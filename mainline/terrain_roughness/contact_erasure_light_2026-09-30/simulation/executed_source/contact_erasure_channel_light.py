"""Exogenous independent leg/temporal-bin collision erasure; native PhysX forces."""
from pathlib import Path
import json, hashlib, re, time
import numpy as np

DT=.0005
START=2.
END=6.
THRESHOLD_N=1e-8
def sha_bytes(a):return hashlib.sha256(a.tobytes()).hexdigest()
def dump(path,value):Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')

def schedule(p,seed,bin_s):
    assert 0<=p<=1 and bin_s in [.01,.02,.05]
    stride=round(bin_s/DT);assert abs(stride*DT-bin_s)<1e-12
    # A common time-indexed uniform bank matches all wave/p/hold-duration conditions.
    rng=np.random.default_rng(2026093001+int(seed))
    uniform=rng.random((round((END-START)/DT),38))
    erased=np.zeros((round(END/DT),38),bool)
    erased[round(START/DT):]=np.repeat(uniform[::stride]<p,stride,axis=0)
    return erased,dict(p=float(p),mask_seed=int(seed),bin_s=bin_s,bin_steps=stride,
        uniform_sha256=sha_bytes(uniform),mask_sha256=sha_bytes(erased),actual_erased_time_fraction=float(erased[round(START/DT):].mean()),
        actual_erased_fraction_per_leg=erased[round(START/DT):].mean(0).tolist(),
        nominal_cycle_s=1.,bins_per_cycle=round(1/bin_s),dt_s=DT,noise_window_s=[START,END],
        unit='38 individual complete leg subtrees; both sides independently drawn',
        temporal_law='Independent Bernoulli(p) per leg x bin. State held within bin; consecutive erased bins remain erased. New draws every cycle.',
        pairing='Same uniform bank per seed across all waves, p and hold durations; different durations have different masks.')

class Channel:
    def __init__(self,sim,robot,rows,plan,folder,report,body_names,order,foot_ids):
        from pxr import Usd,UsdPhysics,Sdf
        import omni.physx
        self.Usd,self.UsdPhysics,self.Sdf=Usd,UsdPhysics,Sdf
        self.flush=omni.physx.get_physx_simulation_interface().flush_changes
        self.sim,self.robot,self.folder,self.report=sim,robot,Path(folder),report
        self.n=len(rows);self.names=body_names;self.order=order;self.foot_ids=foot_ids
        self.cells=plan['joint_cells'];assert len(self.cells)==self.n
        self.masks=[];self.info=[]
        for cell in self.cells:
            mask,info=schedule(cell['p'],cell['mask_seed'],cell['erase_bin_s'])
            self.masks.append(mask);self.info.append(info)
        self.mask=np.stack(self.masks,axis=1);self.active=np.zeros((self.n,38),bool)
        self.attrs=[[[] for _ in range(38)] for _ in range(self.n)]
        self.groups=[];self.colliders=[];self.chassis_attrs=[]
        for leg in range(38):
            pair=leg//2;side='L' if leg%2==0 else 'R'
            members={f'link_leg_{pair}_{side}_{k}' for k in range(4)}|{f'foot_{pair}_{leg%2}'}
            ids=[i for i,name in enumerate(body_names) if name in members]
            assert len(ids)==5,(leg,ids);self.groups.append(ids)
        body_to_leg={body_names[i]:leg for leg,ids in enumerate(self.groups) for i in ids}
        assert len(body_to_leg)==190
        for e in range(self.n):
            counts=np.zeros(38,int);chassis=0
            for prim in Usd.PrimRange(sim.stage.GetPrimAtPath(f'/World/env_{e}/Robot'),Usd.TraverseInstanceProxies()):
                if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
                owner=prim
                while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
                assert owner and not prim.IsInstanceProxy()
                attr=UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr();assert attr.Get()
                name=owner.GetName()
                if name in body_to_leg:
                    leg=body_to_leg[name];self.attrs[e][leg].append(attr);counts[leg]+=1
                    self.colliders.append(dict(environment=e,leg=leg,owner=name,path=str(prim.GetPath())))
                else:self.chassis_attrs.append(attr);chassis+=1
            assert np.all(counts==6) and chassis==21,(counts,chassis)
        assert [body_names[i] for i in foot_ids]==[f"foot_{leg//2}_{leg%2}" for leg in range(38)]
        self.view=robot.root_physx_view
        self.plant_initial={key:getter().detach().cpu().numpy().copy() for key,getter in self.plant_getters().items()}
        self.plant_checks=[]
        self.times=[];self.feet=[];self.legs=[];self.chassis=[]
        self.transitions=[];self.restore_events=[];self.restore_samples=[]
        self.pending=np.zeros(self.n,int);self.restore_serial=np.zeros(self.n,int);self.current_restores=[]
        self.violation_count=0;self.max_erased_leg_force=np.zeros(self.n);self.elapsed_s=0.
        self.last_changed=np.zeros((self.n,38),bool)
        dump(self.folder/'ERASURE_BINDING.json',dict(channels=38,colliders_per_leg=6,leg_colliders=228,chassis_colliders=21,
            source_ids=[c['source_id'] for c in self.cells],legs=self.groups,collider_paths=self.colliders,
            foot_ids=foot_ids,foot_names=[body_names[i] for i in foot_ids],body_order=body_names,
            independent_environment_attributes=True,method='USD collisionEnabled on existing leg shapes; flush buffered PhysX changes BEFORE completed step. No body removal, pose projection, force or restore delay.'))
        dump(self.folder/'ERASURE_SCHEDULE.json',dict(cells=self.cells,schedules=self.info))
        np.savez_compressed(self.folder/'erasure_mask.npz',erased=self.mask,dt_s=DT,noise_start_s=START,end_s=END)

    def plant_getters(self):
        return dict(masses=self.view.get_masses,inertias=self.view.get_inertias,kp=self.view.get_dof_stiffnesses,kd=self.view.get_dof_dampings)

    def check_plant(self,step):
        for key,getter in self.plant_getters().items():
            np.testing.assert_array_equal(getter().detach().cpu().numpy(),self.plant_initial[key],err_msg=f"Erasure must not change native{key}")
        self.plant_checks.append(dict(after_step=step,t_s=(step+1)*DT,masses_inertias_and_full173_PD_exact=True))
        dump(self.folder/'ERASURE_FIXED_PLANT.json',dict(passed=True,checks=self.plant_checks))

    def before(self,step):
        tick=time.perf_counter();wanted=self.mask[step];changed=wanted!=self.active
        self.last_changed=changed
        if not changed.any():return
        restoring=self.active & ~wanted
        # Observe pre-restoration pose without modifying it.
        if restoring.any():
            pose=self.robot.data.body_link_pose_w.detach().cpu().numpy()
            for e in np.flatnonzero(restoring.any(1)):
                self.restore_serial[e]+=1;self.pending[e]=step+2
                legs=np.flatnonzero(restoring[e]).tolist()
                self.restore_events.append(dict(step=step,t_s=step*DT,environment=int(e),serial=int(self.restore_serial[e]),legs=legs,
                    foot_origin_z_m=pose[e,np.array(self.foot_ids)[legs],2].astype(float).tolist()))
        with self.Sdf.ChangeBlock():
            for e,leg in np.argwhere(changed):
                enabled=not wanted[e,leg]
                for attr in self.attrs[e][leg]:assert attr.Set(bool(enabled))
        self.flush()
        for e,leg in np.argwhere(changed):
            assert all(bool(a.Get())==(not wanted[e,leg]) for a in self.attrs[e][leg])
        assert all(a.Get() for a in self.chassis_attrs)
        self.active=wanted.copy()
        self.transitions.append(dict(step=step,t_s=step*DT,erased_legs=[np.flatnonzero(v).tolist() for v in wanted]))
        self.elapsed_s+=time.perf_counter()-tick

    def after(self,step):
        tick=time.perf_counter()
        if step in [4000,11999]:self.check_plant(step)
        matrix=self.report.get_contact_force_matrix(dt=DT).detach().cpu().numpy()[:,0]
        matrix=matrix[self.order].reshape(self.n,len(self.names),3)
        assert np.isfinite(matrix).all()
        feet=np.linalg.norm(matrix[:,self.foot_ids],axis=-1).astype(np.float32)
        legs=np.stack([np.linalg.norm(matrix[:,ids],axis=-1).sum(1) for ids in self.groups],axis=1).astype(np.float32)
        chassis=matrix.sum(1)-sum((matrix[:,ids].sum(1) for ids in self.groups))
        erased=np.where(self.active,legs,0.)
        self.max_erased_leg_force=np.maximum(self.max_erased_leg_force,erased.max(1))
        if (erased>1e-6).any():
            self.violation_count+=1
            dump(self.folder/'ERASURE_FAILURE.json',dict(reason='erased_leg_still_exerts_native_contact_force',step=step,t_s=(step+1)*DT,
                offender_source_ids=[self.cells[int(e)]['source_id'] for e in np.flatnonzero((erased>1e-6).any(1))],forces_N=erased.tolist()))
            raise RuntimeError('Erased leg native contact force: runtime toggle has not been validated')
        self.times.append((step+1)*DT);self.feet.append(feet);self.legs.append(legs);self.chassis.append(chassis.astype(np.float32))
        pending=np.flatnonzero(step<self.pending)
        if len(pending):
            force,points,normals,separation,count,start=self.report.get_contact_data(dt=DT)
            force=force.detach().cpu().numpy().ravel();separation=separation.detach().cpu().numpy().ravel()
            count=count.detach().cpu().numpy().ravel().astype(int);start=start.detach().cpu().numpy().ravel().astype(int)
            assert count.sum()<32768*self.n//6
            for e in pending:
                ids=self.order[e*len(self.names):(e+1)*len(self.names)]
                idx=np.concatenate([np.arange(start[b],start[b]+count[b]) for b in ids])
                self.restore_samples.append(dict(t_s=(step+1)*DT,step=step,environment=int(e),serial=int(self.restore_serial[e]),
                    max_native_depth_m=float(max(0.,-separation[idx].min())) if len(idx) else 0.,peak_normal_N=float(abs(force[idx]).max()) if len(idx) else 0.))
        self.elapsed_s+=time.perf_counter()-tick

    def finish(self,complete):
        np.savez_compressed(self.folder/'erasure_contacts.npz',t=self.times,foot_normal_force=self.feet,leg_normal_force=self.legs,
            chassis_normal_force=self.chassis,foot_names=[self.names[i] for i in self.foot_ids],force_presence_threshold_N=THRESHOLD_N)
        dump(self.folder/'ERASURE_RESULT.json',dict(complete=complete,schedules=self.info,transitions=self.transitions,
            restore_events=self.restore_events,restore_samples=self.restore_samples,deleted_contact_violations=self.violation_count,
            max_erased_leg_normal_force_N=self.max_erased_leg_force.tolist(),instrumentation_wall_s=self.elapsed_s,
            contact_sampling_dt_s=DT,mask_applied_before_step=True,restore_delayed=False,pose_projection=False,custom_forces=False,
            restoration_diagnostic='Native ALL robot contacts in the first1ms after restoration; not exclusive to restored toes. Foot origin z is a pose observation, not shape surface clearance.'))
