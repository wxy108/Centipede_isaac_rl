def worker(name,attempt,keep=None):
    c=read(SIM/'campaign.json');j=next(j for j in c['jobs'] if j['name']==name)
    for p,h in c['pins'].items():assert sha(p)==h,p
    args=argparse.Namespace(**j['args']);rows=read(args.rows);plan=read(args.experiment_plan)
    selected=j['source_ids'] if keep is None else keep;assert selected and set(selected)<=set(j['source_ids'])
    D=SIM/'runs'/attempt/name;D.mkdir(parents=True,exist_ok=False)
    if keep is not None:
        positions=[j['source_ids'].index(i) for i in selected];n=len(selected)
        plan.update(benchmark_source_ids=selected,selected_environments=selected,body_factors=[plan['body_factors'][i] for i in positions],
            leg_cluster_factors=[plan['leg_cluster_factors'][i] for i in positions],joint_cells=[plan['joint_cells'][i] for i in positions],
            quality_offsets_m=[[0.,0.,0.]]*n,episode1_permutation=list(range(n)),peer_recovery=True,original_case_slots=positions)
        plan['scaling_benchmark']['replicas']=n
        dump(D/'effective_plan.json',plan);args.experiment_plan=str(D/'effective_plan.json');args.experiment_plan_sha256=sha(args.experiment_plan)
    log=(D/'worker.log').open('w',encoding='utf-8');sys.stdout.flush();sys.stderr.flush();os.dup2(log.fileno(),sys.stdout.fileno());os.dup2(log.fileno(),sys.stderr.fileno())
    active=dict(pid=os.getpid(),job=name,status='starting',source_ids=selected,started_unix=time.time());dump(D/'active.json',active);exitcode=1
    try:
        os.environ['WARP_CACHE_PATH']=str(D/'warp_cache');sys.path.insert(0,str(engine.CON/'open_loop_review_2026-09-16'));sys.path.insert(0,str(engine.NI/'work/runtime_speed_2026-09-17/deps'))
        import warp as wp
        assert wp.__version__=='1.8.1';wp.init()
        from isaaclab.app import AppLauncher
        launcher=AppLauncher(args)
        import torch
        torch.set_num_threads(1);torch.set_num_interop_threads(1)
        sys.path.insert(0,str(engine.CON/'terrain_surface_library_2026-09-15'));sys.path.insert(0,str(MAIN))
        from unified_trial import simulation
        patch=SIM/'executed_source/adapted_functions.py';exec(compile(patch.read_text(encoding='utf-8'),str(patch),'exec'),simulation.__dict__)
        from types import SimpleNamespace
        runtime=SimpleNamespace(args=args,experiment=plan,rows=rows,input_receipt=dict(input_rows_sha256=j['rows_sha256'],profile_sha256=c['profile_sha256'],fixed_pd_sha256=c['fixed_pd_sha256'],full_table_exact=True,flat_cluster_amplitudes_unchanged=True),run_directory=D,execution_sources=dict(c['pins'],**{str(args.experiment_plan):sha(args.experiment_plan)}))
        active['status']='running';dump(D/'active.json',active);simulation.run_trial(runtime)
        result=read(D/'results.json');assert result['complete'] and result['whole_batch_finite'] and result['actual_duration_s']==6.
        assert not read(D/'native_error_audit.json')['errors']
        assert read(D/'collision_guard.json')['minimum_all_step_collision_margin_m']>.015
        active['status']='finished';exitcode=0
    except BaseException:active['status']='failed';traceback.print_exc()
    finally:active.update(exit_code=exitcode,finished_unix=time.time());dump(D/'active.json',active);sys.stdout.flush();sys.stderr.flush();log.flush()
    os._exit(exitcode)
