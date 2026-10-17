# Narrow terrain-preview adaptations; controller, gains, solver and stepping preserved.

def _create_environments(runtime, sim, rows, source_ids):
    """Create each private terrain and its matched starting pose."""
    experiment = runtime.experiment
    run_directory = runtime.run_directory
    environment_count = len(rows)
    assert environment_count == experiment["scaling_benchmark"]["replicas"]
    columns = 6
    grid_rows = (environment_count + columns - 1) // columns
    pitch = 1.2
    offsets = np.array(experiment["quality_offsets_m"])
    assert offsets.shape == (environment_count, 3)
    common.write(
        run_directory / "layout.json",
        dict(
            layout="All environment transforms at world origin; robot and own terrain collision-isolated",
            pitch_m=pitch,
            columns=columns,
            rows=grid_rows,
            occupied_slots=source_ids,
            offsets_m=offsets.tolist(),
            parent_native_runner_sha256="cb8d70b52c53e82dbc08ca8fade66105db83375ae67fdbf862e2d286c2f8f4a8",
        ),
    )
    material = UsdShade.Material.Define(sim.stage, "/World/Material")
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(0.5)
    physics_material.CreateDynamicFrictionAttr(0.5)
    physics_material.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(
        material.GetPrim()
    ).CreateFrictionCombineModeAttr("max")
    initial_poses = []
    for environment_index, row in enumerate(rows):
        environment = UsdGeom.Xform.Define(sim.stage, f"/World/env_{environment_index}")
        environment.AddTranslateOp().Set(Gf.Vec3d(*offsets[environment_index]))
        path = f"/World/env_{environment_index}/Ground"
        if row["surface_file"]:
            assert common.sha(row["surface_file"]) == row["surface_sha256"]
            surface_data = np.load(row["surface_file"])
            height = surface_data["height"]
            spacing = float(surface_data["dx_m"])
            full_height = height
            ny_full, nx_full = height.shape
            ax_full = (np.arange(nx_full) - (nx_full - 1) / 2) * spacing
            ay_full = (np.arange(ny_full) - (ny_full - 1) / 2) * spacing
            bx0, bx1, by0, by1 = experiment["terrain_roi_m"]
            ix = np.flatnonzero((ax_full >= bx0-1e-10) & (ax_full <= bx1+1e-10))
            iy = np.flatnonzero((ay_full >= by0-1e-10) & (ay_full <= by1+1e-10))
            np.testing.assert_allclose([ax_full[ix[0]], ax_full[ix[-1]], ay_full[iy[0]], ay_full[iy[-1]]], experiment["terrain_roi_m"], atol=1e-12, rtol=0)
            height = full_height[np.ix_(iy, ix)]
            height_rows, height_columns = height.shape
            grid_x, grid_y = np.meshgrid(
                ax_full[ix],
                ay_full[iy],
            )
            vertices = np.c_[grid_x.ravel(), grid_y.ravel(), height.ravel()].astype(
                np.float32
            )
            row_indices, column_indices = np.meshgrid(
                np.arange(height_rows - 1), np.arange(height_columns - 1), indexing="ij"
            )
            cell_start = (row_indices * height_columns + column_indices).ravel()
            faces = np.vstack(
                [
                    np.c_[cell_start, cell_start + 1, cell_start + height_columns + 1],
                    np.c_[
                        cell_start,
                        cell_start + height_columns + 1,
                        cell_start + height_columns,
                    ],
                ]
            ).astype(np.int32)
            mesh = UsdGeom.Mesh.Define(sim.stage, path)
            mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(vertices))
            mesh.CreateFaceVertexCountsAttr(
                Vt.IntArray.FromNumpy(np.full(len(faces), 3, np.int32))
            )
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.ravel()))
            mesh.CreateSubdivisionSchemeAttr("none")
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
                "none"
            )
            ground_prim = mesh.GetPrim()
        else:
            height = None
            spacing = None
            vertices = np.array(
                [
                    [-0.5, -0.5, 0.0],
                    [0.5, -0.5, 0.0],
                    [0.5, 0.5, 0.0],
                    [-0.5, 0.5, 0.0],
                ],
                np.float32,
            )
            faces = np.array([[0, 1, 2], [0, 2, 3]], np.int32)
            assert np.all(
                np.cross(
                    vertices[faces[:, 1]] - vertices[faces[:, 0]],
                    vertices[faces[:, 2]] - vertices[faces[:, 0]],
                )[:, 2]
                > 0
            )
            mesh = UsdGeom.Mesh.Define(sim.stage, path)
            mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(vertices))
            mesh.CreateFaceVertexCountsAttr(
                Vt.IntArray.FromNumpy(np.full(2, 3, np.int32))
            )
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.ravel()))
            mesh.CreateSubdivisionSchemeAttr("none")
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
                "none"
            )
            ground_prim = mesh.GetPrim()
        UsdPhysics.CollisionAPI.Apply(ground_prim).CreateCollisionEnabledAttr(True)
        collision_api = PhysxSchema.PhysxCollisionAPI.Apply(ground_prim)
        collision_api.CreateContactOffsetAttr(2e-05)
        collision_api.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(ground_prim).Bind(
            material, materialPurpose="physics"
        )
        pose = common.initial(row, full_height if height is not None else None, spacing)
        pose[:3] += offsets[environment_index]
        initial_poses.append(pose)
    return (offsets, initial_poses, material)


def _prepare_plant(runtime, rows, source_ids):
    """Bind environments before writing row-specific gains; verify the native plant."""
    args = runtime.args
    experiment = runtime.experiment
    run_directory = runtime.run_directory
    environment_count = len(rows)
    (
        sim,
        scene,
        scene_api,
        scene_switch,
        friction_correlation_distance,
        friction_offset_threshold,
    ) = _create_simulation(runtime)
    offsets, initial_poses, material = _create_environments(
        runtime, sim, rows, source_ids
    )
    robot = _spawn_robot(args)
    counts = _configure_collisions(
        runtime, sim, scene, scene_api, material, environment_count
    )
    sim.reset()
    joint_names = list(robot.joint_names)
    body_names = list(robot.body_names)
    assert len(joint_names) == 173 and counts == [249] * environment_count
    native_roots = list(robot.root_physx_view.prim_paths)
    root_order_ok = len(native_roots) == environment_count and all(
        (
            str(path).split("/")[2] == f"env_{e}" and str(path).split("/")[3] == "Robot"
            for e, path in enumerate(native_roots)
        )
    )
    common.write(
        run_directory / "articulation_binding.json",
        dict(
            native_root_paths=native_roots,
            expected_environment_indices=list(range(environment_count)),
            row_order_verified=root_order_ok,
            checked_before_explicit_gain_and_initial_state_writes=True,
        ),
    )
    assert (
        root_order_ok
    ), "Native articulation row order differs from planned environment order; stop before assigning states or gains"
    scene_receipt = scene_diagnostics.readback(sim, scene_switch)
    common.write(run_directory / "scene_diagnostics_readback.json", scene_receipt)
    None
    assert scene.GetAttribute("physxScene:enableGPUDynamics").Get() is (
        args.device == "cuda:0"
    )
    assert scene.GetAttribute("physxScene:solverType").Get() == "TGS"
    # Complete fixed PD is checked by name before every native write; no fallback.
    fixed = json.loads(Path(runtime.experiment["profile_path"]).read_text())
    table = json.loads((Path(__file__).resolve().parents[1] / "pd_wave4_firm20.json").read_text())["absolute_joint_pd"]
    assert set(joint_names) == set(table) and len(joint_names) == 173
    assert all(r["absolute_joint_pd"] == table and r["family_multipliers"] == {} for r in rows)
    stiffness, damping = common.physical_gains(joint_names, rows)
    robot.write_joint_stiffness_to_sim(torch.tensor(stiffness, device=sim.device))
    robot.write_joint_damping_to_sim(torch.tensor(damping, device=sim.device))
    robot.actuators["all"].stiffness[:] = torch.tensor(stiffness, device=sim.device)
    robot.actuators["all"].damping[:] = torch.tensor(damping, device=sim.device)
    initial_state = torch.tensor(
        np.array(initial_poses), dtype=torch.float32, device=sim.device
    )
    robot.write_root_pose_to_sim(initial_state)
    robot.write_root_velocity_to_sim(
        torch.zeros((environment_count, 6), device=sim.device)
    )
    robot.write_joint_state_to_sim(
        torch.zeros((environment_count, 173), device=sim.device),
        torch.zeros((environment_count, 173), device=sim.device),
    )
    robot.reset()
    sim.forward()
    robot.update(0.0)
    articulation_view = robot.root_physx_view
    np.testing.assert_allclose(
        as_numpy(articulation_view.get_dof_stiffnesses()), stiffness, rtol=0, atol=0
    )
    np.testing.assert_allclose(
        as_numpy(articulation_view.get_dof_dampings()), damping, rtol=0, atol=0
    )
    np.savez_compressed(
        run_directory / "backend.npz",
        masses=as_numpy(articulation_view.get_masses()),
        inertias=as_numpy(articulation_view.get_inertias()),
        kp=stiffness,
        kd=damping,
    )
    reference_ids = [row["baseline_source_id"] for row in rows]
    with np.load(experiment["baseline_backend"]["path"]) as original:
        np.testing.assert_array_equal(
            as_numpy(articulation_view.get_masses()), original["masses"][reference_ids]
        )
        np.testing.assert_array_equal(
            as_numpy(articulation_view.get_inertias()), original["inertias"][reference_ids]
        )
        expected_stiffness = original["kp"].copy()
        expected_damping = original["kd"].copy()
        for e, override in experiment["pd_overrides"].items():
            for name, pd in override.items():
                expected_stiffness[int(e), joint_names.index(name)] = pd[0]
                expected_damping[int(e), joint_names.index(name)] = pd[1]
        np.testing.assert_array_equal(stiffness, expected_stiffness[reference_ids])
        np.testing.assert_array_equal(damping, expected_damping[reference_ids])
    # New terrain placement is a declared generated expectation, not historical readback.
    expected_initial = np.array(experiment["planned_initial_pose"])[source_ids].copy()
    expected_initial[:, :3] += offsets
    np.testing.assert_allclose(
        np.array(initial_poses), expected_initial, rtol=0, atol=1e-12
    )
    common.write(
        run_directory / "matched_plant_check.json",
        dict(
            passed=True,
            masses_and_inertias_exact=True,
            complete_uniform_173_joint_stiffness=True,
            initial_pose_local_matched=True,
            layout_exact=False,
            declared_replication_layout=True,
            complete_uniform_173_joint_damping=True,
            robot_collision_enable_and_geometry_unchanged=True,
            terrain_changed_as_declared=True,
            contact_offset_m=experiment["numerical_change"][
                "contact_offset_all_colliders_m"
            ],
            world_pitch_feedback_and_declared_gain_taper=True,
        ),
    )
    common.write(
        run_directory / "readback.json",
        dict(
            scene_diagnostics=scene_receipt,
            contact_force_components="Native normal-force vector sum; tangential friction excluded",
            gpu_dynamics=scene.GetAttribute("physxScene:enableGPUDynamics").Get(),
            solver=scene.GetAttribute("physxScene:solverType").Get(),
            dt_s=sim.get_physics_dt(),
            contact="native PhysX",
            device=sim.device,
            colliders=counts,
            initial_pose=np.array(initial_poses).tolist(),
            environment_offsets=offsets.tolist(),
            friction=0.5,
            body_names=body_names,
            joint_names=joint_names,
        ),
    )
    roots = [
        prim
        for prim in sim.stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    root_read = [
        dict(
            path=str(prim.GetPath()),
            position=prim.GetAttribute(
                "physxArticulation:solverPositionIterationCount"
            ).Get(),
            velocity=prim.GetAttribute(
                "physxArticulation:solverVelocityIterationCount"
            ).Get(),
        )
        for prim in roots
    ]
    assert len(root_read) == environment_count and all(
        (
            r["position"] == args.position_iterations
            and r["velocity"] == args.velocity_iterations
            for r in root_read
        )
    ), root_read
    common.write(run_directory / "iteration_readback.json", root_read)
    scene_limits = {
        name: scene.GetAttribute("physxScene:" + name + "IterationCount").Get()
        for name in ["minPosition", "maxPosition", "minVelocity", "maxVelocity"]
    }
    assert (
        scene_limits["minPosition"]
        <= args.position_iterations
        <= scene_limits["maxPosition"]
    )
    assert (
        scene_limits["minVelocity"]
        <= args.velocity_iterations
        <= scene_limits["maxVelocity"]
    )
    common.write(
        run_directory / "iteration_scene_limits.json",
        dict(
            values=scene_limits,
            scope="USD post-reset limits and articulation requests; not direct native getter",
        ),
    )
    field_readback = []
    for prim in Usd.PrimRange(sim.stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if "/env_" in path and (not path.startswith("/World/env_0/")):
            continue
        for attr in prim.GetAttributes():
            if attr.GetName().startswith(("physics:", "physx", "drive:")):
                value = attr.Get()
                field_readback.append(
                    dict(
                        path=path,
                        name=attr.GetName(),
                        value=str(value),
                        type=str(attr.GetTypeName()),
                        authored=attr.HasAuthoredValueOpinion(),
                    )
                )
    np.testing.assert_allclose(
        scene_api.GetFrictionCorrelationDistanceAttr().Get(),
        friction_correlation_distance,
        rtol=0,
        atol=1e-08,
    )
    np.testing.assert_allclose(
        scene_api.GetFrictionOffsetThresholdAttr().Get(),
        friction_offset_threshold,
        rtol=0,
        atol=1e-08,
    )
    common.write(
        run_directory / "friction_length_effective.json",
        dict(
            correlation_m=scene_api.GetFrictionCorrelationDistanceAttr().Get(),
            offset_threshold_m=scene_api.GetFrictionOffsetThresholdAttr().Get(),
            scope="USD after reset, not native getter proof",
        ),
    )
    common.write(
        run_directory / "effective_fields_after_pd.json",
        dict(
            scope="USD after reset and tensor gain writes; NOT proof of native solver values",
            attributes=field_readback,
            stage_meters_per_unit=UsdGeom.GetStageMetersPerUnit(sim.stage),
            stage_kilograms_per_unit=UsdPhysics.GetStageKilogramsPerUnit(sim.stage),
        ),
    )
    common.write(
        run_directory / "native_view_capabilities.json",
        dict(
            articulation=[x for x in dir(articulation_view) if x.startswith("get_")],
            body_order=body_names,
            native_link_paths=articulation_view.link_paths,
        ),
    )
    return (
        sim,
        robot,
        joint_names,
        body_names,
        initial_state,
        offsets,
        articulation_view,
    )


def _load_commands(runtime, joint_names, source_ids):
    """Load original float32 commands and verify the captured baseline prefix."""
    experiment = runtime.experiment
    run_directory = runtime.run_directory
    contract_rows = runtime.rows
    unique_ids = list(dict.fromkeys(source_ids))
    unique_rows = [contract_rows[i] for i in unique_ids]
    replica_map = [unique_ids.index(i) for i in source_ids]
    cache = CommandCache(joint_names, 0.001, unique_rows)
    common.write(
        run_directory / "command_audit.json",
        dict(unique_source_ids=unique_ids, replica_map=replica_map, audit=cache.audit),
    )
    original_commands = json.loads(
        Path(experiment["baseline_commands"]["path"]).read_text()
    )
    for e, old in enumerate([original_commands[contract_rows[i]["baseline_source_id"]] for i in unique_ids]):
        assert (
            hashlib.sha256(cache.position[:8304, e].tobytes()).hexdigest()
            == old["position_sha256"]
        )
        assert (
            hashlib.sha256(cache.velocity[:8304, e].tobytes()).hexdigest()
            == old["velocity_sha256"]
        )
    assert len(cache.position) >= 12001
    return (cache, replica_map)


def run_trial(runtime):
    """Run one declared 6 s terrain preview; retain original numerical/controller code."""
    args = runtime.args
    experiment = runtime.experiment
    input_contract_receipt = runtime.input_receipt
    contract_rows = runtime.rows
    run_directory = runtime.run_directory
    rows = json.loads(Path(args.rows).read_text())
    environment_count = len(rows)
    started = time.monotonic()
    assert (
        common.sha(args.rows) == input_contract_receipt["input_rows_sha256"]
        and rows == contract_rows
    )
    source_ids = experiment["benchmark_source_ids"]
    rows = [rows[i] for i in source_ids]
    environment_count = len(rows)
    controlled_environments = list(range(environment_count))
    assert all((i in experiment["selected_environments"] for i in source_ids))
    _write_provenance(runtime, rows, source_ids)
    sim, robot, joint_names, body_names, initial_state, offsets, articulation_view = (
        _prepare_plant(runtime, rows, source_ids)
    )
    timing = {"commands": 0.0, "physics": 0.0, "contact": 0.0, "recording": 0.0}
    cache, replica_map = _load_commands(runtime, joint_names, source_ids)
    control_environment_ids = torch.tensor(controlled_environments, device=sim.device)
    control_joint_ids = torch.tensor(
        [joint_names.index(n) for n in experiment["controlled_joints"]],
        device=sim.device,
    )
    control_body_ids = torch.tensor(
        [body_names.index(n) for n in experiment["controlled_bodies"]],
        device=sim.device,
    )
    joint_metadata = {j["name"]: j for j in common.M["joints"]}
    control_joint_limits = torch.tensor(
        [joint_metadata[n]["range_rad"] for n in experiment["controlled_joints"]],
        device=sim.device,
    )[None]
    controller_samples = []
    controller_times = []
    common.write(
        run_directory / "instrumentation.json",
        dict(
            contact_reports=True,
            detailed_force_recording=experiment["diagnostic_telemetry"],
            pose_sample_dt_s=experiment["recording_dt_s"],
            world_feedback_sample_dt_s=experiment["recording_dt_s"],
            collisions_enabled=True,
            numerical_contact_offset_m=experiment["numerical_change"][
                "contact_offset_all_colliders_m"
            ],
        ),
    )
    # Commands, feedback, contacts, and saved state retain their original cadences.
    numerical_failure = None
    command_positions, command_velocities = _scale_commands(
        runtime, sim, cache, replica_map, joint_names, environment_count
    )
    frame_times = []
    body_poses = []
    joint_positions = []
    joint_velocities = []
    body_velocities = []
    frame_episodes = []
    episode = 0
    reset_events = []
    recording_stride = round(experiment["recording_dt_s"] / args.dt)
    command_stride = round(0.001 / args.dt)
    assert abs(command_stride * args.dt - 0.001) < 1e-12

    def save_frame(t):
        """Save copied state in each environment's local coordinates."""
        frame_episodes.append(episode)
        local_body_pose = as_numpy(robot.data.body_link_pose_w)
        local_body_pose[:, :, :3] -= offsets[:, None, :]
        frame_times.append(t)
        body_poses.append(local_body_pose)
        joint_positions.append(as_numpy(robot.data.joint_pos))
        joint_velocities.append(as_numpy(robot.data.joint_vel))
        body_velocities.append(as_numpy(robot.data.body_link_vel_w))

    body_paths = [
        p for env in articulation_view.link_paths[:environment_count] for p in env
    ]
    ground_filters = [
        [f"/World/env_{e}/Ground"] for e in range(environment_count) for _ in body_names
    ]
    rigid_body_view = robot._physics_sim_view.create_rigid_body_view(body_paths)
    contact_report = robot._physics_sim_view.create_rigid_contact_view(
        body_paths,
        filter_patterns=ground_filters,
        max_contact_data_count=32768 * environment_count // 6,
    )
    assert (
        rigid_body_view.count == environment_count * len(body_names)
        and contact_report.sensor_count == rigid_body_view.count
        and (contact_report.filter_count == 1)
    )
    assert list(contact_report.sensor_paths) == list(rigid_body_view.prim_paths)
    contact_body_order = np.array(
        [list(rigid_body_view.prim_paths).index(p) for p in body_paths]
    )
    foot_ids = [i for i, name in enumerate(body_names) if name.startswith("foot_")]
    assert len(foot_ids) == 38
    head_id = body_names.index("link_body_0")
    contact_t = []
    contact_net = []
    contact_feet = []
    contact_head = []
    contact_minsep = []
    contact_n = []
    contact_maxforce = []
    common.write(
        run_directory / "quality_contact_binding.json",
        dict(
            focal_paths=body_paths,
            report_paths=list(rigid_body_view.prim_paths),
            order=contact_body_order.tolist(),
            filters=ground_filters,
            buffer_capacity=32768 * environment_count // 6,
            contact_sample_dt_s=args.dt,
            definition="Native sensor/own-ground contact separation and normal force immediately after completed physics step; finite buffer audited",
            source_case_ids=source_ids[:environment_count],
        ),
    )
    friction_net = []
    friction_counts = []
    contact_details = []

    def sample_contacts(t):
        """Record detailed own-ground contact data when diagnostic telemetry is enabled."""
        forces, points, normals, sep, counts, starts = contact_report.get_contact_data(
            dt=args.dt
        )
        counts = as_numpy(counts).ravel().astype(int)
        starts = as_numpy(starts).ravel().astype(int)
        sep = as_numpy(sep).ravel()
        forces = as_numpy(forces).ravel()
        normals = as_numpy(normals)
        assert (
            counts.sum() < 32768 * environment_count // 6
            and np.max(starts + counts) <= 32768 * environment_count // 6
        ), "Detailed contact buffer full"
        matrix = as_numpy(contact_report.get_contact_force_matrix(dt=args.dt))[:, 0]
        net = as_numpy(contact_report.get_net_contact_forces(dt=args.dt))
        assert np.max(abs(matrix - net)) < max(
            1e-06, np.max(abs(net)) * 0.0001
        ), "Contact to unexpected surface or binding mismatch"
        reconstructed = np.zeros_like(matrix)
        depth = np.zeros(len(counts))
        peak = np.zeros(len(counts))
        for j in np.flatnonzero(counts):
            contact_slice = slice(starts[j], starts[j] + counts[j])
            depth[j] = min(0, float(sep[contact_slice].min()))
            peak[j] = float(abs(forces[contact_slice]).max())
            reconstructed[j] = (
                forces[contact_slice, None] * normals[contact_slice]
            ).sum(0)
        last_contact_audit = dict(
            t=t,
            count=int(counts.sum()),
            reconstruction_error=float(np.max(abs(reconstructed - matrix))),
            net_filter_gap=float(np.max(abs(matrix - net))),
        )
        if abs(t / 0.1 - round(t / 0.1)) < 1e-07:
            common.write(run_directory / "contact_last_audit.json", last_contact_audit)
        assert np.max(abs(reconstructed - matrix)) < max(
            1e-06, np.max(abs(matrix)) * 0.0001
        ), "Contact detail force reconstruction mismatch"
        contact_points = as_numpy(points)
        for j in np.flatnonzero(counts):
            focal = int(np.flatnonzero(contact_body_order == j)[0])
            contact_details.extend(
                [
                    [
                        t - args.dt,
                        focal,
                        float(sep[z]),
                        *contact_points[z].tolist(),
                        *normals[z].tolist(),
                    ]
                    for z in range(starts[j], starts[j] + counts[j])
                ]
            )
        friction_forces, friction_points, friction_counts_native, friction_starts = (
            contact_report.get_friction_data(dt=args.dt)
        )
        friction_forces = as_numpy(friction_forces)
        friction_counts_native = as_numpy(friction_counts_native).ravel().astype(int)
        friction_starts = as_numpy(friction_starts).ravel().astype(int)
        assert (
            friction_counts_native.sum() < 32768 * environment_count // 6
            and np.max(friction_starts + friction_counts_native)
            <= 32768 * environment_count // 6
        ), "Friction buffer full"
        friction = np.zeros((len(friction_counts_native), 3))
        for j in np.flatnonzero(friction_counts_native):
            friction[j] = friction_forces[
                friction_starts[j] : friction_starts[j] + friction_counts_native[j]
            ].sum(0)
        assert np.isfinite(friction).all()
        friction_net.append(
            friction[contact_body_order]
            .reshape(environment_count, len(body_names), 3)
            .sum(1)
        )
        friction_counts.append(
            friction_counts_native[contact_body_order]
            .reshape(environment_count, len(body_names))
            .sum(1)
        )
        matrix = matrix[contact_body_order].reshape(
            environment_count, len(body_names), 3
        )
        depth = depth[contact_body_order].reshape(environment_count, len(body_names))
        peak = peak[contact_body_order].reshape(environment_count, len(body_names))
        counts = counts[contact_body_order].reshape(environment_count, len(body_names))
        contact_t.append(t)
        contact_net.append(matrix.sum(1))
        contact_feet.append(matrix[:, foot_ids])
        contact_head.append(matrix[:, head_id])
        contact_minsep.append(depth.min(1))
        contact_n.append(counts.sum(1))
        contact_maxforce.append(peak.max(1))

    momentum_t = []
    momentum_before = []
    momentum_after = []
    pre_contact_pose = []
    native_offsets = as_numpy(articulation_view.get_contact_offsets())
    native_rests = as_numpy(articulation_view.get_rest_offsets())
    expected_offset = experiment["numerical_change"]["contact_offset_all_colliders_m"]
    np.testing.assert_allclose(native_offsets, expected_offset, rtol=0, atol=1e-10)
    np.testing.assert_allclose(native_rests, 0.0, rtol=0, atol=1e-12)
    common.write(
        run_directory / "native_contact_offsets.json",
        dict(
            contact=native_offsets.tolist(),
            rest=native_rests.tolist(),
            scope="Native robot articulation shapes; own ground separately verified by USD",
        ),
    )
    native_mass = as_numpy(articulation_view.get_masses())
    common.write(
        run_directory / "contact_time_convention.json",
        dict(
            native_separation="empirically pre-step in analytic sphere fixture; verify for robot",
            force="impulse over completed step divided by dt",
            momentum="world COM linear velocities times native masses",
            limitation="Normal plus separately stored friction force used; articulated momentum closure remains unresolved",
            pose_storage="pre-step link poses every physics step",
            contact_processing_disabled=sim._settings.get(
                "/physics/disableContactProcessing"
            ),
        ),
    )
    common.write(
        run_directory / "telemetry_mode.json",
        dict(
            diagnostic_telemetry=experiment["diagnostic_telemetry"],
            all_step_finite=True,
            trajectory_stride_s=experiment["recording_dt_s"],
            physics_contact_processing_unchanged=True,
            scope="Disable diagnostic contact/momentum sampling and chunk writes only; retain report allocation, all-step state checks and stored trajectories",
        ),
    )
    telemetry_chunks = []
    chunk_io_s = 0.0

    def flush_telemetry():
        """Persist and clear only completed diagnostic chunks."""
        nonlocal chunk_io_s
        if not contact_t:
            return
        io_start = time.perf_counter()
        chunk = run_directory / "telemetry_chunks" / f"{len(telemetry_chunks):05d}"
        chunk.mkdir(parents=True, exist_ok=False)
        np.savez_compressed(
            chunk / "quality_contacts.npz",
            t=contact_t,
            net_normal_force=contact_net,
            net_friction_force=friction_net,
            friction_count=friction_counts,
            foot_normal_force=contact_feet,
            head_normal_force=contact_head,
            min_native_separation_m=contact_minsep,
            contact_count=contact_n,
            max_point_normal_force=contact_maxforce,
        )
        np.savez_compressed(
            chunk / "momentum.npz",
            pre_step_t=momentum_t,
            before=momentum_before,
            after=momentum_after,
            masses=native_mass,
            pre_step_link_pose=pre_contact_pose,
        )
        np.savez_compressed(
            chunk / "contact_points.npz",
            data=contact_details,
            fields=[
                "pre_step_t",
                "focal_body_index",
                "separation",
                "point_x",
                "point_y",
                "point_z",
                "normal_x",
                "normal_y",
                "normal_z",
            ],
        )
        telemetry_chunks.append(
            dict(
                directory=str(chunk),
                steps=len(contact_t),
                first_pre_step_t=momentum_t[0],
                last_post_step_t=contact_t[-1],
            )
        )
        contact_t.clear()
        contact_net.clear()
        friction_net.clear()
        friction_counts.clear()
        contact_feet.clear()
        contact_head.clear()
        contact_minsep.clear()
        contact_n.clear()
        contact_maxforce.clear()
        momentum_t.clear()
        momentum_before.clear()
        momentum_after.clear()
        pre_contact_pose.clear()
        contact_details.clear()
        chunk_io_s += time.perf_counter() - io_start
        common.write(
            run_directory / "telemetry_chunks.json",
            dict(
                chunks=telemetry_chunks,
                chunk_io_s=chunk_io_s,
                complete=False,
                schema="diagnostic telemetry chunks when enabled; intentionally empty when disabled",
            ),
        )

    cluster_times = []
    cluster_forces = []
    cluster_points = []
    cluster_counts = []

    def sample_cluster(t):
        """Store force-weighted foot contact points at the trajectory cadence."""
        force, point, normal, separation, count, start = (
            contact_report.get_contact_data(dt=args.dt)
        )
        force = as_numpy(force).ravel()
        point = as_numpy(point)
        count = as_numpy(count).ravel().astype(int)
        start = as_numpy(start).ravel().astype(int)
        assert (
            count.sum() < 32768 * environment_count // 6
            and np.max(start + count) <= 32768 * environment_count // 6
        )
        contact_force_matrix = as_numpy(
            contact_report.get_contact_force_matrix(dt=args.dt)
        )[:, 0]
        assert np.isfinite(contact_force_matrix).all()
        foot_contact_points = np.full((environment_count, len(foot_ids), 3), np.nan)
        foot_contact_forces = np.zeros((environment_count, len(foot_ids)))
        foot_contact_counts = np.zeros((environment_count, len(foot_ids)), int)
        for environment_index in range(environment_count):
            for foot_index, body_index in enumerate(foot_ids):
                native = contact_body_order[
                    environment_index * len(body_names) + body_index
                ]
                foot_contact_counts[environment_index, foot_index] = count[native]
                if count[native]:
                    contact_slice = slice(start[native], start[native] + count[native])
                    weights = abs(force[contact_slice])
                    foot_contact_forces[environment_index, foot_index] = weights.sum()
                    if foot_contact_forces[environment_index, foot_index] > 0:
                        foot_contact_points[environment_index, foot_index] = (
                            np.average(point[contact_slice], axis=0, weights=weights)
                            - offsets[environment_index]
                        )
        cluster_times.append(t)
        cluster_forces.append(foot_contact_forces)
        cluster_points.append(foot_contact_points)
        cluster_counts.append(foot_contact_counts)

    save_frame(0.0)
    episode_steps = round(args.duration / args.dt)
    steps = episode_steps
    loop_started = time.perf_counter()
    warmup_steps = round(0.1 / args.dt)
    measured_started = None
    for step_index in range(steps):
        # Retained original reset branch: unreachable for this one-episode protocol.
        if step_index == episode_steps:
            episode = 1
            robot.write_root_pose_to_sim(initial_state)
            robot.write_root_velocity_to_sim(
                torch.zeros((environment_count, 6), device=sim.device)
            )
            robot.write_joint_state_to_sim(
                torch.zeros((environment_count, 173), device=sim.device),
                torch.zeros((environment_count, 173), device=sim.device),
            )
            robot.set_joint_position_target(
                torch.zeros((environment_count, 173), device=sim.device)
            )
            robot.set_joint_velocity_target(
                torch.zeros((environment_count, 173), device=sim.device)
            )
            robot.write_data_to_sim()
            robot.reset()
            sim.forward()
            robot.update(0.0)
            max_position_error = float(torch.abs(robot.data.joint_pos).max())
            max_velocity_error = float(torch.abs(robot.data.joint_vel).max())
            assert max_position_error < 1e-07 and max_velocity_error < 1e-07
            reset_events.append(
                dict(
                    global_t_s=step_index * args.dt,
                    episode=episode,
                    q_max_abs=max_position_error,
                    qdot_max_abs=max_velocity_error,
                    command_time_restarts_at_zero=True,
                    source_permutation=experiment["episode1_permutation"],
                    terrain_geometry_changed=False,
                )
            )
            common.write(run_directory / "reset_events.json", reset_events)
            save_frame(step_index * args.dt)
        command_step_index = step_index % episode_steps
        if step_index == warmup_steps:
            measured_started = time.perf_counter()
        if step_index % command_stride == 0:
            tick = time.perf_counter()
            position_targets = command_positions[
                command_step_index // command_stride
            ].clone()
            velocity_targets = command_velocities[
                command_step_index // command_stride
            ].clone()
            if episode == 1:
                position_targets = position_targets[experiment["episode1_permutation"]]
                velocity_targets = velocity_targets[experiment["episode1_permutation"]]
            timing["commands"] += time.perf_counter() - tick
        # World-pitch feedback is refreshed every physics step (0.5 ms in this profile).
        current_positions = robot.data.joint_pos[
            control_environment_ids[:, None], control_joint_ids
        ]
        current_velocities = robot.data.joint_vel[
            control_environment_ids[:, None], control_joint_ids
        ]
        parent_orientations = robot.data.body_link_pose_w[
            control_environment_ids[:, None], control_body_ids, 3:
        ]
        parent_angular_velocity = robot.data.body_link_vel_w[
            control_environment_ids[:, None], control_body_ids, 3:
        ]
        (
            feedback_positions,
            feedback_velocities,
            world_pitch,
            world_pitch_rate,
            feedback_valid,
        ) = world_feedback.references(
            torch,
            parent_orientations,
            parent_angular_velocity,
            current_positions,
            current_velocities,
            control_joint_limits,
        )
        position_targets[control_environment_ids[:, None], control_joint_ids] = (
            feedback_positions
        )
        velocity_targets[control_environment_ids[:, None], control_joint_ids] = (
            feedback_velocities
        )
        robot.set_joint_position_target(position_targets)
        robot.set_joint_velocity_target(velocity_targets)
        robot.write_data_to_sim()
        if step_index % recording_stride == 0:
            controller_times.append(step_index * args.dt)
            controller_samples.append(
                as_numpy(
                    torch.stack(
                        [
                            current_positions,
                            current_velocities,
                            feedback_positions,
                            feedback_velocities,
                            world_pitch,
                            world_pitch_rate,
                            feedback_valid.to(current_positions.dtype),
                        ],
                        dim=-1,
                    )
                )
            )
        assert not robot.has_external_wrench
        if experiment["diagnostic_telemetry"]:
            momentum_before.append(
                (
                    native_mass[:, :, None]
                    * as_numpy(robot.data.body_com_vel_w)[..., :3]
                ).sum(1)
            )
            pre_contact_pose.append(as_numpy(robot.data.body_link_pose_w))
            momentum_t.append(step_index * args.dt)
        tick = time.perf_counter()
        sim.step(render=False)
        robot.update(args.dt)
        timing["physics"] += time.perf_counter() - tick
        if experiment["diagnostic_telemetry"]:
            momentum_after.append(
                (
                    native_mass[:, :, None]
                    * as_numpy(robot.data.body_com_vel_w)[..., :3]
                ).sum(1)
            )
        if experiment["diagnostic_telemetry"]:
            tick = time.perf_counter()
            sample_contacts((step_index + 1) * args.dt)
            timing["contact"] += time.perf_counter() - tick
        if not bool(
            torch.isfinite(robot.data.body_link_pose_w).all()
            and torch.isfinite(robot.data.body_link_vel_w).all()
            and torch.isfinite(robot.data.joint_pos).all()
            and torch.isfinite(robot.data.joint_vel).all()
        ):
            numerical_failure = dict(
                time_s=(step_index + 1) * args.dt,
                reason="nonfinite_state",
                environments=torch.nonzero(
                    ~(
                        torch.isfinite(robot.data.body_link_pose_w)
                        .all(dim=2)
                        .all(dim=1)
                        & torch.isfinite(robot.data.body_link_vel_w)
                        .all(dim=2)
                        .all(dim=1)
                        & torch.isfinite(robot.data.joint_pos).all(dim=1)
                        & torch.isfinite(robot.data.joint_vel).all(dim=1)
                    )
                )
                .flatten()
                .cpu()
                .tolist(),
            )
            numerical_failure["fields"] = {
                name: torch.nonzero(
                    ~torch.isfinite(value).reshape(environment_count, -1).all(dim=1)
                )
                .flatten()
                .cpu()
                .tolist()
                for name, value in [
                    ("pose", robot.data.body_link_pose_w),
                    ("body_velocity", robot.data.body_link_vel_w),
                    ("q", robot.data.joint_pos),
                    ("qdot", robot.data.joint_vel),
                ]
            }
            common.write(run_directory / "NUMERICAL_FAILURE.json", numerical_failure)
            save_frame((step_index + 1) * args.dt)
            break
        # A cropped collision mesh is valid only while every body stays well inside.
        local_xy = robot.data.body_link_pose_w[:, :, :2].detach().cpu().numpy() - offsets[:, None, :2]
        bx0, bx1, by0, by1 = experiment["terrain_roi_m"]
        boundary_margin = min(float(local_xy[:,:,0].min()-bx0), float(bx1-local_xy[:,:,0].max()), float(local_xy[:,:,1].min()-by0), float(by1-local_xy[:,:,1].max()))
        if boundary_margin < 0.015:
            failure_t=(step_index+1)*args.dt
            failure_pose=as_numpy(robot.data.body_link_pose_w)
            failure_pose[:,:,:3]-=offsets[:,None,:]
            failure_q=as_numpy(robot.data.joint_pos)
            failure_v=as_numpy(robot.data.joint_vel)
            failure_body_v=as_numpy(robot.data.body_link_vel_w)
            env_bounds=[]
            for ee in range(environment_count):
                pxyz=failure_pose[ee,:,:3]
                margins=[float(pxyz[:,0].min()-bx0),float(bx1-pxyz[:,0].max()),float(pxyz[:,1].min()-by0),float(by1-pxyz[:,1].max())]
                env_bounds.append(dict(environment_index=ee,wave=rows[ee]["wave"],margins_m=margins,
                    xyz_min_m=pxyz.min(0).tolist(),xyz_max_m=pxyz.max(0).tolist(),
                    max_abs_joint_velocity_rad_s=float(abs(failure_v[ee]).max()),
                    max_body_linear_speed_m_s=float(np.linalg.norm(failure_body_v[ee,:,:3].astype(float),axis=-1).max())))
            common.write(run_directory / "BOUNDARY_FAILURE.json", dict(t=failure_t, minimum_body_center_margin_m=boundary_margin, environments=env_bounds,
                saved_regular_frame_end_s=frame_times[-1], all_state_finite=bool(np.isfinite(failure_pose).all() and np.isfinite(failure_v).all())))
            np.savez_compressed(run_directory/"failure_state.npz",t=failure_t,pose=failure_pose,q=failure_q,v=failure_v,body_velocity=failure_body_v,body_names=body_names,joint_names=joint_names)
            np.savez_compressed(run_directory/"trajectory_partial.npz",t=frame_times,episode=frame_episodes,pose=body_poses,q=joint_positions,v=joint_velocities,body_velocity=body_velocities,body_names=body_names,joint_names=joint_names)
            raise RuntimeError("Body entered 15 mm crop-boundary guard; this preview cannot be accepted")
        if (step_index + 1) % recording_stride == 0:
            save_frame((step_index + 1) * args.dt)
            sample_cluster((step_index + 1) * args.dt)
        if (step_index + 1) % round(0.1 / args.dt) == 0:
            flush_telemetry()
        if (step_index + 1) % round(0.1 / args.dt) == 0:
            print(
                json.dumps(
                    {
                        "t": (step_index + 1) * args.dt,
                        "wall_s": time.monotonic() - started,
                    }
                ),
                flush=True,
            )
    measured_wall = time.perf_counter() - measured_started if measured_started else None
    common.write(
        run_directory / "runtime_settings.json",
        {
            key: sim._settings.get(key)
            for key in [
                "/physics/updateToUsd",
                "/physics/suppressReadback",
                "/physics/updateVelocitiesToUsd",
                "/physics/updateForceSensorsToUsd",
            ]
        },
    )
    common.write(
        run_directory / "benchmark_timing.json",
        dict(
            measured_wall_s=measured_wall,
            warmup_steps=warmup_steps,
            measured_steps=max(0, step_index + 1 - warmup_steps),
            environment_count=environment_count,
            loop_wall_s=time.perf_counter() - loop_started,
            steps_completed=step_index + 1,
            simulated_s=(step_index + 1) * args.dt,
            physics_timing_s=timing,
            source_ids=source_ids,
        ),
    )
    np.savez_compressed(
        run_directory / "world_feedback.npz",
        t=controller_times,
        data=controller_samples,
        fields=[
            "q",
            "qdot",
            "q_target",
            "qdot_target",
            "world_pitch",
            "world_pitch_rate",
            "valid",
        ],
        environments=controlled_environments,
        joints=experiment["controlled_joints"],
    )
    np.savez_compressed(
        run_directory / "trajectory.npz",
        t=frame_times,
        episode=frame_episodes,
        pose=body_poses,
        q=joint_positions,
        v=joint_velocities,
        body_velocity=body_velocities,
        body_names=body_names,
        joint_names=joint_names,
    )
    np.savez_compressed(
        run_directory / "cluster_contacts.npz",
        t=cluster_times,
        force=cluster_forces,
        point=cluster_points,
        count=cluster_counts,
        foot_names=[body_names[i] for i in foot_ids],
    )
    flush_telemetry()
    common.write(
        run_directory / "telemetry_chunks.json",
        dict(
            chunks=telemetry_chunks,
            chunk_io_s=chunk_io_s,
            complete=numerical_failure is None,
            schema="diagnostic telemetry chunks when enabled; intentionally empty when disabled",
        ),
    )
    sys.stdout.flush()
    sys.stderr.flush()
    errors = [
        line
        for line in (runtime.run_directory / "worker.log")
        .read_text(errors="replace")
        .splitlines()
        if re.search(
            "\\[Error\\]|overflow|out of memory|buffer.*too small|failed.*alloc|dropped.*contact",
            line,
            re.I,
        )
    ]
    common.write(run_directory / "native_error_audit.json", dict(errors=errors))
    common.write(
        run_directory / "results.json",
        dict(
            timing_s=timing,
            position_iterations=args.position_iterations,
            velocity_iterations=args.velocity_iterations,
            complete=numerical_failure is None,
            actual_duration_s=frame_times[-1],
            numerical_failure=numerical_failure,
            engine="GPU_PhysX" if args.device == "cuda:0" else "CPU_PhysX",
            wall_s=time.monotonic() - started,
            rows=rows,
            duration=args.duration,
            episodes=1,
            reset_count=len(reset_events),
            finite=bool(
                np.isfinite(body_poses).all()
                and np.isfinite(joint_positions).all()
                and np.isfinite(joint_velocities).all()
            ),
            whole_batch_finite=numerical_failure is None,
            finite_check_interval_s=args.dt,
        ),
    )
