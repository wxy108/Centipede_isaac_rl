"""Authorized600 extra Libo observations; frozen physics and original worker."""
from pathlib import Path
import os
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[key] = '1'
os.environ['PXR_WORK_THREAD_LIMIT'] = '2'
import sys, json, copy, ast, argparse, shutil
import numpy as np

ROOT = Path(__file__).resolve().parent
MAIN = ROOT.parent
sys.path.insert(0, str(ROOT))
import run_libo_repeats as original

read, sha, dump = original.read, original.sha, original.dump
PARENT = ROOT / 'libo_spawn_repeats'
BASE = PARENT / 'extension_spawn30_2026-09-30'
SIM = BASE / 'simulation'
HEIGHTS = [.25, .5, .75, 1., 1.25, 1.5]
WAVES = [1.5, 2., 2.5, 3., 3.5]
NEW_SPAWNS = list(range(10, 30))
SEED = 2026093001
AUTHORIZATION = '2026-09-30 user: 好的，去做吧; scope is the immediately proposed600 extra cases at the six existing4mm Libo heights only.'

original.OUT = BASE
original.SIM = SIM
original.__file__ = str(Path(__file__).resolve())
old_source = (ROOT / 'run_libo_repeats.py').read_text(encoding='utf-8')
nodes = {n.name: ast.get_source_segment(old_source, n) for n in ast.parse(old_source).body if isinstance(n, ast.FunctionDef)}
run_source = nodes['run'].replace('all205 planned observations', 'all600 planned observations')
assert run_source != nodes['run']
exec(compile(run_source, str(Path(__file__).resolve()), 'exec'), original.__dict__)


def make_spawns(parent):
    from scipy.spatial import ConvexHull
    refs, origin_offset = [], None
    pinned_flat_sources = parent['flat_reference_sources']
    for path, digest in pinned_flat_sources.items():
        assert sha(path) == digest
        path = Path(path)
        rb = read(path.parent / 'readback.json')
        source_rows = read(path.parent / 'manifest.json')['rows']
        with np.load(path) as z:
            pose = z['pose'][z['t'] <= 6 + 1e-9, :, :, :3]
        for environment in range(pose.shape[1]):
            if source_rows[environment]['wave'] > 3.5:
                continue
            origin = pose[0, environment].mean(0)
            root_pose = np.array(rb['initial_pose'][environment])
            delta = origin[:2] - root_pose[:2]
            if origin_offset is None:
                origin_offset = delta
            np.testing.assert_allclose(delta, origin_offset, atol=1e-7, rtol=0)
            refs.append((pose[:, environment, :, :2] - origin[:2]).reshape(-1, 2))
    # Reuse exactly the same five-wave flat envelope construction as the parent.
    refs = np.concatenate(refs)
    hull = refs[ConvexHull(refs).vertices]
    rmax = max(parent['collision_bounds']['body_radius_m'].values())
    roi = np.array([[parent['terrain_roi_m'][0], parent['terrain_roi_m'][1]],
                    [parent['terrain_roi_m'][2], parent['terrain_roi_m'][3]]])
    rng = np.random.default_rng(SEED)
    spawns = copy.deepcopy(parent['spawns'])
    assert [s['spawn_index'] for s in spawns] == list(range(10))
    for sid in NEW_SPAWNS:
        sector = (sid - 10) % 10
        yaw = float((sector + rng.random()) * 36)
        angle = np.deg2rad(yaw)
        rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        rotated = hull @ rotation.T
        lo = roi[:, 0] - rotated.min(0) + .03 + rmax
        hi = roi[:, 1] - rotated.max(0) - .03 - rmax
        assert np.all(hi > lo)
        center = lo + rng.uniform(.1, .9, 2) * (hi - lo)
        root = center - origin_offset @ rotation.T
        actual = rotated + center
        margins = [float(actual[:, 0].min() - roi[0, 0] - rmax),
                   float(roi[0, 1] - actual[:, 0].max() - rmax),
                   float(actual[:, 1].min() - roi[1, 0] - rmax),
                   float(roi[1, 1] - actual[:, 1].max() - rmax)]
        assert min(margins) > .03
        spawns.append(dict(spawn_index=sid, start_xy_m=root.tolist(), start_yaw_deg=yaw,
                           initial_mean_link_center_xy_m=center.tolist(),
                           initial_center_sampling_box_m=[lo.tolist(), hi.tolist()],
                           predicted_flat_collision_margins_m=margins,
                           forward_xy=[float(-np.cos(angle)), float(-np.sin(angle))],
                           CPG_initial_phase='Original command sample0',
                           new_heading_sector=sector, new_heading_round=(sid - 10) // 10))
    assert spawns[:10] == parent['spawns']
    assert len({tuple(s['start_xy_m']) for s in spawns}) == 30
    return spawns, hull


def check():
    c = read(SIM / 'campaign.json')
    parent = read(PARENT / 'simulation/campaign.json')
    for path, digest in c['pins'].items():
        assert sha(path) == digest, path
    assert c['spawns'][:10] == parent['spawns'] and len(c['spawns']) == 30
    assert c['profile'] == parent['profile'] and c['flat_cluster_cells'] == parent['flat_cluster_cells']
    assert c['fixed_pd_sha256'] == parent['fixed_pd_sha256']
    assert c['collision_bounds'] == parent['collision_bounds']
    assert c['workers'] == 4 and c['threads_per_worker'] == 2 and c['environments_per_worker'] == [3, 2, 3, 2]
    assert c['new_flat_condition_count'] == 0 and c['condition_count'] == 600
    assert sha(SIM / 'executed_source/adapted_functions.py') == sha(PARENT / 'simulation/executed_source/adapted_functions.py')
    pd = read(MAIN / 'pd_wave4_firm20.json')['absolute_joint_pd']
    assert len(pd) == 173
    seen = set()
    for job in c['jobs']:
        args = job['args']
        assert args['dt'] == .0005 and args['duration'] == 6 and args['position_iterations'] == 64 and args['velocity_iterations'] == 16
        assert sha(args['rows']) == job['rows_sha256']
        assert sha(args['experiment_plan']) == args['experiment_plan_sha256']
        plan, rows = read(args['experiment_plan']), read(args['rows'])
        assert plan['benchmark_source_ids'] == job['source_ids']
        for cell, source_id in zip(job['cells'], job['source_ids']):
            row = rows[source_id]
            assert row['absolute_joint_pd'] == pd and not row['family_multipliers']
            sp = c['spawns'][row['spawn_index']]
            assert row['start_xy'] == sp['start_xy_m'] and row['start_yaw_deg'] == sp['start_yaw_deg']
            ref = next(r for r in c['flat_cluster_cells'] if r['wave'] == cell['wave'])
            assert all(cell[k] == ref[k] for k in ['A_deg', 'B_deg', 'body_factor', 'leg_factor'])
            assert row['start_index'] == 0 and sha(row['surface_file']) == row['surface_sha256']
            key = (row['height_scale'], row['wave'], row['spawn_index'])
            assert key not in seen
            seen.add(key)
    assert seen == {(h, w, sid) for h in HEIGHTS for w in WAVES for sid in NEW_SPAWNS}
    assert len(c['jobs']) == 240 and len(c['batches']) == 60
    dump(SIM / 'PREPARATION_CHECK.json', dict(passed=True, new_conditions=600,
         original_spawns_preserved=True, original_worker_source_bitexact=True,
         full173_PD_fixed=True, amplitudes_fixed=True, physics_profile_fixed=True,
         terrain_surfaces_unchanged=True, batch_layout_preserved=[3, 2, 3, 2],
         authorization=AUTHORIZATION, no_capacity_or_convergence_search=True))
    print('INPUT CHECKS PASS:600 new conditions; original6s definition and native worker preserved', flush=True)


original.check = check


def prepare():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    assert not BASE.exists(), 'Preserve earlier spawn30 preparation'
    parent = read(PARENT / 'simulation/campaign.json')
    extension = read(PARENT / 'extension_1p25_1p5/simulation/campaign.json')
    assert read(PARENT / 'simulation/supervisor.json')['complete']
    assert read(PARENT / 'extension_1p25_1p5/simulation/supervisor.json')['complete']
    for path, digest in {**parent['pins'], **extension['pins']}.items():
        assert sha(path) == digest, path
    assert parent['spawns'] == extension['spawns'] and parent['profile'] == extension['profile']
    spawns, hull = make_spawns(parent)
    BASE.mkdir()
    SIM.mkdir()
    snapshot = SIM / 'executed_source'
    snapshot.mkdir()
    shutil.copy2(PARENT / 'simulation/executed_source/adapted_functions.py', snapshot / 'adapted_functions.py')
    (snapshot / 'scheduler_function.py').write_text(run_source + '\n', encoding='utf-8')
    pd = read(MAIN / 'pd_wave4_firm20.json')['absolute_joint_pd']
    cells = parent['flat_cluster_cells']
    jobs = []
    for height in HEIGHTS:
        source_campaign = parent if height <= 1 else extension
        template_job = next(j for j in source_campaign['jobs'] if j['height_scale'] == height and j['group'] == 'low')
        old_rows = read(template_job['args']['rows'])
        templates = {w: next(row for row in old_rows if row['wave'] == w) for w in WAVES}
        surface = Path(templates[WAVES[0]]['surface_file'])
        assert sha(surface) == templates[WAVES[0]]['surface_sha256']
        with np.load(surface) as z:
            terrain, spacing = z['height'].copy(), float(z['dx_m'])
        level = 'height_x' + f'{height:.2f}'.replace('.', 'p')
        rows, initials = [], []
        for sid in NEW_SPAWNS:
            sp = spawns[sid]
            for wave in WAVES:
                row = copy.deepcopy(templates[wave])
                row.update(id=f'{level}_s{sid:02d}_w{wave:g}', spawn_index=sid,
                           start_xy=sp['start_xy_m'], start_yaw_deg=sp['start_yaw_deg'], start_index=0)
                assert row['absolute_joint_pd'] == pd and not row['family_multipliers']
                allowed = {'id', 'spawn_index', 'start_xy', 'start_yaw_deg', 'start_index'}
                assert all(row[k] == templates[wave][k] for k in row if k not in allowed)
                pose = np.array(row['matched_initial_root_pose'], float)
                pose[:2] = sp['start_xy_m']
                angle = np.deg2rad(sp['start_yaw_deg'])
                pose[3:] = [np.cos(angle / 2), 0, 0, np.sin(angle / 2)]
                ny, nx = terrain.shape
                ix, iy = int(pose[0] / spacing + (nx - 1) / 2), int(pose[1] / spacing + (ny - 1) / 2)
                radius = int(np.ceil(.075 / spacing))
                pose[2] += terrain[max(0, iy-radius):min(ny, iy+radius+1), max(0, ix-radius):min(nx, ix+radius+1)].max() + .002
                rows.append(row)
                initials.append(pose.tolist())
        rows_path = SIM / f'{level}_rows.json'
        dump(rows_path, rows)
        for sid in NEW_SPAWNS:
            local = (sid - 10) * 5
            for group, ids in [('low', list(range(local, local+3))), ('high', list(range(local+3, local+5)))]:
                template = next(j for j in source_campaign['jobs'] if j['height_scale'] == height and j['group'] == group)
                n = len(ids)
                name = f'{level}_s{sid:02d}_{group}'
                selected = [dict(cells[i % 5], source_id=i, baseline_source_id=cells[i % 5]['source_id'],
                                 spawn_index=sid, height_scale=height, surface=level,
                                 fixed_pair_angle_rms_deg=template['cells'][0]['fixed_pair_angle_rms_deg']) for i in ids]
                plan = read(template['args']['experiment_plan'])
                plan.update(schema='libo_spawn30_extension_v1', run_name=name,
                            rows_sha256=sha(rows_path), benchmark_source_ids=ids, selected_environments=ids,
                            pd_overrides={str(i):pd for i in ids}, planned_initial_pose=initials,
                            body_factors=[c['body_factor'] for c in selected],
                            leg_cluster_factors=[c['leg_factor'] for c in selected], joint_cells=selected,
                            quality_offsets_m=[[0., 0., 0.]] * n, episode1_permutation=list(range(n)),
                            worker_filename=Path(__file__).name, campaign_note=AUTHORIZATION,
                            quality_protocol=dict(duration_s=6., comparison='New20 shared spawns appended to original10, six existing4mm heights and five fixed waves', only_focal_poses_recorded=False))
                plan['scaling_benchmark']['replicas'] = n
                plan_path = SIM / f'{name}_plan.json'
                dump(plan_path, plan)
                job = copy.deepcopy(template)
                job.update(name=name, level=level, spawn_index=sid, source_ids=ids, cells=selected,
                           rows_sha256=sha(rows_path))
                job['args'].update(name=name, rows=str(rows_path), experiment_plan=str(plan_path),
                                   experiment_plan_sha256=sha(plan_path))
                jobs.append(job)
    batches = []
    for pair in range(10):
        shift = pair % len(HEIGHTS)
        for height in HEIGHTS[shift:] + HEIGHTS[:shift]:
            names = [j['name'] for sid in [10 + 2*pair, 11 + 2*pair] for group in ['low', 'high']
                     for j in jobs if j['height_scale'] == height and j['spawn_index'] == sid and j['group'] == group]
            assert len(names) == 4
            batches.append(dict(name=f'new_pair{pair:02d}_height{height:g}', jobs=names))
    pins = dict(parent['pins'], **extension['pins'])
    additional = [Path(__file__), PARENT / 'simulation/campaign.json',
                  PARENT / 'extension_1p25_1p5/simulation/campaign.json', snapshot / 'adapted_functions.py',
                  snapshot / 'scheduler_function.py']
    pins.update({str(path):sha(path) for path in additional})
    for path in [Path(__file__), ROOT / 'run_libo_repeats.py', ROOT / 'run_shortwave.py']:
        shutil.copy2(path, snapshot / path.name)
    campaign = copy.deepcopy(parent)
    campaign.update(schema='libo_shared_spawn30_extension_v1', jobs=jobs, batches=batches, pins=pins,
                    spawns=spawns, spawn_seed=SEED, height_scales=HEIGHTS,
                    condition_count=600, rough_condition_count=600, new_flat_condition_count=0,
                    new_spawn_indices=NEW_SPAWNS, total_shared_spawns=30, appended_shared_spawns=20,
                    old_conditions_preserved=300, preserved_flat_controls=5,
                    parents=[str(PARENT / 'simulation/campaign.json'), str(PARENT / 'extension_1p25_1p5/simulation/campaign.json')],
                    authorization=AUTHORIZATION,
                    scope='Only600 new observations at existing Libo4mm heights; no higher-height/cutoff/random-erasure runs',
                    spawn_policy='Retain original10 exactly; add20 fresh position/heading conditions, two new random headings per original36deg sector; original collision-envelope and location-sampling rule.',
                    heading_sector_width_deg=36, new_heading_rounds=2,
                    boundary_envelope='Same five-wave original flat references, envelope construction and authored radii; minimum30mm predicted margin and original15mm per-step guard.',
                    preselected_new_demo_spawn=12)
    dump(SIM / 'campaign.json', campaign)
    fig, ax = plt.subplots(figsize=(10, 7))
    roi = campaign['terrain_roi_m']
    ax.set_aspect('equal')
    ax.set(xlim=roi[:2], ylim=roi[2:], xlabel='X (m)', ylabel='Y (m)', title='Libo:30 shared spawn positions and initial heading')
    palette = plt.cm.tab20(np.linspace(0, 1, 20))
    for sp in spawns:
        sid = sp['spawn_index']
        x, y = sp['initial_mean_link_center_xy_m']
        fx, fy = sp['forward_xy']
        color = '#a4aab0' if sid < 10 else palette[sid-10]
        ax.scatter([x], [y], color=color, s=22)
        ax.arrow(x, y, .022*fx, .022*fy, width=.0007, color=color, length_includes_head=True)
        ax.text(x+.003, y+.003, str(sid), fontsize=7.5, color=color)
    ax.grid(alpha=.2)
    fig.text(.1, .035, 'Gray0-9: preserved historical cases. Color10-29: new cases. Same phase0,30mm envelope reserve,15mm native boundary guard.', fontsize=8.5)
    fig.savefig(BASE / 'spawn_preview.png', dpi=160)
    plt.close(fig)
    check()
    print('PREPARED600:30 groups,20 new shared spawns,60 batches of4 workers, original2 threads and3/2 environment layout', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['prepare', 'check', 'run', 'worker'])
    parser.add_argument('--job')
    parser.add_argument('--attempt', default='v1')
    parser.add_argument('--keep')
    args = parser.parse_args()
    if args.mode == 'prepare':
        prepare()
    elif args.mode == 'check':
        check()
    elif args.mode == 'run':
        original.run()
    else:
        original.worker(args.job, args.attempt, None if args.keep is None else list(map(int, args.keep.split(','))))
