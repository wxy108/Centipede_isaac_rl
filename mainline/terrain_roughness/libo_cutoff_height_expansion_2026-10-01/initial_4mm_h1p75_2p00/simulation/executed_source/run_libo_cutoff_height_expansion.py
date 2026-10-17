"""Run an explicitly supplied Libo terrain stage with the frozen native worker.

The finite manifest is the execution scope. This runner does not choose subsequent
height levels, interpret a locomotion stopping criterion, or resume other studies.
"""
from pathlib import Path
import os
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[key] = '1'
os.environ['PXR_WORK_THREAD_LIMIT'] = '2'
import argparse
import ast
import copy
import json
import re
import shutil
import sys
import numpy as np

ROOT = Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
MAIN = ROOT.parent
sys.path.insert(0, str(ROOT))
import run_libo_repeats as original

read, sha, dump = original.read, original.sha, original.dump
PARENT = ROOT / 'libo_spawn_repeats'
SHARED_PARENT = PARENT / 'extension_spawn30_2026-09-30/simulation'
EXPANSION_ROOT = ROOT / 'libo_cutoff_height_expansion_2026-10-01'
WAVES = [1.5, 2., 2.5, 3., 3.5]
SPAWN_INDICES = list(range(30))
AUTHORIZATION = ('2026-10-01 user: 好，做1.75以上高度和2/8mm截断吧，今晚都做完，你有什么问题吗. '
                 'Only the supplied finite Libo height/cutoff manifest is dispatched; '
                 'random-erasure studies remain paused, no shutdown or parameter search.')
RUNNER = Path(__file__).resolve()
SOURCE = (ROOT / 'run_libo_repeats.py').read_text(encoding='utf-8')
FUNCTIONS = {node.name: ast.get_source_segment(SOURCE, node)
             for node in ast.parse(SOURCE).body if isinstance(node, ast.FunctionDef)}
BASE = SIM = STAGE = RUN_SOURCE = None


def configure(stage, condition_count):
    global BASE, SIM, STAGE, RUN_SOURCE
    assert re.fullmatch(r'[a-z0-9][a-z0-9_-]{0,63}', stage), 'Stage must be a safe directory name'
    assert stage.upper() not in {'CON', 'PRN', 'AUX', 'NUL', *(f'COM{i}' for i in range(10)),
                                 *(f'LPT{i}' for i in range(10))}
    STAGE = stage
    BASE = EXPANSION_ROOT / stage
    SIM = BASE / 'simulation'
    original.OUT, original.SIM = BASE, SIM
    original.__file__ = str(RUNNER)
    original.EXPANSION_STAGE = stage
    original.check = check
    # Two exact textual substitutions retain the original scheduler/recovery logic.
    anchor = "cmd=[sys.executable,'-B',str(Path(__file__)),'worker','--job',name,'--attempt',attempt]"
    replacement = anchor[:-1] + ",'--stage',EXPANSION_STAGE]"
    caption = 'all205 planned observations'
    assert FUNCTIONS['run'].count(anchor) == 1 and FUNCTIONS['run'].count(caption) == 1
    RUN_SOURCE = FUNCTIONS['run'].replace(anchor, replacement).replace(
        caption, f'all{condition_count} planned observations')
    assert RUN_SOURCE.replace(replacement, anchor).replace(
        f'all{condition_count} planned observations', caption) == FUNCTIONS['run']
    ast.parse(RUN_SOURCE)
    exec(compile(RUN_SOURCE, str(RUNNER), 'exec'), original.__dict__)


def parents():
    parent = read(PARENT / 'simulation/campaign.json')
    shared = read(SHARED_PARENT / 'campaign.json')
    assert read(PARENT / 'simulation/supervisor.json')['complete']
    assert read(SHARED_PARENT / 'supervisor.json')['complete']
    assert shared['spawns'][:10] == parent['spawns']
    assert len(shared['spawns']) == 30
    assert [sp['spawn_index'] for sp in shared['spawns']] == SPAWN_INDICES
    for key in ('profile', 'profile_sha256', 'fixed_pd_sha256', 'flat_cluster_cells',
                'collision_bounds', 'terrain_roi_m', 'flat_reference_sources'):
        assert shared[key] == parent[key], key
    return parent, shared


def terrain_level(cutoff, height):
    number = format(height, '.12g').replace('.', 'p')
    return f'cutoff_{cutoff:g}mm_height_x{number}'


def load_manifest(path):
    path = Path(path).resolve()
    payload = read(path)
    records = payload if isinstance(payload, list) else payload['terrains']
    assert isinstance(records, list) and records, 'A nonempty finite terrain list is required'
    result, seen = [], set()
    for supplied in records:
        record = copy.deepcopy(supplied)
        cutoff, height = float(record['cutoff_mm']), float(record['height_scale'])
        rms = float(record['fixed_pair_angle_rms_deg'])
        assert cutoff in (2., 4., 8.)
        assert np.isfinite(height) and height > 0
        assert np.isfinite(rms) and rms >= 0
        key = (cutoff, height)
        assert key not in seen, f'Duplicate terrain: {key}'
        seen.add(key)
        # Completed4mm groups are evidence to reuse, not newly dispatched repeats.
        assert not (cutoff == 4. and height in (.25, .5, .75, 1., 1.25, 1.5)), \
            'Existing4mm30-spawn group must be reused instead of reexecuted'
        surface = Path(record['path']).resolve()
        assert surface.is_file() and sha(surface) == record['sha256'], str(surface)
        with np.load(surface, allow_pickle=False) as data:
            terrain = data['height']
            spacing = float(data['dx_m'])
            assert terrain.ndim == 2 and min(terrain.shape) > 1
            assert np.isfinite(terrain).all() and np.isfinite(spacing) and spacing > 0
            shape = list(terrain.shape)
        # Geometry matching is chosen in the supplied terrain generation receipt.
        # Validate ROI coverage here; do not silently resample or normalize surfaces.
        roi = original.ROI
        xspan, yspan = (shape[1] - 1) * spacing, (shape[0] - 1) * spacing
        assert -xspan / 2 <= roi[0] < roi[1] <= xspan / 2
        assert -yspan / 2 <= roi[2] < roi[3] <= yspan / 2
        record.update(cutoff_mm=cutoff, height_scale=height, path=str(surface),
                      fixed_pair_angle_rms_deg=rms, level=terrain_level(cutoff, height),
                      grid_shape=shape, grid_spacing_m=spacing)
        result.append(record)
    assert len({r['level'] for r in result}) == len(result), 'Terrain label collision'
    return path, result


def template_inputs(parent):
    jobs = {group: next(j for j in parent['jobs'] if j['height_scale'] == 1.
                       and j['spawn_index'] == 0 and j['group'] == group)
            for group in ('low', 'high')}
    source_rows = read(jobs['low']['args']['rows'])
    templates = {wave: next(row for row in source_rows if row['wave'] == wave)
                 for wave in WAVES}
    assert all(row['terrain'] == 'rough' and row['terrain_site'] == 'libo'
               for row in templates.values())
    return jobs, templates


ROW_VARIABLES = {'id', 'spawn_index', 'start_xy', 'start_yaw_deg', 'start_index',
                 'height_scale', 'surface_file', 'surface_sha256', 'restoration_level', 'cutoff_mm'}


def initial_pose(row, spawn, terrain, spacing):
    """Exact historical local-height clearance formula; baseline Z is not double-added."""
    pose = np.array(row['matched_initial_root_pose'], float)
    pose[:2] = spawn['start_xy_m']
    angle = np.deg2rad(spawn['start_yaw_deg'])
    pose[3:] = [np.cos(angle / 2), 0, 0, np.sin(angle / 2)]
    ny, nx = terrain.shape
    ix = int(pose[0] / spacing + (nx - 1) / 2)
    iy = int(pose[1] / spacing + (ny - 1) / 2)
    radius = int(np.ceil(.075 / spacing))
    local = terrain[max(0, iy-radius):min(ny, iy+radius+1),
                    max(0, ix-radius):min(nx, ix+radius+1)]
    assert local.size
    pose[2] += local.max() + .002
    return pose.tolist()


def check():
    campaign = read(SIM / 'campaign.json')
    parent, shared = parents()
    for path, digest in campaign['pins'].items():
        assert sha(path) == digest, path
    for key in ('profile', 'profile_sha256', 'fixed_pd_sha256', 'flat_cluster_cells',
                'collision_bounds', 'terrain_roi_m', 'flat_reference_sources', 'spawns'):
        assert campaign[key] == shared[key], key
    assert campaign['stage'] == STAGE
    assert campaign['workers'] == 4 and campaign['threads_per_worker'] == 2
    assert campaign['environments_per_worker'] == [3, 2, 3, 2]
    assert campaign['new_flat_condition_count'] == 0
    assert campaign['condition_count'] == len(campaign['terrains']) * 150
    assert sha(SIM / 'executed_source/adapted_functions.py') == sha(
        PARENT / 'simulation/executed_source/adapted_functions.py')
    assert (SIM / 'executed_source/original_worker_function.py').read_text(
        encoding='utf-8') == FUNCTIONS['worker'] + '\n'
    assert (SIM / 'executed_source/scheduler_function.py').read_text(
        encoding='utf-8') == RUN_SOURCE + '\n'
    pd = read(MAIN / 'pd_wave4_firm20.json')['absolute_joint_pd']
    assert len(pd) == 173
    template_jobs, templates = template_inputs(parent)
    terrain_by_level = {t['level']: t for t in campaign['terrains']}
    seen, per_job_count = set(), []
    for job in campaign['jobs']:
        args, reference_job = job['args'], template_jobs[job['group']]
        assert args['dt'] == .0005 and args['duration'] == 6.
        assert args['position_iterations'] == 64 and args['velocity_iterations'] == 16
        assert all(args[key] == value for key, value in reference_job['args'].items()
                   if key not in {'name', 'rows', 'experiment_plan', 'experiment_plan_sha256'})
        assert sha(args['rows']) == job['rows_sha256']
        assert sha(args['experiment_plan']) == args['experiment_plan_sha256']
        plan, rows = read(args['experiment_plan']), read(args['rows'])
        assert plan['rows_sha256'] == job['rows_sha256']
        assert plan['benchmark_source_ids'] == job['source_ids']
        assert plan['selected_environments'] == job['source_ids']
        assert plan['resource_threads'] == 2 and plan['control_dt_s'] == .001
        assert plan['feedback_dt_s'] == .0005
        terrain = terrain_by_level[job['level']]
        with np.load(terrain['path']) as data:
            heights, spacing = data['height'], float(data['dx_m'])
        for cell, source_id in zip(job['cells'], job['source_ids']):
            row, wave = rows[source_id], cell['wave']
            template = templates[wave]
            assert set(row) == set(template) | {'cutoff_mm'}
            assert all(row[key] == value for key, value in template.items() if key not in ROW_VARIABLES)
            assert row['absolute_joint_pd'] == pd and not row['family_multipliers']
            spawn = campaign['spawns'][row['spawn_index']]
            assert row['start_xy'] == spawn['start_xy_m']
            assert row['start_yaw_deg'] == spawn['start_yaw_deg'] and row['start_index'] == 0
            assert row['surface_file'] == terrain['path'] and row['surface_sha256'] == terrain['sha256']
            assert row['cutoff_mm'] == cell['cutoff_mm'] == terrain['cutoff_mm']
            assert row['height_scale'] == cell['height_scale'] == terrain['height_scale']
            assert cell['fixed_pair_angle_rms_deg'] == terrain['fixed_pair_angle_rms_deg']
            reference = next(c for c in campaign['flat_cluster_cells'] if c['wave'] == wave)
            assert all(cell[key] == reference[key] for key in ('A_deg', 'B_deg', 'body_factor', 'leg_factor'))
            assert plan['pd_overrides'][str(source_id)] == pd
            np.testing.assert_array_equal(plan['planned_initial_pose'][source_id],
                                          initial_pose(row, spawn, heights, spacing))
            key = (cell['cutoff_mm'], cell['height_scale'], wave, row['spawn_index'])
            assert key not in seen
            seen.add(key)
        per_job_count.append(len(job['source_ids']))
    expected = {(t['cutoff_mm'], t['height_scale'], w, sid)
                for t in campaign['terrains'] for w in WAVES for sid in SPAWN_INDICES}
    assert seen == expected and len(seen) == campaign['condition_count']
    assert len(campaign['jobs']) == len(campaign['terrains']) * 60
    assert len(campaign['batches']) == len(campaign['terrains']) * 15
    lookup = {job['name']: job for job in campaign['jobs']}
    batched = []
    for batch in campaign['batches']:
        jobs = [lookup[name] for name in batch['jobs']]
        assert [len(j['source_ids']) for j in jobs] == [3, 2, 3, 2]
        assert [j['group'] for j in jobs] == ['low', 'high', 'low', 'high']
        assert jobs[0]['spawn_index'] == jobs[1]['spawn_index']
        assert jobs[2]['spawn_index'] == jobs[3]['spawn_index'] == jobs[0]['spawn_index'] + 1
        assert len({j['level'] for j in jobs}) == 1
        batched.extend(batch['jobs'])
    assert sorted(batched) == sorted(lookup)
    dump(SIM / 'PREPARATION_CHECK.json', dict(passed=True, stage=STAGE,
         new_conditions=len(seen), terrain_count=len(campaign['terrains']), shared_spawn_count=30,
         original_worker_source_bitexact=True, scheduler_only_stage_and_caption_changes=True,
         full173_PD_fixed=True, amplitudes_fixed=True, physics_profile_fixed=True,
         exact30_shared_spawns_reused=True, no_new_flat_trials=True,
         original_initial_height_formula_preserved=True, batch_layout_preserved=[3, 2, 3, 2],
         authorization=AUTHORIZATION, no_capacity_or_convergence_search=True))
    print(f'INPUT CHECKS PASS: {len(seen)} finite-manifest conditions; unchanged native worker', flush=True)


def prepare(manifest_path):
    assert not BASE.exists(), 'Preserve previous stage; use a fresh stage name'
    manifest_path, terrains = load_manifest(manifest_path)
    parent, shared = parents()
    for path, digest in {**parent['pins'], **shared['pins']}.items():
        assert sha(path) == digest, path
    template_jobs, templates = template_inputs(parent)
    spawns = copy.deepcopy(shared['spawns'])
    pd = read(MAIN / 'pd_wave4_firm20.json')['absolute_joint_pd']
    cells = shared['flat_cluster_cells']
    SIM.mkdir(parents=True)
    snapshot = SIM / 'executed_source'
    snapshot.mkdir()
    shutil.copy2(PARENT / 'simulation/executed_source/adapted_functions.py', snapshot / 'adapted_functions.py')
    (snapshot / 'original_worker_function.py').write_text(FUNCTIONS['worker'] + '\n', encoding='utf-8')
    (snapshot / 'scheduler_function.py').write_text(RUN_SOURCE + '\n', encoding='utf-8')
    shutil.copy2(manifest_path, SIM / 'TERRAIN_MANIFEST.json')
    jobs = []
    for terrain in terrains:
        level = terrain['level']
        cutoff, height = terrain['cutoff_mm'], terrain['height_scale']
        with np.load(terrain['path']) as data:
            heights, spacing = data['height'].copy(), float(data['dx_m'])
        rows, initials = [], []
        for sid in SPAWN_INDICES:
            spawn = spawns[sid]
            for wave in WAVES:
                row = copy.deepcopy(templates[wave])
                row.update(id=f'{level}_s{sid:02d}_w{wave:g}', spawn_index=sid,
                           start_xy=spawn['start_xy_m'], start_yaw_deg=spawn['start_yaw_deg'],
                           start_index=0, height_scale=height, cutoff_mm=cutoff,
                           surface_file=terrain['path'], surface_sha256=terrain['sha256'],
                           restoration_level=f'nominal_{cutoff:g}mm')
                assert all(row[key] == value for key, value in templates[wave].items()
                           if key not in ROW_VARIABLES)
                rows.append(row)
                initials.append(initial_pose(row, spawn, heights, spacing))
        rows_path = SIM / f'{level}_rows.json'
        dump(rows_path, rows)
        terrain.update(rows=str(rows_path), rows_sha256=sha(rows_path))
        for sid in SPAWN_INDICES:
            local = sid * 5
            for group, ids in [('low', list(range(local, local+3))),
                               ('high', list(range(local+3, local+5)))]:
                template = template_jobs[group]
                n, name = len(ids), f'{level}_s{sid:02d}_{group}'
                selected = [dict(cells[i % 5], source_id=i,
                                 baseline_source_id=cells[i % 5]['source_id'],
                                 spawn_index=sid, cutoff_mm=cutoff, height_scale=height, surface=level,
                                 fixed_pair_angle_rms_deg=terrain['fixed_pair_angle_rms_deg']) for i in ids]
                plan = read(template['args']['experiment_plan'])
                plan.update(schema='libo_cutoff_height_shared_spawn30_v1', run_name=name,
                            rows_sha256=sha(rows_path), benchmark_source_ids=ids, selected_environments=ids,
                            pd_overrides={str(i): pd for i in ids}, planned_initial_pose=initials,
                            body_factors=[c['body_factor'] for c in selected],
                            leg_cluster_factors=[c['leg_factor'] for c in selected], joint_cells=selected,
                            quality_offsets_m=[[0., 0., 0.]] * n, episode1_permutation=list(range(n)),
                            worker_filename=RUNNER.name, campaign_note=AUTHORIZATION,
                            quality_protocol=dict(duration_s=6., only_focal_poses_recorded=False,
                                                  comparison='Same30 positions/headings, finite supplied Libo height/cutoff stage'))
                plan['scaling_benchmark']['replicas'] = n
                plan_path = SIM / f'{name}_plan.json'
                dump(plan_path, plan)
                job = copy.deepcopy(template)
                job.update(name=name, level=level, cutoff_mm=cutoff, height_scale=height,
                           spawn_index=sid, source_ids=ids, cells=selected, rows_sha256=sha(rows_path))
                job['args'].update(name=name, rows=str(rows_path), experiment_plan=str(plan_path),
                                   experiment_plan_sha256=sha(plan_path))
                jobs.append(job)
    batches = []
    lookup = {(j['level'], j['spawn_index'], j['group']): j['name'] for j in jobs}
    for pair in range(15):
        shift = pair % len(terrains)
        for terrain in terrains[shift:] + terrains[:shift]:
            names = [lookup[(terrain['level'], sid, group)]
                     for sid in (2*pair, 2*pair+1) for group in ('low', 'high')]
            batches.append(dict(name=f'pair{pair:02d}_{terrain["level"]}', jobs=names))
    pins = {**parent['pins'], **shared['pins']}
    sources = [RUNNER, ROOT / 'run_libo_repeats.py', ROOT / 'run_shortwave.py']
    for path in sources:
        shutil.copy2(path, snapshot / path.name)
    additional = [*sources, manifest_path, SIM / 'TERRAIN_MANIFEST.json',
                  PARENT / 'simulation/campaign.json', SHARED_PARENT / 'campaign.json',
                  *sorted(snapshot.iterdir()), *(Path(t['path']) for t in terrains)]
    pins.update({str(path): sha(path) for path in additional})
    campaign = copy.deepcopy(shared)
    campaign.update(schema='libo_cutoff_height_shared_spawn30_v1', stage=STAGE,
                    jobs=jobs, batches=batches, pins=pins, spawns=spawns, terrains=terrains,
                    waves=WAVES, height_scales=sorted({t['height_scale'] for t in terrains}),
                    cutoff_mm_levels=sorted({t['cutoff_mm'] for t in terrains}),
                    condition_count=len(terrains)*150, rough_condition_count=len(terrains)*150,
                    new_flat_condition_count=0, preserved_flat_controls=5,
                    shared_spawn_indices=SPAWN_INDICES, total_shared_spawns=30,
                    terrain_manifest_path=str(manifest_path), terrain_manifest_sha256=sha(manifest_path),
                    terrain_manifest_snapshot=str(SIM / 'TERRAIN_MANIFEST.json'),
                    parents=[str(PARENT / 'simulation/campaign.json'), str(SHARED_PARENT / 'campaign.json')],
                    authorization=AUTHORIZATION,
                    scope='Only the finite supplied terrain manifest; no implicit height continuation or other study',
                    spawn_policy='Exact30 shared spawn records copied from completed600 extension, no resampling',
                    boundary_envelope='Original flat movement envelope placements unchanged; original15mm all-step guard',
                    original_worker_function_source=str(snapshot / 'original_worker_function.py'),
                    scheduler_changes=['Worker subprocess CLI passes --stage', 'Completion caption uses supplied condition count'],
                    preselected_new_demo_spawn=12)
    # Do not inherit old extension-only accounting as if these were another20 spawns.
    for key in ('new_spawn_indices', 'appended_shared_spawns', 'old_conditions_preserved',
                'new_heading_rounds', 'heading_sector_width_deg'):
        campaign.pop(key, None)
    dump(SIM / 'campaign.json', campaign)
    check()
    print(f'PREPARED {len(terrains)*150}: {len(terrains)} terrains x5waves x30shared spawns; '
          f'{len(batches)} fixed4-worker batches', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['prepare', 'check', 'run', 'worker'])
    parser.add_argument('--stage', required=True)
    parser.add_argument('--terrains', help='Required only for prepare; JSON list or object with terrains list')
    parser.add_argument('--job')
    parser.add_argument('--attempt', default='v1')
    parser.add_argument('--keep')
    args = parser.parse_args()
    if args.mode == 'prepare':
        assert args.terrains, '--terrains is required for prepare'
        _, supplied_terrains = load_manifest(args.terrains)
        configure(args.stage, len(supplied_terrains)*150)
        prepare(args.terrains)
    else:
        # Validate stage before resolving or reading its campaign.
        configure(args.stage, 0)
        count = read(SIM / 'campaign.json')['condition_count']
        configure(args.stage, count)
        if args.mode == 'check':
            check()
        elif args.mode == 'run':
            original.run()
        else:
            assert args.job, '--job is required for worker'
            original.worker(args.job, args.attempt,
                            None if args.keep is None else list(map(int, args.keep.split(','))))
