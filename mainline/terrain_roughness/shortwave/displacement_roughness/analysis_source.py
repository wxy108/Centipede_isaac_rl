"""Read completed six-second trajectories; fixed-foot roughness, no simulation."""
from pathlib import Path
import ast
import hashlib
import json
import os
import shutil

for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[key] = '1'

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import qmc
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from PIL import Image
import workflow

HERE = Path(__file__).resolve().parent
SW = HERE / 'shortwave'
OUT = SW / 'displacement_roughness'
WAVES = [1.5, 2, 2.5, 3, 3.5, 4]
SITES = ['libo', 'narvik']
LEVELS = ['original', 'restore4', 'restore2']
LEVEL_LABELS = ['Original', '4 mm', '2 mm']
COLORS = ['#2474b5', '#df7c13', '#26915a', '#bd4545', '#8862b2', '#765441']
MARKERS = ['o', 's', '^']


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, obj):
    Path(path).write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def extract(verification, sources):
    cells, time_series = [], {}
    verified = {r['job']: r for r in verification['runs']}
    shared_local_com = shared_names = shared_masses = None
    max_rotation_check = max_origin_com_difference = 0.0
    for site in SITES:
        for level in LEVELS:
            for group in ['low', 'high']:
                name = f'{site}_{level}_{group}'
                root = SW / 'simulation_v2/runs/v2' / name
                paths = {s: root / s for s in ['trajectory.npz', 'backend.npz', 'effective_fields_after_pd.json', 'amplitude_modulation.json']}
                for path in paths.values():
                    sources[str(path)] = sha(path)
                assert sources[str(paths['trajectory.npz'])] == verified[name]['trajectory_sha256']
                with np.load(paths['trajectory.npz']) as z:
                    pose, t, names = z['pose'].astype(float), z['t'], z['body_names'].tolist()
                with np.load(paths['backend.npz']) as z:
                    masses = z['masses'].astype(float)
                assert pose.shape == (601, 3, 251, 7) and np.isfinite(pose).all()
                np.testing.assert_allclose(t, np.arange(601) / 100, atol=1e-12, rtol=0)
                assert np.all(masses > 0)
                fields = read(paths['effective_fields_after_pd.json'])
                com_fields = {a['path'].rsplit('/', 1)[-1]: a for a in fields['attributes'] if a['name'] == 'physics:centerOfMass'}
                assert set(com_fields) == set(names) and all(a['authored'] for a in com_fields.values())
                local = np.array([ast.literal_eval(com_fields[n]['value']) for n in names], float)
                assert np.isfinite(local).all()
                if shared_names is None:
                    shared_names, shared_local_com, shared_masses = names, local, masses[0]
                else:
                    assert names == shared_names
                    np.testing.assert_array_equal(local, shared_local_com)
                np.testing.assert_array_equal(masses, np.broadcast_to(shared_masses, masses.shape))
                # Saved body_link_pose_w uses wxyz. Transform the authored local COM,
                # then mass-average all 251 rigid bodies with native PhysX masses.
                quat = pose[..., 3:]
                qnorm = np.linalg.norm(quat, axis=-1, keepdims=True)
                assert np.max(abs(qnorm - 1)) < 2e-6
                quat = quat / qnorm
                cross = np.cross(quat[..., 1:], local)
                rotated = local + 2 * (quat[..., :1] * cross + np.cross(quat[..., 1:], cross))
                # Independent library check on beginning/middle/end transforms.
                subset = quat[[0, 300, 600]]
                rcheck = Rotation.from_quat(subset[..., [1, 2, 3, 0]].reshape(-1, 4)).apply(np.broadcast_to(local, subset.shape[:-1] + (3,)).reshape(-1, 3))
                rotation_error = np.max(abs(rcheck - rotated[[0, 300, 600]].reshape(-1, 3)))
                max_rotation_check = max(max_rotation_check, float(rotation_error))
                assert rotation_error < 1e-14
                com = ((pose[..., :3] + rotated) * masses[None, :, :, None]).sum(2) / masses.sum(1)[None, :, None]
                origins = (pose[..., :3] * masses[None, :, :, None]).sum(2) / masses.sum(1)[None, :, None]
                direction = pose[0, :, names.index('link_body_0'), :2] - pose[0, :, names.index('link_body_20'), :2]
                direction /= np.linalg.norm(direction, axis=-1, keepdims=True)
                np.testing.assert_allclose(direction, np.tile([-1, 0], (3, 1)), atol=2e-6, rtol=0)
                fwd = ((com[..., :2] - com[0, :, :2]) * direction[None]).sum(2) * 1000
                origin_fwd = ((origins[..., :2] - origins[0, :, :2]) * direction[None]).sum(2) * 1000
                max_origin_com_difference = max(max_origin_com_difference, float(np.max(abs(fwd - origin_fwd))))
                lateral = np.stack([-direction[:, 1], direction[:, 0]], -1)
                side = ((com[..., :2] - com[0, :, :2]) * lateral[None]).sum(2) * 1000
                amplitudes = read(paths['amplitude_modulation.json'])['cells']
                for e, c in enumerate(amplitudes):
                    cell = dict(site=site, level=level, wave=c['wave'], run=name, environment_index=e,
                                A_deg=c['A_deg'], B_deg=c['B_deg'],
                                forward_axis_xy=direction[e].tolist(),
                                forward_0_6_mm=float(fwd[-1, e]),
                                forward_4_6_mm=float(fwd[-1, e] - fwd[400, e]),
                                lateral_0_6_mm=float(side[-1, e]),
                                com_delta_xyz_mm=((com[-1, e] - com[0, e]) * 1000).tolist(),
                                com_path_xy_0_6_mm=float(np.linalg.norm(np.diff(com[:, e, :2], axis=0), axis=-1).sum() * 1000))
                    cells.append(cell)
                    time_series[f'{site}_{level}_w{c["wave"]:g}'] = fwd[:, e]
    assert len(cells) == 36
    for cell in cells:
        ref = next(c for c in cells if c['site'] == cell['site'] and c['wave'] == cell['wave'] and c['level'] == 'original')
        cell['change_from_original_0_6_mm'] = cell['forward_0_6_mm'] - ref['forward_0_6_mm']
        cell['change_from_original_4_6_mm'] = cell['forward_4_6_mm'] - ref['forward_4_6_mm']
    return cells, time_series, dict(rotation_library_check_max_error_m=max_rotation_check,
                                  max_difference_from_mass_weighted_link_origin_forward_mm=max_origin_com_difference,
                                  masses_identical_all_conditions=True, local_COM_identical_all_conditions=True,
                                  total_mass_kg=float(shared_masses.sum()), forward_from_initial_tail_to_head=True)


def roughness(campaign, sources):
    template_path = HERE / 'preview/terrains.npz'
    sources[str(template_path)] = sha(template_path)
    with np.load(template_path) as z:
        foot = z['fixed_foot_xy_m'].astype(float)
    assert foot.shape == (38, 2)
    roi = np.array(campaign['terrain_roi_m'], float).reshape(2, 2)
    # Erode by the maximum template radius so every orientation uses exactly
    # the same center domain and every one of the 38 feet lies in the mesh ROI.
    radius = float(np.linalg.norm(foot, axis=-1).max())
    domain = roi + [radius, -radius]
    power, seed = 16, 918
    u = qmc.Sobol(3, scramble=True, seed=seed).random_base2(power)
    centers = domain[:, 0] + u[:, :2] * (domain[:, 1] - domain[:, 0])
    theta = u[:, 2] * 2 * np.pi
    co, si = np.cos(theta)[:, None], np.sin(theta)[:, None]
    rotated = np.stack([co * foot[:, 0] - si * foot[:, 1], si * foot[:, 0] + co * foot[:, 1]], -1)
    points = rotated + centers[:, None, :]
    assert np.all((points >= roi[:, 0]) & (points <= roi[:, 1]))
    terrain_report = read(SW / 'REPORT.json')
    pair_i, pair_j = np.triu_indices(38, 1)
    distances = np.linalg.norm(foot[pair_i] - foot[pair_j], axis=-1)
    assert len(distances) == 703 and distances.min() > 0
    output = []
    for site in SITES:
        for level in LEVELS:
            surface = terrain_report['sites'][site]['levels'][level]
            path = Path(surface['path'])
            sources[str(path)] = sha(path)
            assert sources[str(path)] == surface['sha256']
            with np.load(path) as z:
                # Match the float32 collision-mesh vertices; no extra height
                # scaling, detrending, smoothing, wrapping, or extrapolation.
                h = z['height'].astype(np.float32).astype(float)
                dx = float(z['dx_m'])
            ax = ((np.arange(h.shape[1]) - (h.shape[1] - 1) / 2) * dx).astype(np.float32).astype(float)
            ay = ((np.arange(h.shape[0]) - (h.shape[0] - 1) / 2) * dx).astype(np.float32).astype(float)
            heights = workflow.surface_height(h, ax, ay, points[..., 0], points[..., 1])
            angle_m2_sum = 0.0
            for start in range(0, len(u), 2048):
                hh = heights[start:start+2048]
                beta = np.arctan2(abs(hh[:, pair_i] - hh[:, pair_j]), distances)
                angle_m2_sum += float(np.mean(beta * beta, axis=-1).sum())
            value = float(np.rad2deg(np.sqrt(angle_m2_sum / len(u))))
            output.append(dict(site=site, level=level, fixed_pair_angle_rms_deg=value, surface_path=str(path), surface_sha256=surface['sha256']))
            print(f'{site} {level}: fixed R = {value:.8f} deg', flush=True)
    method = dict(foot_count=38, unordered_pair_count=703, foot_template_path=str(template_path),
                  template_key='fixed_foot_xy_m', coordinate='Neutral common-model foot sphere center XY, centered on 21-spine-body centroid',
                  formula='R_deg = degrees(sqrt(mean_placements(mean_unordered_pairs(atan2(abs(f(x_i,y_i)-f(x_j,y_j)), distance_xy_ij)^2))))',
                  user_selection='Uniform positions in simulation terrain region and all headings; same metric for all wave numbers',
                  physics_roi_xy_m=roi.tolist(), template_max_radius_m=radius, common_center_domain_xy_m=domain.tolist(),
                  heading_range_rad=[0, float(2*np.pi)], sampling='Shared scrambled Sobol samples in x, y, heading; no gait phase',
                  sobol_power=power, sobol_seed=seed, placement_count=len(u),
                  computational_note='One fixed sample budget, reused historical 2^16 and seed 918; no convergence or capacity search',
                  interpolation='Original collision-mesh triangle diagonal (x0,y0)-(x1,y1), float32 vertex coordinates',
                  foot_own_z_used=False, per_gait_geometry_used=False, actual_contact_filter_used=False,
                  averaging='Mean squared pair angles then square root, not mean of per-placement RMS',
                  spatial_samples_are_independent_trials=False)
    return output, method


def figures(cells, rough):
    lookup = {(c['site'], c['level'], c['wave']): c for c in cells}
    rlookup = {(r['site'], r['level']): r['fixed_pair_angle_rms_deg'] for r in rough}
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11, 'axes.spines.top': False,
                         'axes.spines.right': False, 'axes.titleweight': 'bold', 'axes.linewidth': .8,
                         'savefig.facecolor': 'white'})
    specs = [
        ('forward_0_6_mm', 'forward_0_6.png', 'Forward displacement vs. terrain roughness', 'Forward displacement, 0-6 s (mm)',
         'Whole-body center of mass projected onto the initial forward direction (-X).'),
        ('change_from_original_0_6_mm', 'change_from_original.png', 'Change in displacement from the original terrain', 'Change in 0-6 s forward displacement (mm)',
         'Each wave uses its own original-terrain displacement as the zero reference.'),
        ('forward_4_6_mm', 'forward_4_6.png', 'Forward displacement during the final 2 seconds', 'Forward displacement, 4-6 s (mm)',
         'Same saved 6 s runs; the 4-6 s window is a startup check, not proof of steady state.'),
    ]
    paths = []
    for key, filename, title, ylabel, note in specs:
        fig, axs = plt.subplots(1, 2, figsize=(12.8, 6.9), sharey=True)
        fig.subplots_adjust(left=.09, right=.98, bottom=.235, top=.75, wspace=.13)
        fig.suptitle(title, y=.972, fontsize=18, weight='bold')
        handles = [Line2D([0], [0], color=color, lw=2.3, label=f'Wave {wave:g}') for wave, color in zip(WAVES, COLORS)]
        fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(.54, .917), ncol=6, frameon=False,
                   columnspacing=1.45, handlelength=2)
        for ax, site in zip(axs, SITES):
            xs = np.array([rlookup[(site, level)] for level in LEVELS])
            for wave, color in zip(WAVES, COLORS):
                ys = [lookup[(site, level, wave)][key] for level in LEVELS]
                ax.plot(xs, ys, color=color, lw=2.0, alpha=.95)
                for x, y, marker in zip(xs, ys, MARKERS):
                    ax.scatter(x, y, c=color, marker=marker, s=56, edgecolors='white', linewidth=.8, zorder=4)
            ax.set_title(site.capitalize(), pad=11, fontsize=14)
            ax.grid(axis='y', alpha=.19)
            ax.grid(axis='x', alpha=.12)
            ax.set_xticks(xs, [f'{x:.4f}\n{label}' for x, label in zip(xs, LEVEL_LABELS)], fontsize=10)
            ax.set_xlabel('Fixed-foot pair-angle RMS (deg)', labelpad=9)
            dx = xs.max() - xs.min()
            ax.set_xlim(xs.min() - .09*dx, xs.max() + .09*dx)
            if key.startswith('change'):
                ax.axhline(0, color='#666666', lw=.9, ls='--', zorder=0)
            else:
                ax.set_ylim(bottom=0)
        axs[0].set_ylabel(ylabel, labelpad=10)
        fig.text(.09, .105, note, fontsize=10.5, color='#303846')
        fig.text(.09, .068, '38 neutral feet / 703 pairs; shared position + all-heading average. 4 / 2 mm label restored wavelength bands.', fontsize=10, color='#505763')
        fig.text(.09, .032, 'One saved run per condition. Flat-cluster amplitudes and Sep 17 wave-4 PD fixed; lines connect observations.', fontsize=10, color='#505763')
        path = OUT / filename
        fig.savefig(path, dpi=160)
        plt.close(fig)
        with Image.open(path) as im:
            im.load()
            assert im.size == (2048, 1104)
        paths.append(dict(path=str(path), sha256=sha(path), width=2048, height=1104))
    return paths


def main():
    if OUT.exists():
        raise RuntimeError(f'Existing analysis output must not be overwritten: {OUT}')
    verification = read(SW / 'SIMULATION_VERIFICATION.json')
    campaign = read(SW / 'simulation_v2/campaign.json')
    assert verification['complete'] and verification['condition_count'] == 36
    assert all(verification['checks'].values())
    paths = [Path(__file__), HERE/'workflow.py', SW/'SIMULATION_VERIFICATION.json', SW/'simulation_v2/campaign.json', SW/'REPORT.json']
    sources = {str(p): sha(p) for p in paths}
    tests = workflow.checks()
    cells, series, com_checks = extract(verification, sources)
    rough, method = roughness(campaign, sources)
    OUT.mkdir()
    shutil.copy2(Path(__file__), OUT / 'analysis_source.py')
    np.savez_compressed(OUT/'forward_timeseries.npz', t_s=np.arange(601)/100, **series)
    plots = figures(cells, rough)
    rlookup = {(r['site'], r['level']): r['fixed_pair_angle_rms_deg'] for r in rough}
    for c in cells:
        c['fixed_pair_angle_rms_deg'] = rlookup[(c['site'], c['level'])]
    assert all(sha(p) == digest for p, digest in sources.items())
    report = dict(complete=True, scope='Descriptive postprocessing of completed simulation_v2/runs/v2, no new physics or optimization',
                  fixed_roughness_sampling=method,
                  displacement_definition=dict(reference='Whole-body COM from native masses and authored local COMs transformed by saved rigid-body link poses',
                                               local_com_evidence='effective_fields_after_pd.json records all 251 authored physics:centerOfMass values; native COM tensor not separately recorded',
                                               frame='Terrain-local world frame; environment offsets were already subtracted by simulation.py',
                                               direction='Initial XY tail link_body_20 to head link_body_0 unit vector, effectively -X; held fixed in time',
                                               total_window_s=[0, 6], later_window_s=[4, 6],
                                               current_heading_integral=False, path_length=False, common_duration_all_conditions=True),
                  roughness=rough, cells=cells, checks=dict(analytic_metric_checks=tests, **com_checks, source_hashes_unchanged=True, images_decoded=True),
                  source_hashes=sources, plots=plots,
                  limitations=['One seed and one start per condition; no independent replicates or statistical error bars.',
                               'Whole-region roughness is a common descriptor, not the exact terrain encountered by each trajectory.',
                               'Shortwave restoration changes more than this single scalar; line segments do not establish a general causal roughness law.',
                               'The 0-6 s window includes settling; terrain-dependent initial height uses the same original initialization rule.',
                               '4-6 s is a supplementary fixed window, not a validated steady-state interval.',
                               'This computes only the fixed-neutral geometry descriptor on six completed surfaces, not the pending per-gait sweep.'])
    write(OUT/'RESULTS.json', report)
    for site in SITES:
        print(site, flush=True)
        for wave in WAVES:
            row = [next(c for c in cells if (c['site'],c['level'],c['wave']) == (site, level, wave)) for level in LEVELS]
            print(f"  wave {wave:g}: " + ' / '.join(f"{c['forward_0_6_mm']:.3f}" for c in row) + f" mm; 2mm-original {row[-1]['change_from_original_0_6_mm']:+.3f} mm", flush=True)
    print(json.dumps(dict(complete=True, plots=plots, checks=report['checks']), indent=2), flush=True)


if __name__ == '__main__':
    main()
