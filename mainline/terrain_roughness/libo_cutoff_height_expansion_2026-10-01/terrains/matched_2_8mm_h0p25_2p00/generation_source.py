"""Build matched-grid Libo cutoff/height surfaces and scientific previews.

Terrain preparation only: no Isaac import, physics launch, queue, retuning,
low-pass filtering, global RMS normalization or new random spectral phase.
Existing evidence is read-only. --out must identify a fresh directory.
"""
from pathlib import Path
import argparse
import ast
import hashlib
import json
import os
import shutil
import time

for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[key] = '1'

import numpy as np
from scipy.stats import qmc
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from PIL import Image

ROOT = Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
CONTROL = Path('X:/Data/Centipede_Terrain_Surface_2026-09-15/open_loop_review_2026-09-16/optimization_surfaces_v1/Libo-9-3-1-3_seed1501_512.npz')
DX = .0005
N = 2048
END = 2045
ORIGIN = -.511
METRICS = ROOT / 'shortwave/displacement_roughness/RESULTS.json'
EXPECTED = {
    CONTROL: 'caeede88cd6e4b2681090ae1d3112f75f81157646754642773b0ebd77d09e321',
    ROOT / 'shortwave/libo_restore4.npz': '4ac46df511d99a62ca42b17284a2c9c9809269c1f41dbf0784051ef7567ab7eb',
    ROOT / 'shortwave/libo_restore2.npz': '36b03fa0e7a0368b2332d8461183f9dae5d6827383ee7f6ef5bc4562da31bd3f',
    ROOT / 'shortwave_terrain.py': '767a5658d9e69c3d103e8eb6c19bac8db94e8cd2c550e0671fbaf39dbe0c94af',
    ROOT / 'workflow.py': '53efd2a6d91ee51be38f7a58ca479e61402716658b4f3c74092c67a6623abadd',
    ROOT / 'height_amplitude_preview.py': 'eeaa315f5bae45f73f66ecba8655c3789e02cbf132038fc5dcb7f9cfa2c5bccf',
    ROOT / 'shortwave/REPORT.json': 'ba1c02e844cbb8f945b42ddaa1fd7e5822a5eb6ee83da127baac8e3b13d0fb4f',
    METRICS: '336aa7ab316f4c2a24292229dc7b2e1134831b8f4d57ca064869d1be8b8f8497',
    ROOT / 'height_amplitude_4mm/REPORT.json': 'f414778dd5e0af218f5943f19575f89d97a46ac2b79db8040372412975d15b68',
    ROOT / 'preview/terrains.npz': 'e94d147215446c5b8bed936a95b80646770fa96b87ecf48884149438d4efa251',
}


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def frozen_functions():
    """Reuse exact recorded numerical methods without executing module imports."""
    namespace = dict(np=np, qmc=qmc, ROOT=ROOT, METRICS=METRICS, read=read,
                     N=N, END=END, DX=DX, ORIGIN=ORIGIN)
    selections = [
        (ROOT / 'workflow.py', ('surface_height',)),
        (ROOT / 'shortwave_terrain.py', ('exact_triangle_upsample',)),
        (ROOT / 'height_amplitude_preview.py', ('sampler', 'roughness')),
    ]
    receipt = []
    for path, names in selections:
        tree = ast.parse(path.read_text(encoding='utf-8'))
        for name in names:
            node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
            exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
            receipt.append(dict(path=str(path), source_sha256=sha(path), function=name,
                                ast_sha256=hashlib.sha256(ast.dump(node, include_attributes=False).encode()).hexdigest()))
    return namespace, receipt


def save_surface(path, height):
    assert not path.exists(), str(path)
    np.savez_compressed(path, height=height, dx_m=DX)
    with np.load(path) as z:
        np.testing.assert_array_equal(z['height'], height)
        assert float(z['dx_m']) == DX


def fine_control(out, namespace):
    with np.load(CONTROL) as z:
        old = z['height'].astype(float)
        assert old.shape == (512, 512) and float(z['dx_m']) == .002
    fine = namespace['exact_triangle_upsample'](old)[:END, :END].copy()
    np.testing.assert_array_equal(fine[::4, ::4], old)
    axis_old = (np.arange(512) - 255.5) * .002
    axis_fine = (np.arange(END) - (END - 1) / 2) * DX
    roi = np.asarray(read(METRICS)['fixed_roughness_sampling']['physics_roi_xy_m'])
    rng = np.random.default_rng(2026100108)
    xy = rng.uniform(roi[:, 0], roi[:, 1], (10000, 2))
    sample = namespace['surface_height']
    old_values = sample(old, axis_old, axis_old, *xy.T)
    fine_values = sample(fine, axis_fine, axis_fine, *xy.T)
    max_error = float(np.max(np.abs(old_values - fine_values)))
    assert max_error < 2e-15
    old_float = sample(old.astype(np.float32), axis_old.astype(np.float32), axis_old.astype(np.float32), *xy.T)
    fine_float = sample(fine.astype(np.float32), axis_fine.astype(np.float32), axis_fine.astype(np.float32), *xy.T)
    float_delta = old_float - fine_float
    mother_dir = out / 'mothers'
    mother_dir.mkdir()
    path = mother_dir / 'libo_cutoff8_triangle_refined_0p5mm.npz'
    save_surface(path, fine)
    receipt = dict(path=str(path), sha256=sha(path), source_control_path=str(CONTROL),
                   source_control_sha256=sha(CONTROL), original_nodes_exact=True,
                   subdivision_factor_per_axis=4, full_shape=[END, END], dx_m=DX,
                   continuous_triangle_surface_max_sample_error_m=max_error,
                   float32_vertices_max_sample_error_m=float(np.max(np.abs(float_delta))),
                   float32_vertices_rms_sample_error_m=float(np.sqrt(np.mean(float_delta**2))),
                   error_check_sample_count=len(xy), error_check_seed=2026100108,
                   error_check_roi_xy_m=roi.tolist(),
                   qualification='Intended continuous 9/17 triangle surface preserved to floating-point rounding. Float32 vertices and subdivided mesh/cooking are not bitexact old collision geometry; no dynamics identity claim. Sampled errors are not global bounds.',
                   no_filtering=True, no_RMS_normalization=True, new_spectral_phases=False)
    return fine, path, receipt


def previews(out, records, patches, cutoffs, heights):
    """One physical patch, viewpoint and Z/color limits for every panel."""
    center = [.17, -.12]
    a = np.linspace(-.05, .05, 201)
    xx, yy = np.meshgrid(a * 1000, a * 1000)
    limit = max(.5, float(np.ceil(max(np.abs(p).max() for p in patches.values()) * 1000 * 2) / 2))
    norm = Normalize(-limit, limit)
    width = max(12., 3.35 * len(heights))
    fig = plt.figure(figsize=(width, 3.65 * len(cutoffs) + 1.35))
    fig.subplots_adjust(left=.015, right=.90, bottom=.16, top=.80, wspace=.06, hspace=.28)
    lookup = {(r['cutoff_wavelength_nominal_mm'], r['height_scale']): r for r in records}
    for row, cutoff in enumerate(cutoffs):
        for col, height in enumerate(heights):
            ax = fig.add_subplot(len(cutoffs), len(heights), row * len(heights) + col + 1, projection='3d')
            patch = patches[(cutoff, height)]
            ax.plot_surface(xx, yy, patch * 1000, cmap='terrain', norm=norm,
                            rcount=201, ccount=201, linewidth=0, antialiased=True)
            ax.view_init(elev=29, azim=-125)
            ax.set(xlim=(-50, 50), ylim=(-50, 50), zlim=(-limit, limit),
                   xlabel='X (mm)', ylabel='Y (mm)', zlabel='Height (mm)')
            ax.set_box_aspect((100, 100, 2 * limit * 3), zoom=.86)
            r = lookup[(cutoff, height)]
            ax.set_title(f"{cutoff} mm cutoff | height x{height:g}\nR = {r['fixed_pair_angle_rms_deg']:.2f} deg", fontsize=10.5)
            ax.tick_params(labelsize=7, pad=0)
    cax = fig.add_axes([.925, .24, .012, .44])
    fig.colorbar(ScalarMappable(norm=norm, cmap='terrain'), cax=cax, label='Height (mm)')
    fig.suptitle('Libo: matched-grid cutoff and terrain-height surfaces', fontsize=16, weight='bold', y=.975)
    fig.text(.5, .89, 'Same 100 x 100 mm patch at (170, -120) mm | 0.5 mm grid\nFixed camera and common height/color limits for every panel', ha='center', fontsize=10.5)
    fig.text(.025, .028, 'Z display exaggerated 3x. R uses the shared full simulation ROI and all headings.\nCutoff labels identify nominal wavelength bands; they are not roughness values.', fontsize=9.5)
    path = out / 'libo_cutoff_height_preview.png'
    fig.savefig(path, dpi=140, facecolor='white')
    plt.close(fig)
    with Image.open(path) as image:
        image.load()
    receipt = dict(path=str(path), sha256=sha(path), patch_center_xy_m=center,
                   patch_width_m=.1, display_grid_m=DX, z_limits_mm=[-limit, limit],
                   camera=dict(elevation_deg=29, azimuth_deg=-125), vertical_display_exaggeration=3,
                   rows_cutoff_mm=cutoffs, columns_height_scales=heights, image_decoded=True,
                   manual_visual_review=False)
    fig, axes = plt.subplots(1, 2, figsize=(11.8, 5), layout='constrained')
    colors = {2: '#247c98', 4: '#d48924', 8: '#536b80'}
    for cutoff in cutoffs:
        rows = [lookup[(cutoff, s)] for s in heights]
        axes[0].plot(heights, [r['fixed_pair_angle_rms_deg'] for r in rows], '-o', color=colors[cutoff], label=f'{cutoff} mm')
        axes[1].plot(heights, [r['height_Sq_mm'] for r in rows], '-o', color=colors[cutoff], label=f'{cutoff} mm')
    for ax in axes:
        ax.set_xlabel('Terrain height multiplier')
        ax.set_xticks(heights)
        ax.grid(alpha=.2)
        ax.legend(title='Nominal cutoff', frameon=False)
    axes[0].set_ylabel('Fixed-foot pair-angle RMS (deg)')
    axes[0].set_title('38 neutral feet / 703 pairs / shared positions and headings')
    axes[1].set_ylabel('Height RMS about mean (mm)')
    axes[1].set_title('Saved full surface; no common-RMS normalization')
    fig.suptitle('Libo terrain inputs only | same 0.5 mm grid | no robot simulation')
    stats_path = out / 'roughness_vs_height_cutoff.png'
    fig.savefig(stats_path, dpi=160, facecolor='white')
    plt.close(fig)
    with Image.open(stats_path) as image:
        image.load()
    return [receipt, dict(path=str(stats_path), sha256=sha(stats_path), image_decoded=True, manual_visual_review=False)]


def generate(cutoffs, heights, out):
    start = time.time()
    out = out.resolve()
    assert not out.exists(), f'Preserve existing evidence; output already exists: {out}'
    assert cutoffs == sorted(set(cutoffs)) and set(cutoffs) <= {2, 4, 8}
    assert heights == sorted(set(heights)) and heights and min(heights) > 0
    pins = {str(p): sha(p) for p in EXPECTED}
    for p, expected in EXPECTED.items():
        assert pins[str(p)] == expected, f'Input changed: {p}'
    out.mkdir(parents=True)
    namespace, function_receipt = frozen_functions()
    points, i, j, distances, method, foot = namespace['sampler']()
    assert points.shape == (65536, 38, 2) and len(distances) == 703
    axis = (np.arange(END) - (END - 1) / 2) * DX
    collision_axis = axis.astype(np.float32).astype(float)
    a = np.linspace(-.05, .05, 201)
    xx, yy = np.meshgrid(a + .17, a - .12)
    existing4 = {r['height_scale']: r for r in read(ROOT / 'height_amplitude_4mm/REPORT.json')['terrains'] if r['site'] == 'libo'}
    records = []
    patches = {}
    mothers = []
    fine_receipt = None
    for cutoff in cutoffs:
        if cutoff == 8:
            h0, mother_path, fine_receipt = fine_control(out, namespace)
            mother_receipt = fine_receipt.copy()
        else:
            mother_path = ROOT / f'shortwave/libo_restore{cutoff}.npz'
            with np.load(mother_path) as z:
                h0 = z['height'].astype(float)
                assert float(z['dx_m']) == DX
            mother_receipt = dict(path=str(mother_path), sha256=sha(mother_path), existing_mother_reused=True,
                                  source_control_path=str(CONTROL), source_control_sha256=sha(CONTROL),
                                  spectral_construction_report=str(ROOT / 'shortwave/REPORT.json'),
                                  no_filtering=True, no_RMS_normalization=True, new_spectral_phases=False)
        assert h0.shape == (END, END) and np.isfinite(h0).all()
        mean = float(h0.mean())
        centered = h0 - mean
        sd0 = float(centered.std())
        mother_receipt.update(cutoff_wavelength_nominal_mm=cutoff, grid_spacing_m=DX,
                              shape=list(h0.shape), mean_height_mm=mean * 1000,
                              height_Sq_mm=sd0 * 1000, physical_span_m=[float(axis[0]), float(axis[-1])])
        mothers.append(mother_receipt)
        for scale in heights:
            h = h0.copy() if scale == 1 else mean + scale * centered
            reused = None
            if scale == 1:
                path = mother_path
                reused = 'unmodified_mother'
            elif cutoff == 4 and scale in existing4:
                candidate = existing4[scale]
                old_path = Path(candidate['path'])
                assert sha(old_path) == candidate['sha256']
                with np.load(old_path) as z:
                    equal = float(z['dx_m']) == DX and np.array_equal(z['height'], h)
                if equal:
                    path = old_path
                    reused = 'existing_height_surface_array_exact'
                    pins[str(old_path)] = candidate['sha256']
            if reused is None:
                tag = f'{scale:.6f}'.rstrip('0').rstrip('.').replace('.', 'p')
                path = out / f'libo_cutoff{cutoff}_height_x{tag}.npz'
                save_surface(path, h)
            np.testing.assert_allclose(h.mean(), mean, atol=1e-16, rtol=0)
            np.testing.assert_allclose(h.std(), scale * sd0, atol=1e-15, rtol=1e-13)
            r = namespace['roughness'](h, collision_axis, points, i, j, distances)
            if cutoff in (2, 4) and scale == 1:
                reference = next(v['fixed_pair_angle_rms_deg'] for v in read(METRICS)['roughness'] if v['site'] == 'libo' and v['level'] == f'restore{cutoff}')
                assert abs(r - reference) < 1e-10
            if cutoff == 4 and scale in existing4:
                assert abs(r - existing4[scale]['fixed_pair_angle_rms_deg']) < 1e-10
            if cutoff == 8 and scale == 1:
                assert abs(r - 8.571256213884043) < 1e-6
            row = dict(site='libo', cutoff_mm=cutoff, cutoff_wavelength_nominal_mm=cutoff, height_scale=scale,
                       path=str(path), sha256=sha(path), reused_existing_surface=reused,
                       mother_surface_path=str(mother_path), mother_surface_sha256=sha(mother_path),
                       source_control_path=str(CONTROL), source_control_sha256=sha(CONTROL),
                       shape=list(h.shape), grid_spacing_m=DX, physical_span_m=[float(axis[0]), float(axis[-1])],
                       mean_height_mm=float(h.mean() * 1000), height_Sq_mm=float(h.std() * 1000),
                       min_height_mm=float(h.min() * 1000), max_height_mm=float(h.max() * 1000),
                       fixed_pair_angle_rms_deg=r)
            records.append(row)
            patches[(cutoff, scale)] = namespace['surface_height'](h, axis, axis, xx, yy)
            print(f'cutoff {cutoff} mm height x{scale:g}: R={r:.8f} deg, Sq={row["height_Sq_mm"]:.8f} mm; reuse={reused}', flush=True)
        cutoff_r = [r['fixed_pair_angle_rms_deg'] for r in records if r['cutoff_wavelength_nominal_mm'] == cutoff]
        assert np.all(np.diff(cutoff_r) > 0)
    figures = previews(out, records, patches, cutoffs, heights)
    for path, expected in pins.items():
        assert sha(path) == expected, f'Input changed during preparation: {path}'
    source_copy = out / 'generation_source.py'
    shutil.copyfile(Path(__file__).resolve(), source_copy)
    report = dict(schema='libo_matched_cutoff_height_terrain_preparation_v1', complete=True,
                  cutoffs_nominal_mm=cutoffs, height_scales=heights, grid_spacing_m=DX,
                  height_transform='h_s = mean(h_0) + s * (h_0 - mean(h_0)); s=1 directly reuses the mother file',
                  preserved='Mother XY layout, phases, coordinate span, and normalized non-DC spectral shape; non-DC PSD scales by s^2. No common-RMS renormalization.',
                  cutoff_note='2/4 mm labels describe measured-spectrum restoration increment; original piecewise-planar high-frequency tails retained. Fine8 is exact triangle refinement of the old nominal8 control, not a new lowpass surface.',
                  fidelity_status='Measured-spectrum constrained synthetic surfaces; no new natural morphology/noise-floor validation or dynamics identity claim.',
                  roughness_sampling=method, frozen_function_receipt=function_receipt,
                  mothers=mothers, fine8_refinement=fine_receipt, terrains=records, figures=figures,
                  input_hashes=pins, executed_source_path=str(source_copy), executed_source_sha256=sha(source_copy),
                  checks=dict(inputs_unchanged=True, saved_arrays_exact=True, mean_height_preserved=True,
                              standard_deviation_scales_exactly=True, historical_4mm_R_matches=True,
                              shared_38_foot_703_pair_sampling=True, roughness_increases_with_scale=True,
                              no_overwrites=True, images_decoded=True),
                  preparation_only=True, new_physics_steps=0, no_runtime_or_queue_created=True,
                  generation_elapsed_s=time.time() - start)
    terrain_manifest = dict(schema='libo_matched_cutoff_height_terrain_manifest_v1', complete=True,
                            cutoffs_nominal_mm=cutoffs, height_scales=heights, terrains=records,
                            report_path=str(out / 'REPORT.json'), report_sha256=None,
                            preparation_only=True, new_physics_steps=0)
    dump(out / 'REPORT.json', report)
    terrain_manifest['report_sha256'] = sha(out / 'REPORT.json')
    dump(out / 'TERRAINS.json', terrain_manifest)
    print(str(out / 'REPORT.json'), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cutoffs', nargs='+', required=True, type=int, choices=[2, 4, 8])
    parser.add_argument('--heights', nargs='+', required=True, type=float)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    generate(sorted(set(args.cutoffs)), sorted(set(args.heights)), args.out)


if __name__ == '__main__':
    main()
