import os
for name in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[name] = '1'

import hashlib
import json
import runpy
import sys
import time
from pathlib import Path
import psutil

psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
ROOT = Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
BASE = ROOT / 'libo_cutoff_height_expansion_2026-10-01'
OUT = BASE / 'interim_2026-10-01'
INPUTS = [ROOT / 'libo_spawn_repeats/extension_spawn30_2026-09-30/results/RESULTS.json',
          BASE / 'initial_4mm_h1p75_2p00/results/RESULTS.json']
RENDERER = ROOT / 'render_libo_cutoff_height_expansion.py'
RENDERER_SHA = '1b639be8f4763c72bea517067aee7bc5e66204f1fa0337e18704ca9617f29067'

def read(path):
    return json.loads(path.read_text(encoding='utf-8'))

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')

def progress():
    state = read(BASE / 'STATUS.json')
    supervisor = read(BASE / 'matched_2_8mm_h0p25_2p00/simulation/supervisor.json')
    live = []
    for pid in (state.get('pid'), state.get('active_pid')):
        try:
            process = psutil.Process(pid)
            live.append(dict(pid=pid, command=process.cmdline(), creation_unix=process.create_time()))
        except psutil.NoSuchProcess:
            live.append(dict(pid=pid, exited=True))
    return dict(unix=time.time(), status=state['status'], batch=supervisor.get('batch'),
                finished_worker_attempts=len(supervisor.get('finished', [])),
                complete=supervisor['complete'], live_orchestration=live)

assert not OUT.exists(), 'Preserve previous interim output'
assert sha(RENDERER) == RENDERER_SHA
source_hashes = {str(path): sha(path) for path in INPUTS}
reports = [read(path) for path in INPUTS]
assert all(report['complete'] for report in reports)
for field in ('fixed_pd_sha256', 'profile_sha256'):
    assert reports[0][field] == reports[1][field]
assert all(report['native_worker_bitexact_to_original'] for report in reports)
flat = reports[0]['flat_controls']
assert len(flat) == 5
for left, right in zip(sorted(flat, key=lambda c: c['wave']), sorted(reports[1]['flat_controls'], key=lambda c: c['wave'])):
    for field in ('wave', 'run_path', 'forward_0_6_mm', 'forward_2_6_mm', 'forward_4_6_mm', 'A_deg', 'B_deg'):
        assert left[field] == right[field], field

summaries = [cell for report in reports for cell in report['summaries']]
waves = [1.5, 2., 2.5, 3., 3.5]
heights = sorted({cell['height_scale'] for cell in summaries})
assert heights == [.25, .5, .75, 1., 1.25, 1.5, 1.75, 2.]
assert len(summaries) == 40
assert len({(c['height_scale'], c['wave']) for c in summaries}) == 40
assert all(c['n_planned'] == 30 and c['n_retained'] == c['n_complete_before'] - 1 for c in summaries)
compact = []
for cell in summaries:
    compact.append({key: cell[key] for key in ('height_scale', 'wave', 'fixed_pair_angle_rms_deg', 'n_planned', 'n_complete_before', 'n_failed', 'n_retained', 'excluded_spawn_index')} |
                   {key: {stat: cell[key][stat] for stat in ('n', 'mean', 'std')} for key in ('forward_0_6_mm', 'forward_2_6_mm', 'forward_4_6_mm')})

cells = [c for report in reports for c in report['cells'] if c['height_scale'] > 0 and c['status'] == 'complete']
assert len(cells) == 1098
assert len({(c['height_scale'], c['wave'], c['spawn_index']) for c in cells}) == len(cells)
paired = []
for height in heights:
    wave_cells = {wave: {c['spawn_index']: c for c in cells if c['height_scale'] == height and c['wave'] == wave} for wave in waves}
    for wave in (3., 3.5):
        shared = sorted(wave_cells[2.5].keys() & wave_cells[wave].keys())
        values = [wave_cells[wave][spawn]['forward_0_6_mm'] - wave_cells[2.5][spawn]['forward_0_6_mm'] for spawn in shared]
        import statistics
        paired.append(dict(height_scale=height, wave_a=2.5, wave_b=wave, n=len(values),
                           mean=statistics.mean(values), std=statistics.stdev(values),
                           positive_count=sum(v > 0 for v in values), spawn_indices=shared, values=values))

OUT.mkdir()
started = time.time()
receipt = dict(complete=False, interim=True, started_unix=started, pid=os.getpid(),
    user_authorization='2026-10-01: 一边后台跑着，你一边帮我看看目前结果怎么样，适当生成一些demo之类的',
    scope='Lightweight analysis of committed 4mm reports and serial previews of saved complete spawn12 trajectories; background native campaign continues.',
    new_physics_steps=0, priority='BelowNormal', numerical_library_threads=1,
    renderer_sha256=RENDERER_SHA, source_hashes=source_hashes, before=progress(),
    fixed_pd_sha256=reports[0]['fixed_pd_sha256'], profile_sha256=reports[0]['profile_sha256'],
    rough_attempts=1200, complete_rough=1098, incomplete_rough=102,
    retained_rough=1058, flat_controls=5, summaries=compact, paired_raw=paired,
    statistics='Each height × wave omits one complete 0–6s observation farthest from its median, as in committed reports; sample SD. Same retained set for other windows. Raw paired comparisons use all jointly complete spawns, with no omission.',
    limitations=['Means condition on complete numerical trajectories; failed conditions are not zero displacement or physical stalls.',
                 '2/8mm stage remains in progress and is not analyzed here.',
                 'Flat has one existing control per wave, without an estimated SD.',
                 'Preview spawn12 was preselected; failed cells are labeled and never substituted.',
                 'Serial low-priority rendering can modestly reduce concurrent throughput; no physics/resource setting is changed.'])
save(OUT / 'INTERIM_REPORT.json', receipt)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11, 'axes.spines.top': False,
                     'axes.spines.right': False, 'svg.fonttype': 'none'})
colors = ['#2479b8', '#e88929', '#269d73', '#bb536c', '#795caf']
rms = {height: next(c['fixed_pair_angle_rms_deg'] for c in summaries if c['height_scale'] == height) for height in heights}

fig, ax = plt.subplots(figsize=(12.6, 6.8))
for wave, color in zip(waves, colors):
    group = sorted([c for c in summaries if c['wave'] == wave], key=lambda c: c['height_scale'])
    x = [0.] + [c['fixed_pair_angle_rms_deg'] for c in group]
    y = [next(c['forward_0_6_mm'] for c in flat if c['wave'] == wave)] + [c['forward_0_6_mm']['mean'] for c in group]
    sd = [c['forward_0_6_mm']['std'] for c in group]
    ax.errorbar(x[1:], y[1:], yerr=sd, fmt='none', ecolor=color, alpha=.24, capsize=4, elinewidth=1.8)
    ax.plot(x, y, 'o-', color=color, label=f'Wave {wave:g}', linewidth=2.0, markersize=5.5)
ax.axhline(0, color='#687281', linewidth=.8)
ax.set(xlabel='Terrain roughness: fixed 38-foot / 703-pair slope-angle RMS (deg)',
       ylabel='Forward displacement, 0–6 s (mm)', xlim=(-.35, 16.5))
ax.grid(alpha=.17)
ax.legend(ncol=5, loc='upper right', frameon=False)
top = ax.secondary_xaxis('top')
top.set_xticks([0.] + [rms[h] for h in heights], ['0'] + [f'{h:g}' for h in heights])
top.set_xlabel('Height multiplier (same 4 mm cutoff and terrain phases)', labelpad=8)
fig.suptitle('Libo · 4 mm cutoff | 30 shared spawns per rough condition', y=.99, fontsize=17)
fig.text(.5, .035, 'Complete trajectories only · one observation omitted per rough group · pale error bars = sample SD\nFlat: one existing control per wave. Numerical / boundary failures are not assigned zero displacement.', ha='center', fontsize=10, color='#505c69')
fig.subplots_adjust(left=.085, right=.98, top=.82, bottom=.19)
for ext in ('png', 'svg'):
    fig.savefig(OUT / f'displacement_vs_roughness_4mm.{ext}', dpi=180)
plt.close(fig)

fig, axes = plt.subplots(1, 2, figsize=(13.8, 5.5))
for wave, color in [(3., colors[3]), (3.5, colors[4])]:
    group = [c for c in paired if c['wave_b'] == wave]
    x = [rms[c['height_scale']] for c in group]
    y = [c['mean'] for c in group]
    axes[0].errorbar(x, y, yerr=[c['std'] for c in group], fmt='none', ecolor=color, alpha=.23, capsize=4)
    axes[0].plot(x, y, 'o-', color=color, label=f'Wave {wave:g} − wave 2.5')
axes[0].axhline(0, color='#5c6672', lw=1)
axes[0].set(title='Same-spawn paired differences', ylabel='Forward displacement difference, 0–6 s (mm)')
axes[0].legend(frameon=False, fontsize=10, loc='upper left')
for wave, color in zip(waves, colors):
    group = sorted([c for c in summaries if c['wave'] == wave], key=lambda c: c['height_scale'])
    axes[1].plot([c['fixed_pair_angle_rms_deg'] for c in group], [100*c['n_failed']/30 for c in group], 'o-', color=color, label=f'{wave:g}')
axes[1].set(title='Incomplete numerical / boundary trials', ylabel='Incomplete fraction (% of 30)', ylim=(-2, 52))
axes[1].legend(ncol=5, title='Wave', frameon=False, fontsize=9, loc='upper left')
for ax in axes:
    ax.set_xlabel('Shared slope-angle RMS (deg)')
    ax.grid(alpha=.17)
fig.suptitle('Libo · 4 mm cutoff | Pairing and completion qualify the mean curves', fontsize=15)
fig.text(.5, .035, 'Left: all jointly complete raw spawns, without omission; pale bars = sample SD.  Right: failures remain separate from physical stalling.', ha='center', fontsize=9.5, color='#505c69')
fig.subplots_adjust(left=.065, right=.985, bottom=.19, top=.85, wspace=.24)
for ext in ('png', 'svg'):
    fig.savefig(OUT / f'paired_differences_and_incomplete_4mm.{ext}', dpi=180)
plt.close(fig)
print('Interim figures written', flush=True)

from vtkmodules.vtkCommonCore import vtkMultiThreader, vtkSMPTools
vtkMultiThreader.SetGlobalMaximumNumberOfThreads(1)
vtkMultiThreader.SetGlobalDefaultNumberOfThreads(1)
vtkSMPTools.Initialize(1)
for flags, name in [([], 'simulation_previews'), (['--video'], 'demos')]:
    sys.argv = [str(RENDERER), '--stage', 'initial_4mm_h1p75_2p00', '--out', str(OUT/name), *flags]
    runpy.run_path(str(RENDERER), run_name='__main__')
    print(name, 'finished', flush=True)

assert source_hashes == {str(path): sha(path) for path in INPUTS}
assert sha(RENDERER) == RENDERER_SHA
receipt.update(complete=True, finished_unix=time.time(), elapsed_s=time.time()-started,
               after=progress(), helper_sha256=sha(Path(__file__)), visual_review_pending=True)
save(OUT / 'INTERIM_REPORT.json', receipt)
print(json.dumps({'complete': True, 'output': str(OUT), 'elapsed_s': receipt['elapsed_s']}, ensure_ascii=False), flush=True)
