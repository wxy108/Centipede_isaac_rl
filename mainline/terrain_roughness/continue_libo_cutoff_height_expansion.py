"""Sequential, checkpointed orchestration of the user-started Libo expansion.

Native physics stays in the unchanged worker. This controller makes no parameter
search, retries no failed numerical condition, and never resumes contact erasure.
"""
from pathlib import Path
import argparse, hashlib, json, os, subprocess, sys, time, traceback

ROOT = Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
BASE = ROOT / 'libo_cutoff_height_expansion_2026-10-01'
PYTHON = Path(sys.executable)
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[key] = '1'
os.environ['PXR_WORK_THREAD_LIMIT'] = '2'
os.environ['PYTHONUTF8'] = '1'

def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def dump(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    temporary.replace(path)

def execute(script, arguments, state, label):
    for path, digest in read(BASE/'PROTOCOL.json')['orchestration_sources'].items():
        assert sha(path) == digest, path
    command = [str(PYTHON), '-X', 'utf8', '-B', str(ROOT/script), *map(str, arguments)]
    log_path = BASE/'logs'/f'{label}.log'
    log_path.parent.mkdir(exist_ok=True)
    with log_path.open('ab') as log:
        process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT,
                                   creationflags=subprocess.CREATE_NO_WINDOW)
        state.update(active_action=label, active_pid=process.pid,
                     active_command=command, updated_unix=time.time())
        dump(BASE/'STATUS.json', state)
        if script == 'run_libo_cutoff_height_expansion.py' and arguments[0] == 'run':
            stage = arguments[arguments.index('--stage')+1]
            simulation = BASE/stage/'simulation'
            campaign = read(simulation/'campaign.json')
            dump(simulation/'DISPATCH.json', dict(pid=process.pid, status='started', stage=stage,
                 runner=str(ROOT/script), runner_sha256=sha(ROOT/script), command=command,
                 started_unix=time.time(), condition_count=campaign['condition_count'],
                 scope=campaign['scope'], controller_pid=os.getpid()))
        returncode = process.wait()
    state.pop('active_pid', None)
    state.update(active_returncode=returncode, updated_unix=time.time())
    dump(BASE/'STATUS.json', state)
    if returncode:
        raise RuntimeError(f'{label} returned {returncode}; retained log {log_path}')

def perform_stage(name, cutoffs, heights, state, allow_resume):
    terrain_directory = BASE/'terrains'/name
    manifest = terrain_directory/'TERRAINS.json'
    simulation = BASE/name/'simulation'
    if not manifest.exists():
        assert not terrain_directory.exists(), 'Partly generated terrain requires inspection'
        execute('build_libo_cutoff_height_terrains.py',
                ['--cutoffs', *cutoffs, '--heights', *heights, '--out', terrain_directory],
                state, f'{name}_terrain')
    if not (simulation/'campaign.json').exists():
        assert not (BASE/name).exists(), 'Partly prepared stage requires inspection'
        execute('run_libo_cutoff_height_expansion.py',
                ['prepare', '--stage', name, '--terrains', manifest], state, f'{name}_prepare')
    assert read(simulation/'PREPARATION_CHECK.json')['passed']
    supervisor = read(simulation/'supervisor.json') if (simulation/'supervisor.json').exists() else {}
    if not supervisor.get('complete'):
        if (simulation/'runs/v1').exists():
            assert allow_resume, 'Existing incomplete dispatch requires owned-process reconciliation'
            execute('resume_libo_cutoff_height_expansion.py', ['--stage', name], state, f'{name}_resume')
        else:
            execute('run_libo_cutoff_height_expansion.py', ['run', '--stage', name], state, f'{name}_run')
    supervisor = read(simulation/'supervisor.json')
    assert supervisor['complete'] and supervisor['all_attempts_accounted_for']
    result_path = BASE/name/'results/RESULTS.json'
    if not result_path.exists():
        execute('analyze_libo_cutoff_height_expansion.py',
                ['stage', '--stage', name, '--out', result_path.parent], state, f'{name}_analysis')
    result = read(result_path)
    assert result['complete']
    if name not in state['finished_stages']:
        state['finished_stages'].append(name)
    state.update(updated_unix=time.time(), active_action='stage_complete')
    dump(BASE/'STATUS.json', state)
    return result

def combine(state):
    output = BASE/'combined'/f'v{len(state["finished_stages"]):02d}'
    if not (output/'RESULTS.json').exists():
        execute('analyze_libo_cutoff_height_expansion.py',
                ['combine', '--stages', *state['finished_stages'], '--out', output],
                state, f'combined_v{len(state["finished_stages"]):02d}')
    report = read(output/'RESULTS.json')
    assert report['complete']
    state['latest_combined_results'] = str(output/'RESULTS.json')
    decisions = {}
    for cutoff in (2, 4, 8):
        rows = sorted([j for j in report['stop_judgements'] if j['cutoff_mm'] == cutoff],
                      key=lambda j: j['height_scale'])
        assert rows, f'No stopping observations for cutoff{cutoff}'
        stopped = next((j for j in rows if j['stop']), None)
        top = rows[-1]
        decisions[str(cutoff)] = dict(status='stalled' if stopped else
                                     ('progressing' if top['eligible'] else 'insufficient_complete'),
                                     stopping_observation=stopped, latest_observation=top)
    state.update(cutoff_decisions=decisions, updated_unix=time.time())
    dump(BASE/'STATUS.json', state)
    return decisions

def main(allow_resume=False):
    protocol = read(BASE/'PROTOCOL.json')
    assert protocol['explicit_start'] and protocol['stop_rule_user_confirmed']
    assert protocol['priority'] == 'matched_through2_then_increment'
    state_path = BASE/'STATUS.json'
    if state_path.exists():
        assert allow_resume, 'Use deliberate resume; do not blind restart an existing controller'
        state = read(state_path)
        assert not state.get('execution_finished'), 'Finished controller must remain preserved'
        import psutil
        for field in ('pid', 'active_pid'):
            pid = state.get(field)
            if pid and psutil.pid_exists(pid):
                process = psutil.Process(pid)
                command = ' '.join(process.cmdline())
                assert not ('cutoff_height' in command and 'python' in process.name().lower()), \
                    f'An owned controller/stage process still runs: {pid}'
        state.setdefault('controller_resumes', []).append(dict(unix=time.time(), previous_pid=state.get('pid')))
    else:
        state = dict(started_unix=time.time(), finished_stages=[], execution_finished=False)
    state.update(pid=os.getpid(), status='running', updated_unix=time.time(),
                 controller_source_sha256=sha(__file__))
    dump(state_path, state)
    try:
        perform_stage('initial_4mm_h1p75_2p00', [4], [1.75, 2.], state, allow_resume)
        perform_stage('matched_2_8mm_h0p25_2p00', [2, 8],
                      [.25, .5, .75, 1., 1.25, 1.5, 1.75, 2.], state, allow_resume)
        decisions = combine(state)
        height = max(d['latest_observation']['height_scale'] for d in decisions.values()) + .25
        while True:
            active = [int(k) for k, value in decisions.items() if value['status'] == 'progressing']
            if not active:
                break
            token = f'{height:.2f}'.replace('.', 'p')
            stage = 'higher_h'+token+'_c'+'_'.join(map(str, active))
            perform_stage(stage, active, [height], state, allow_resume)
            decisions = combine(state)
            height += .25
        state.update(execution_finished=True,
                     status='needs_review' if any(d['status']=='insufficient_complete' for d in decisions.values()) else 'stop_criterion_reached',
                     physics_queue_empty=True, updated_unix=time.time(),
                     elapsed_wall_s=time.time()-state['started_unix'],
                     meaning='All dispatched conditions accounted for. Insufficient complete trajectories are not classified as stalled. Scientific figure/video QA and final delivery remain separate.')
        dump(state_path, state)
    except BaseException:
        state.update(status='needs_attention', execution_finished=False,
                     updated_unix=time.time(), error=traceback.format_exc())
        dump(state_path, state)
        raise

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--resume', action='store_true')
    main(parser.parse_args().resume)
