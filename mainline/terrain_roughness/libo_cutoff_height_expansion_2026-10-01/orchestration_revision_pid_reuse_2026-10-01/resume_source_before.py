"""Reconcile a stopped finite terrain stage, then dispatch only unaccounted cases.

No numerical/boundary offender is retried. An externally interrupted worker may
repeat its exact unscored condition layout; this is not a new independent sample.
"""
from pathlib import Path
import os
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[key] = '1'
os.environ['PXR_WORK_THREAD_LIMIT'] = '2'
import argparse
import copy
import json
import subprocess
import sys
import time
import psutil

ROOT = Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
sys.path.insert(0, str(ROOT))
import run_libo_cutoff_height_expansion as runner

read, dump, sha = runner.read, runner.dump, runner.sha
SELF = Path(__file__).resolve()
WORKER_RUNNER = ROOT / 'run_libo_cutoff_height_expansion.py'


def has_stage(command, stage):
    return any(command[i] == '--stage' and command[i+1] == stage
               for i in range(len(command)-1))


def owns_command(command, stage):
    script_paths = {os.path.normcase(str(path.resolve()))
                    for path in (WORKER_RUNNER, SELF, Path(runner.RUNNER))}
    return has_stage(command, stage) and any(
        os.path.normcase(os.path.abspath(word)) in script_paths for word in command)


def pid_receipt(pid, stage):
    """A reused PID is recorded separately; inaccessible recorded identity blocks resume."""
    try:
        process = psutil.Process(int(pid))
        command = process.cmdline()
        return dict(pid=int(pid), alive=process.is_running(), owned=owns_command(command, stage),
                    command=command, create_time=process.create_time())
    except psutil.NoSuchProcess:
        return dict(pid=int(pid), alive=False, owned=False)
    except psutil.AccessDenied as error:
        raise RuntimeError(f'Cannot verify recorded PID ownership: {pid}') from error


def ownership_audit(sim, stage, prior, dispatch):
    pids = {int(dispatch['pid'])}
    pids.update(int(item['pid']) for item in prior.get('current', []) if item.get('pid'))
    for path in (sim / 'runs').glob('*/*/active.json'):
        active = read(path)
        if active.get('pid'):
            pids.add(int(active['pid']))
    receipts = [pid_receipt(pid, stage) for pid in sorted(pids)]
    assert not any(item['alive'] and item['owned'] for item in receipts), \
        'Recorded dispatcher/worker for this stage is still alive'
    live = []
    for process in psutil.process_iter(['pid', 'cmdline']):
        try:
            command = process.info['cmdline'] or []
            if process.pid != os.getpid() and owns_command(command, stage):
                live.append(dict(pid=process.pid, command=command))
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            # Recorded PID AccessDenied was handled above; unrelated protected OS
            # processes cannot be treated as evidence of a live owned dispatcher.
            continue
    assert not live, f'Another owned process exists for this stage: {live}'
    return dict(stage=stage, checked_unix=time.time(), recorded_pids=receipts,
                other_live_owned_processes=live, no_processes_killed=True)


def is_complete(directory):
    if not (directory / 'results.json').exists():
        return False
    result = read(directory / 'results.json')
    if not result.get('complete'):
        return False
    assert result['whole_batch_finite'] and result['actual_duration_s'] == 6.
    assert not read(directory / 'native_error_audit.json')['errors']
    assert read(directory / 'collision_guard.json')['minimum_all_step_collision_margin_m'] > .015
    for name in ('trajectory.npz', 'cluster_contacts.npz', 'readback.json'):
        assert (directory / name).is_file(), directory / name
    return True


def selected_ids(item, job, directory):
    active = read(directory / 'active.json') if (directory / 'active.json').exists() else {}
    effective = (read(directory / 'effective_plan.json')
                 if (directory / 'effective_plan.json').exists() else {})
    choices = []
    if item.get('keep') is not None:
        choices.append(item['keep'])
    elif item['attempt'] == 'v1':
        choices.append(job['source_ids'])
    if active.get('source_ids') is not None:
        choices.append(active['source_ids'])
    if effective.get('benchmark_source_ids') is not None:
        choices.append(effective['benchmark_source_ids'])
    assert choices, f'Cannot reconstruct actual worker layout: {directory}'
    ids = list(choices[0])
    assert ids and len(set(ids)) == len(ids) and set(ids) <= set(job['source_ids'])
    assert all(list(candidate) == ids for candidate in choices), directory
    if active.get('job'):
        assert active['job'] == job['name']
    return ids, active


def accounting(finished, jobs, sim):
    completed, offended, peers_by_job = set(), set(), {}
    for item in finished:
        job = jobs[item['job']]
        directory = sim / 'runs' / item['attempt'] / item['job']
        ids, _ = selected_ids(item, job, directory)
        if item['exit'] == 0:
            assert is_complete(directory)
            completed.update((item['job'], source_id) for source_id in ids)
        elif not item.get('external_interruption'):
            peers, offenders = runner.original.failure_peers(directory, ids)
            offended.update((item['job'], source_id) for source_id in offenders)
            peers_by_job.setdefault(item['job'], set()).update(peers)
    assert not completed & offended, 'A native quality offender has been retried/scored'
    settled = completed | offended
    pending = {}
    for name, peers in peers_by_job.items():
        ids = [source_id for source_id in jobs[name]['source_ids']
               if source_id in peers and (name, source_id) not in settled]
        if ids:
            pending[name] = ids
    return completed, offended, pending


def resume(stage):
    runner.configure(stage, 0)
    sim = runner.SIM
    campaign = read(sim / 'campaign.json')
    runner.configure(stage, campaign['condition_count'])
    assert campaign['stage'] == stage
    prior, dispatch = read(sim / 'supervisor.json'), read(sim / 'DISPATCH.json')
    assert not prior['complete'], 'A completed stage cannot be resumed'
    audit = ownership_audit(sim, stage, prior, dispatch)
    runner.check()
    jobs = {job['name']: job for job in campaign['jobs']}
    history = sim / 'dispatch_resume_history'
    history.mkdir(exist_ok=True)
    number = 1
    while (history / f'supervisor_before_{number:02d}.json').exists():
        number += 1
    # Exclusive claim prevents two dispatchers racing after the ownership check.
    claim_path = sim / 'RESUME_CLAIM.json'
    if claim_path.exists():
        claim = read(claim_path)
        assert claim['stage'] == stage
        receipt = pid_receipt(claim['pid'], stage)
        assert not (receipt['alive'] and receipt['owned']), 'Existing resume claim is active'
        claim_path.replace(history / f'resume_claim_before_{number:02d}.json')
    with claim_path.open('x', encoding='utf-8') as handle:
        json.dump(dict(pid=os.getpid(), stage=stage, created_unix=time.time(), runner=str(SELF)), handle, indent=2)
    before = history / f'supervisor_before_{number:02d}.json'
    before.write_bytes((sim / 'supervisor.json').read_bytes())
    (history / f'dispatch_before_{number:02d}.json').write_bytes((sim / 'DISPATCH.json').read_bytes())
    (history / f'resume_source_{number:02d}.py').write_bytes(SELF.read_bytes())
    dump(history / f'ownership_audit_{number:02d}.json', audit)
    started = prior['started_unix']
    finished = copy.deepcopy(prior['finished'])
    known = {(item['attempt'], item['job']) for item in finished}
    assert len(known) == len(finished), 'Duplicate supervisor attempt records'
    pending_items = {(item['attempt'], item['job']): copy.deepcopy(item)
                     for item in prior.get('current', []) if (item['attempt'], item['job']) not in known}
    # Find workers launched immediately before the last supervisor checkpoint.
    for directory in sorted((sim / 'runs').glob('*/*')):
        if not directory.is_dir():
            continue
        attempt, name = directory.parent.name, directory.name
        assert name in jobs, f'Unexpected job directory: {directory}'
        key = (attempt, name)
        if key not in known and key not in pending_items:
            pending_items[key] = dict(job=name, attempt=attempt, keep=None)
    external = []
    for item in pending_items.values():
        assert item['job'] in jobs
        job = jobs[item['job']]
        directory = sim / 'runs' / item['attempt'] / item['job']
        ids, active = selected_ids(item, job, directory)
        if active.get('pid'):
            item['pid'] = active['pid']
        item['keep'] = None if ids == job['source_ids'] else ids
        if (directory / 'BOUNDARY_FAILURE.json').exists() or (directory / 'NUMERICAL_FAILURE.json').exists():
            runner.original.failure_peers(directory, ids)
            item.update(exit=1, status='failed', reconciled_after_dispatch_interruption=True)
        elif is_complete(directory):
            item.update(exit=0, status='finished', reconciled_after_dispatch_interruption=True)
        else:
            assert active.get('status') != 'failed' and active.get('exit_code') in (None, 0), \
                f'Non-quality worker failure requires diagnosis: {directory}'
            assert not (directory / 'results.json').exists(), \
                f'Unclassified partial/invalid result requires diagnosis: {directory}'
            log = directory / 'worker.log'
            assert not log.exists() or 'Traceback (most recent call last)' not in log.read_text(
                encoding='utf-8', errors='replace'), f'Unclassified worker exception: {directory}'
            receipt = dict(classification='external_interruption_unscored', source_ids=ids,
                           observed_unix=time.time(), last_active_status=active.get('status'),
                           evidence='Owned dispatcher and workers absent; no complete result or native quality-failure receipt.',
                           cause='Unknown; do not label as numerical divergence or zero displacement.',
                           physics_or_amplitude_changes=False, reexecute_same_conditions=True,
                           original_worker_layout=ids, original_CPG_phase=0,
                           resume_number=number)
            if (directory / 'EXTERNAL_INTERRUPTION.json').exists():
                old = read(directory / 'EXTERNAL_INTERRUPTION.json')
                assert old['source_ids'] == ids
            else:
                dump(directory / 'EXTERNAL_INTERRUPTION.json', receipt)
            item.update(exit=-999, status='externally_interrupted', external_interruption=True)
            external.append((item['job'], f'interruption_resume{number:02d}', item['keep']))
        finished.append(item)
    completed, offended, pending_peers = accounting(finished, jobs, sim)
    settled = completed | offended
    actual_external = []
    for name, attempt, keep in external:
        ids = keep if keep is not None else jobs[name]['source_ids']
        pending = [source_id for source_id in ids if (name, source_id) not in settled]
        if not pending:
            continue
        assert pending == ids, 'Conflicting external attempt accounting; preserve evidence and diagnose'
        actual_external.append((name, attempt, keep))
        pending_peers.pop(name, None)
    assert len({name for name, _, _ in actual_external}) == len(actual_external), \
        'More than one unresolved external worker layout for a job'
    queue = [(name, f'peer_resume{number:02d}_round1', ids) for name, ids in pending_peers.items()]
    dump(sim / 'DISPATCH.json', dict(pid=os.getpid(), status='resumed', stage=stage,
         runner=str(SELF), runner_sha256=sha(SELF), worker_runner=str(WORKER_RUNNER),
         worker_runner_sha256=sha(WORKER_RUNNER), resumed_unix=time.time(),
         original_started_unix=started, condition_count=campaign['condition_count'],
         scope=campaign['scope'], previous_receipt=str(history / f'dispatch_before_{number:02d}.json')))

    def batch(entries, label):
        nonlocal completed, offended
        assert 0 < len(entries) <= 4
        procs = []
        for name, attempt, keep in entries:
            ids = keep if keep is not None else jobs[name]['source_ids']
            assert not any((name, source_id) in completed | offended for source_id in ids)
            assert not (sim / 'runs' / attempt / name).exists()
            command = [sys.executable, '-B', str(WORKER_RUNNER), 'worker', '--stage', stage,
                       '--job', name, '--attempt', attempt]
            if keep is not None:
                command += ['--keep', ','.join(map(str, keep))]
            with (sim / f'{attempt}_{name}_launch.log').open('wb') as log:
                process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                           stdout=log, stderr=subprocess.STDOUT,
                                           creationflags=subprocess.CREATE_NO_WINDOW)
            procs.append((name, attempt, keep, process))
            time.sleep(2)
        while True:
            current = []
            for name, attempt, keep, process in procs:
                try:
                    active = read(sim / 'runs' / attempt / name / 'active.json')
                except (FileNotFoundError, json.JSONDecodeError):
                    active = {}
                current.append(dict(job=name, attempt=attempt, keep=keep, pid=process.pid,
                                    exit=process.poll(), status=active.get('status')))
            dump(sim / 'supervisor.json', dict(complete=False, batch=label,
                 started_unix=started, elapsed_s=time.time()-started, finished=finished,
                 current=current, resume_number=number, original_interruption_receipt=str(before)))
            if all(process.poll() is not None for *_, process in procs):
                break
            time.sleep(10)
        finished.extend(current)
        for item in current:
            ids = item['keep'] if item['keep'] is not None else jobs[item['job']]['source_ids']
            directory = sim / 'runs' / item['attempt'] / item['job']
            if item['exit'] == 0:
                assert is_complete(directory)
                completed.update((item['job'], source_id) for source_id in ids)
            else:
                peers, offenders = runner.original.failure_peers(directory, ids)
                item['offender_source_ids'], item['interrupted_peer_source_ids'] = offenders, peers
                offended.update((item['job'], source_id) for source_id in offenders)
                if peers:
                    queue.append((item['job'], f'peer_resume{number:02d}_after_{item["attempt"]}', peers))
        assert not completed & offended
        print(json.dumps(dict(batch=label, worker_attempts=len(finished),
                              elapsed_s=time.time()-started, pending_peers=len(queue))), flush=True)

    for offset in range(0, len(actual_external), 4):
        batch(actual_external[offset:offset+4], f'external_interruption_resume{number:02d}_{offset//4}')
    attempted_formal = {item['job'] for item in finished if item['attempt'] == 'v1'}
    for original_batch in campaign['batches']:
        entries = [(name, 'v1', None) for name in original_batch['jobs'] if name not in attempted_formal]
        if entries:
            batch(entries, original_batch['name'])
    recovery_round = 0
    while queue:
        recovery_round += 1
        assert recovery_round <= 3, 'Bounded three-round nonoffender recovery reached'
        current, queue = queue[:], []
        for offset in range(0, len(current), 4):
            batch(current[offset:offset+4], f'peer_recovery_resume{number:02d}_{recovery_round}_{offset//4}')
    completed, offended, outstanding = accounting(finished, jobs, sim)
    assert not outstanding
    expected = {(name, source_id) for name, job in jobs.items() for source_id in job['source_ids']}
    assert completed | offended == expected
    dump(sim / 'supervisor.json', dict(complete=True, all_attempts_accounted_for=True,
         started_unix=started, elapsed_s=time.time()-started, finished=finished, current=[],
         resume_number=number, original_interruption_receipt=str(before),
         meaning=f'All{campaign["condition_count"]} finite-manifest conditions accounted for; '
                 'native failures retained, external unscored attempts separate, no new independent samples.'))
    dump(claim_path, dict(pid=os.getpid(), stage=stage, completed=True, completed_unix=time.time(), runner=str(SELF)))
    print('FINITE-MANIFEST ATTEMPTS FINISHED; no further physics queued', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', required=True)
    args = parser.parse_args()
    resume(args.stage)
