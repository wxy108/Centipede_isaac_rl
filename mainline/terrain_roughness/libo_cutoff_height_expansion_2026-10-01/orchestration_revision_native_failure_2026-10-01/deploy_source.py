"""Deploy reviewed terminal-failure bookkeeping; never change scientific inputs."""
from pathlib import Path
import hashlib
import json
import shutil
import time
import psutil

LOCAL = Path(__file__).resolve().parent
ROOT = Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
BASE = ROOT / 'libo_cutoff_height_expansion_2026-10-01'
SIM = BASE / 'higher_h2p25_c2_4_8/simulation'
CASE = SIM / 'runs/v1/cutoff_4mm_height_x2p25_s02_high'
ARCHIVE = BASE / 'orchestration_revision_native_failure_2026-10-01'

def read(path):
    return json.loads(path.read_text(encoding='utf-8'))

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def dump(path, data):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)

def owned_processes_absent(status, supervisor):
    pids = [status['pid'], *[item['pid'] for item in supervisor['current']]]
    if status.get('active_pid'):
        pids.append(status['active_pid'])
    for pid in pids:
        assert not psutil.pid_exists(pid), f'Current PID reappeared; identity review required: {pid}'
    owned_scripts = {'continue_libo_cutoff_height_expansion.py',
                     'run_libo_cutoff_height_expansion.py',
                     'resume_libo_cutoff_height_expansion.py', 'run_libo_repeats.py'}
    live = []
    for p in psutil.process_iter(['pid', 'name']):
        if 'python' not in (p.info['name'] or '').lower() or p.pid == psutil.Process().pid:
            continue
        try:
            command = p.cmdline()
        except psutil.NoSuchProcess:
            continue
        except psutil.AccessDenied as error:
            raise RuntimeError(f'Unknown Python identity blocks deployment: {p.pid}') from error
        if any(Path(word).name in owned_scripts for word in command):
            live.append(dict(pid=p.pid, command=command))
    assert not live, live
    return pids

protocol = read(BASE / 'PROTOCOL.json')
status = read(BASE / 'STATUS.json')
supervisor = read(SIM / 'supervisor.json')
assert status['status'] == 'needs_attention' and not status['execution_finished']
assert status['active_action'] == 'higher_h2p25_c2_4_8_run'
assert status['pid'] == 16224 and supervisor['batch'] == 'pair01_cutoff_4mm_height_x2p25'
assert not supervisor['complete'] and len(supervisor['finished']) == 12
expected_jobs = {
    'cutoff_4mm_height_x2p25_s02_low': 13120,
    'cutoff_4mm_height_x2p25_s02_high': 7420,
    'cutoff_4mm_height_x2p25_s03_low': 3188,
    'cutoff_4mm_height_x2p25_s03_high': 14144,
}
assert {item['job']: item['pid'] for item in supervisor['current']} == expected_jobs
assert all(item['attempt'] == 'v1' and item['exit'] == 1 for item in supervisor['current'])
absent_pids = owned_processes_absent(status, supervisor)
assert all(sha(Path(p)) == digest for p, digest in protocol['orchestration_sources'].items())
campaign = read(SIM / 'campaign.json')
assert all(sha(Path(p)) == digest for p, digest in campaign['pins'].items())
receipt = read(LOCAL / 'NATIVE_FAILURE_CLASSIFICATION.json')
assert receipt['schema'] == 'libo_explicit_native_failure_v1'
assert receipt['classification'] == 'native_startup_error'
assert receipt['source_ids'] == [13, 14]
assert receipt['scoring_allowed'] is False and receipt['retry_allowed'] is False
assert len(receipt['artifact_hashes']) == 14
for p, digest in receipt['artifact_hashes'].items():
    artifact = Path(p)
    assert artifact.is_absolute() and artifact.resolve().parent == CASE.resolve()
    assert sha(artifact) == digest, p
assert not (CASE / 'NATIVE_FAILURE_CLASSIFICATION.json').exists()
assert not any((CASE / name).exists() for name in
               ('BOUNDARY_FAILURE.json', 'NUMERICAL_FAILURE.json', 'QUALITY_FAILURE.json'))
assert read(CASE / 'native_error_audit.json')['errors'] == receipt['native_errors']
assert read(CASE / 'active.json')['source_ids'] == [13, 14]

expected_reports = {
    BASE / 'initial_4mm_h1p75_2p00/results/RESULTS.json':
        '1f7219061498dd6269c88279c47764afa2e528c34370518dbe8e8dd7a7c9bc6d',
    BASE / 'matched_2_8mm_h0p25_2p00/results/RESULTS.json':
        '4c39abc7a08f2409920b62d001ba8d6589e490552caff39a85dafb41a5d2a658',
    BASE / 'combined/v02/RESULTS.json':
        '71a35258b808c344b4360545810a0dccd7468e16202ed213b3d344031e555e47',
}
assert all(sha(path) == digest for path, digest in expected_reports.items())
assert sha(ROOT / 'render_libo_cutoff_height_expansion.py') == \
       '1b639be8f4763c72bea517067aee7bc5e66204f1fa0337e18704ca9617f29067'

patches = {
    ROOT / 'resume_libo_cutoff_height_expansion.py':
        LOCAL / 'work/native_failure_resumer_patch_2026-10-02/resume_libo_cutoff_height_expansion.py',
    ROOT / 'analyze_libo_cutoff_height_expansion.py': LOCAL / 'analyze_libo_cutoff_height_expansion.py',
}
test_summary = read(LOCAL / 'NATIVE_FAILURE_REVISION_TESTS.json')
assert test_summary['passed'] and test_summary['independent_review_passed']
assert all(sha(source) == test_summary['draft_hashes'][source.name] for source in patches.values())
assert not ARCHIVE.exists(), 'Revision already exists; do not repeat deployment'
ARCHIVE.mkdir()
for name in ('PROTOCOL.json', 'STATUS.json', 'CONTROLLER_DISPATCH.json'):
    shutil.copyfile(BASE / name, ARCHIVE / name)
for name in ('supervisor.json', 'DISPATCH.json', 'campaign.json'):
    shutil.copyfile(SIM / name, ARCHIVE / ('stage_' + name))
shutil.copyfile(BASE / 'logs/higher_h2p25_c2_4_8_run.log', ARCHIVE / 'stage_run_before_revision.log')
shutil.copyfile(LOCAL / 'NATIVE_FAILURE_CLASSIFICATION.json', ARCHIVE / 'NATIVE_FAILURE_CLASSIFICATION.json')
shutil.copyfile(LOCAL / 'NATIVE_FAILURE_REVISION_TESTS.json', ARCHIVE / 'TESTS.json')
shutil.copyfile(Path(__file__), ARCHIVE / 'deploy_source.py')
changes = []
for target, source in patches.items():
    old_sha = sha(target)
    new_sha = sha(source)
    shutil.copyfile(target, ARCHIVE / (target.stem + '_before.py'))
    shutil.copyfile(source, ARCHIVE / (target.stem + '_after.py'))
    changes.append(dict(source=str(target), previous_sha256=old_sha, new_sha256=new_sha))

# A new exclusion receipt preserves every original worker artifact byte for byte.
shutil.copyfile(LOCAL / 'NATIVE_FAILURE_CLASSIFICATION.json', CASE / 'NATIVE_FAILURE_CLASSIFICATION.json')
for target, source in patches.items():
    shutil.copyfile(source, target)
    protocol['orchestration_sources'][str(target)] = sha(target)
revision = dict(unix=time.time(), reason='Classify reviewed startup-plugin native gate failure as terminal invalid trials; preserve zero-error gate; no scoring or retry.',
                sources=changes, classified_source_ids=[13, 14],
                classification_receipt=str(CASE / 'NATIVE_FAILURE_CLASSIFICATION.json'),
                scientific_worker_PD_amplitude_physics_inputs_unchanged=True,
                original_runner_unchanged=True,
                verification=['Resumer and analyzer receipt/rejection tests passed',
                              'Independent review accepted terminal exclusion without scoring/retry',
                              'All36 stage scientific pins and preserved RESULTS hashes match'],
                archive=str(ARCHIVE))
protocol.setdefault('orchestration_revisions', []).append(revision)
dump(BASE / 'PROTOCOL.json', protocol)
assert all(sha(Path(p)) == digest for p, digest in campaign['pins'].items())
assert all(sha(Path(p)) == digest for p, digest in receipt['artifact_hashes'].items())
assert all(sha(path) == digest for path, digest in expected_reports.items())
assert all(sha(Path(p)) == digest for p, digest in protocol['orchestration_sources'].items())
dump(ARCHIVE / 'REVISION.json', dict(revision, deployment_passed=True,
     all_owned_processes_absent=True, absent_current_pids=absent_pids,
     original_artifact_hashes=receipt['artifact_hashes'],
     preserved_reports={str(p): d for p, d in expected_reports.items()},
     scientific_pins=campaign['pins']))
print(json.dumps(dict(deployment_passed=True, archive=str(ARCHIVE), sources=changes,
                     native_failure_ids=[13, 14], science_pins_unchanged=len(campaign['pins'])), ensure_ascii=False))
