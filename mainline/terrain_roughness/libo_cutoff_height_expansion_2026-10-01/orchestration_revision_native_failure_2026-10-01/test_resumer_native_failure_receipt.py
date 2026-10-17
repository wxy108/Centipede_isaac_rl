"""C-only synthetic receipt/accounting checks; never launches native physics."""
from pathlib import Path
import ast
import copy
import hashlib
import json
import runpy
import tempfile

BASE = Path(__file__).resolve().parent
PATCH = BASE / 'resume_libo_cutoff_height_expansion.py'
FROZEN = BASE / 'resumer_frozen_before.py'
ROOT_DRAFT = BASE.parents[1] / 'NATIVE_FAILURE_CLASSIFICATION.json'
ns = runpy.run_path(str(PATCH))
errors = json.loads(ROOT_DRAFT.read_text(encoding='utf-8'))['native_errors']
required = ('active.json', 'input_contract_validation.json', 'matched_plant_check.json',
            'readback.json', 'native_error_audit.json', 'results.json', 'trajectory.npz',
            'cluster_contacts.npz', 'collision_guard.json', 'worker.log')
passed = []


def save(path, value):
    path.write_text(json.dumps(value), encoding='utf-8')


def fixture(parent, name):
    directory = parent / name
    directory.mkdir()
    for filename in required:
        (directory / filename).write_text('SYNTHETIC_TEST_NOT_A_NATIVE_RUN', encoding='utf-8')
    save(directory / 'active.json', dict(source_ids=[13, 14], status='failed', exit_code=1))
    save(directory / 'native_error_audit.json', dict(errors=errors))
    save(directory / 'results.json', dict(complete=True, whole_batch_finite=True,
         finite=True, actual_duration_s=6.))
    save(directory / 'collision_guard.json', dict(minimum_all_step_collision_margin_m=.02))
    receipt = dict(schema='libo_explicit_native_failure_v1', classification='native_startup_error',
                   source_ids=[13, 14], retry_allowed=False, scoring_allowed=False,
                   native_errors=copy.deepcopy(errors), artifact_hashes={})
    write_receipt(directory, receipt)
    return directory, receipt


def write_receipt(directory, receipt):
    receipt['artifact_hashes'] = {str((directory / filename).resolve()):
                                 hashlib.sha256((directory / filename).read_bytes()).hexdigest()
                                 for filename in required}
    save(directory / 'NATIVE_FAILURE_CLASSIFICATION.json', receipt)


def rejected(label, callback):
    try:
        callback()
    except (AssertionError, KeyError, RuntimeError):
        passed.append(label)
    else:
        raise AssertionError(f'Must reject: {label}')


with tempfile.TemporaryDirectory(prefix='synthetic_native_receipt_', dir=BASE) as temporary:
    parent = Path(temporary)
    d, receipt = fixture(parent, 'valid')
    assert ns['explicit_native_failure'](d, [13, 14]) == receipt
    assert ns['terminal_failure_peers'](d, [13, 14]) == ([], [13, 14])
    passed.append('valid_explicit_terminal_receipt')
    rejected('native_error_zero_gate_unchanged', lambda: ns['is_complete'](d))

    d, receipt = fixture(parent, 'tampered')
    (d / 'worker.log').write_text('TAMPERED_SYNTHETIC_DATA', encoding='utf-8')
    rejected('tampered_artifact_hash', lambda: ns['explicit_native_failure'](d, [13, 14]))

    d, receipt = fixture(parent, 'empty_receipt')
    save(d / 'NATIVE_FAILURE_CLASSIFICATION.json', {})
    rejected('empty_receipt', lambda: ns['explicit_native_failure'](d, [13, 14]))

    d, receipt = fixture(parent, 'empty_audit')
    save(d / 'native_error_audit.json', dict(errors=[]))
    receipt['native_errors'] = []
    write_receipt(d, receipt)
    rejected('empty_native_audit', lambda: ns['explicit_native_failure'](d, [13, 14]))

    d, receipt = fixture(parent, 'unknown_errors')
    unknown = errors + ['2026-10-02T01:14:00Z [411ms] [Error] PhysX unknown fault']
    save(d / 'native_error_audit.json', dict(errors=unknown))
    receipt['native_errors'] = unknown
    write_receipt(d, receipt)
    rejected('unknown_additional_native_error', lambda: ns['explicit_native_failure'](d, [13, 14]))

    d, receipt = fixture(parent, 'ids_mismatch')
    rejected('source_ID_mismatch', lambda: ns['explicit_native_failure'](d, [13]))

    d, receipt = fixture(parent, 'quality_overlap')
    save(d / 'BOUNDARY_FAILURE.json', dict(offender_source_ids=[13]))
    rejected('quality_receipt_overlap', lambda: ns['terminal_failure_peers'](d, [13, 14]))

    d, receipt = fixture(parent, 'missing_receipt')
    (d / 'NATIVE_FAILURE_CLASSIFICATION.json').unlink()
    assert ns['explicit_native_failure'](d, [13, 14]) is None
    rejected('unclassified_native_failure_still_blocks', lambda: ns['terminal_failure_peers'](d, [13, 14]))

    d, receipt = fixture(parent, 'outside_case')
    outside = parent / 'OUTSIDE_SYNTHETIC_ARTIFACT.txt'
    outside.write_text('synthetic', encoding='utf-8')
    receipt['artifact_hashes'][str(outside.resolve())] = hashlib.sha256(outside.read_bytes()).hexdigest()
    save(d / 'NATIVE_FAILURE_CLASSIFICATION.json', receipt)
    rejected('artifact_path_outside_case', lambda: ns['explicit_native_failure'](d, [13, 14]))

    d, receipt = fixture(parent, 'late_error')
    late = [line.replace('[411ms]', '[1411ms]') for line in errors]
    save(d / 'native_error_audit.json', dict(errors=late))
    receipt['native_errors'] = late
    write_receipt(d, receipt)
    rejected('nonstartup_error_timestamp', lambda: ns['explicit_native_failure'](d, [13, 14]))

    for label, old, new in [('same_family_other_date', '2026-10-02', '2026-10-03'),
                            ('same_family_other_millisecond', '[411ms]', '[412ms]')]:
        d, receipt = fixture(parent, label)
        changed = [line.replace(old, new) for line in errors]
        save(d / 'native_error_audit.json', dict(errors=changed))
        receipt['native_errors'] = changed
        write_receipt(d, receipt)
        rejected(label, lambda: ns['explicit_native_failure'](d, [13, 14]))

    for kind in ('boundary', 'numerical'):
        d = parent / kind
        d.mkdir()
        if kind == 'boundary':
            save(d / 'BOUNDARY_FAILURE.json', dict(offender_source_ids=[14]))
        else:
            save(d / 'NUMERICAL_FAILURE.json', dict(environments=[1]))
        assert ns['terminal_failure_peers'](d, [13, 14]) == ([13], [14])
        passed.append(f'original_{kind}_peer_path_unchanged')

    # Terminal startup errors join accounting without masquerading as numerical failures.
    sim = parent / 'synthetic_sim'
    case = sim / 'runs/v1/native_case'
    case.parent.mkdir(parents=True)
    case, receipt = fixture(case.parent, 'native_case')
    job = dict(name='native_case', source_ids=[13, 14])
    save(case / 'active.json', dict(job='native_case', source_ids=[13, 14], status='failed', exit_code=1))
    write_receipt(case, receipt)
    item = dict(job='native_case', attempt='v1', keep=None, exit=1)
    ns['record_native_failure'](item, receipt, [13, 14])
    completed, terminal, pending = ns['accounting']([item], {'native_case': job}, sim)
    assert not completed and terminal == {('native_case', 13), ('native_case', 14)} and not pending
    assert item['classification'] == 'native_startup_error' and item['native_failure_source_ids'] == [13, 14]
    assert 'offender_source_ids' not in item and not item.get('external_interruption')
    passed.append('terminal_native_accounting_no_retry_no_score')


def function_source(path, name):
    source = path.read_text(encoding='utf-8')
    return next(ast.get_source_segment(source, node) for node in ast.parse(source).body
                if isinstance(node, ast.FunctionDef) and node.name == name)


assert function_source(FROZEN, 'is_complete') == function_source(PATCH, 'is_complete')
passed.append('is_complete_source_bitexact')
print(json.dumps(dict(scope='Temporary C-only synthetic fixtures; no native execution',
                     passed=True, cases=passed,
                     frozen_sha256=hashlib.sha256(FROZEN.read_bytes()).hexdigest(),
                     patched_sha256=hashlib.sha256(PATCH.read_bytes()).hexdigest()), indent=2))
