# IsaacLab watchdog wrapper — usage

`scripts\sanity\run_isaaclab_with_watchdog.ps1` runs any IsaacLab audit or training command with PID-narrowed supervision: log capture, terminal-marker detection, silent-stall detection, Omniverse-shutdown-hang handling, and PID-narrowed force-kill (no broad `Get-Process python | Stop-Process`).

## Why use it instead of plain Bash run_in_background + grep Monitor

Two pitfalls bit us on 2026-05-10:

1. **PowerShell `Tee-Object` writes UTF-16 LE by default**, so a Bash `grep` watching the log finds zero matches even when the success line is right there. The watchdog redirects stdout+stderr via cmd.exe's `>` (ANSI bytes) so any reader works.
2. **Broad `Get-Process python | Stop-Process` is dangerous** when you have other python jobs (e.g., a parallel audit). The watchdog finds python.exe whose ParentProcessId is the launcher's cmd.exe PID and kills only those.

It also automates the "wait 10 s after success marker, force-kill if Omniverse cleanup hangs" pattern that we kept doing manually.

## Common invocations

### 100-iter overnight training

```powershell
.\scripts\sanity\run_isaaclab_with_watchdog.ps1 `
    -Cmd '.\IsaacLab\isaaclab.bat -p scripts\rl\train_stage_b_smoke.py --num-envs 16 --max-iterations 100 --headless --seed 42 --pass11-cot --pass12-cot-bounded --pass12-functional-guard --checkpoint outputs\rl_smoke\stage_b_transition_global6d\2026-05-08_15-27-26\nn\centipede_stage_b_transition_global6d_smoke.pth --out-dir outputs\rl_smoke\stage_b_transition_global6d_pass12_relaxed_overnight_2026-05-10' `
    -LogFile 'outputs\analysis\overnight_session_2026-05-10\05_overnight_training.log' `
    -ExpectedDurationMin 35
```

### Audit (latest_policy / bio_metrics / reward_decomp)

For an audit, set `-ExpectedReport` to the report file the audit emits, so the watchdog can verify success on hang:

```powershell
.\scripts\sanity\run_isaaclab_with_watchdog.ps1 `
    -Cmd '.\IsaacLab\isaaclab.bat -p scripts\sanity\audit_pass12_reward_decomposition.py --checkpoint <CKPT> --pass12-flag cot_bounded --pass12-flag functional_guard --report-suffix overnight_BEST' `
    -LogFile 'outputs\analysis\overnight_session_2026-05-10\06_reward_decomp_BEST.log' `
    -ExpectedDurationMin 30 `
    -ExpectedReport 'outputs\sanity\pass12_reward_decomposition_report_overnight_BEST.txt'
```

## Parameters

| param | required | default | description |
|---|---|---|---|
| `-Cmd` | yes | — | Full IsaacLab command line (string) |
| `-LogFile` | yes | — | Log path (overwritten if exists) |
| `-ExpectedReport` | no | '' | Report file the audit/training emits; on success-marker hit, watchdog verifies it exists |
| `-ExpectedDurationMin` | no | 30 | Used to compute deadline (1.5×) |
| `-PollSec` | no | 15 | Polling interval |
| `-StaleLogMin` | no | 10 | Stale-log threshold for silent-stall classification |
| `-SessionStatePath` | no | `outputs\analysis\overnight_session_2026-05-10\SESSION_STATE.md` | Where to append status updates |

## Exit codes

- `0` — success (success marker hit, expected report present if specified)
- `1` — anything else (error marker, silent stall, deadline, process died without marker, success but expected report missing)

## When to invoke from a Bash background task

```bash
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/sanity/run_isaaclab_with_watchdog.ps1 -Cmd '<...>' -LogFile '<...>' -ExpectedDurationMin <N>
```

Use Bash `run_in_background: true` so you get a harness completion notification when the watchdog itself returns. The watchdog's exit code will be reported.
