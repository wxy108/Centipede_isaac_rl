<#
.SYNOPSIS
    Run an IsaacLab audit/training with PID-narrowed watchdog supervision.

.DESCRIPTION
    Launches the given IsaacLab command via cmd.exe so we get an exact PID,
    captures stdout+stderr to a log file (UTF-8 via cmd redirection so
    Bash/grep readers don't trip over UTF-16). Polls every PollSec seconds
    for terminal markers (success/error), silent stalls (log stale + GPU
    near-zero + python CPU not growing), and an overall deadline.

    On terminal hit:
      - success marker → wait up to 10 s for the launcher's python.exe
        descendants to exit cleanly. If they don't (Omniverse cleanup hang),
        force-kill ONLY the launcher's python.exe descendants by
        ParentProcessId — never broad-kill all python.exe.
      - error marker → kill the same PID-narrowed set, return failure.
      - silent stall → kill the same PID-narrowed set, return hang.

    Status updates are appended to SessionStatePath (default
    outputs\analysis\overnight_session_2026-05-10\SESSION_STATE.md).

.PARAMETER Cmd
    The full IsaacLab command line (string), e.g.
    '.\IsaacLab\isaaclab.bat -p scripts\rl\train_stage_b_smoke.py ...'.
    Will be invoked by cmd.exe /c with stdout+stderr redirected to LogFile.

.PARAMETER LogFile
    Project-relative or absolute path to the log file (overwritten if exists).

.PARAMETER ExpectedReport
    Optional expected output file (e.g. the report .txt for an audit). If
    set, on a success-marker hit we also verify this file exists; if it
    doesn't, result is 'success_but_no_report'.

.PARAMETER ExpectedDurationMin
    Expected job duration in minutes. Deadline = launch time + 1.5x this.

.PARAMETER PollSec
    Polling interval in seconds. Default 15.

.PARAMETER StaleLogMin
    Stale-log threshold in minutes for silent-stall classification. Default 10.

.PARAMETER SessionStatePath
    SESSION_STATE.md to append status updates. Default
    outputs\analysis\overnight_session_2026-05-10\SESSION_STATE.md.

.EXAMPLE
    .\scripts\sanity\run_isaaclab_with_watchdog.ps1 `
        -Cmd '.\IsaacLab\isaaclab.bat -p scripts\rl\train_stage_b_smoke.py --num-envs 16 --max-iterations 100 --headless --seed 42 --pass11-cot --pass12-cot-bounded --pass12-functional-guard --checkpoint outputs\rl_smoke\stage_b_transition_global6d\2026-05-08_15-27-26\nn\centipede_stage_b_transition_global6d_smoke.pth --out-dir outputs\rl_smoke\stage_b_transition_global6d_pass12_relaxed_overnight_2026-05-10' `
        -LogFile 'outputs\analysis\overnight_session_2026-05-10\05_overnight_training.log' `
        -ExpectedDurationMin 35

    Returns exit code 0 on clean success, 1 otherwise.
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)] [string]$Cmd,
    [Parameter(Mandatory=$true)] [string]$LogFile,
    [string]$ExpectedReport = '',
    [int]$ExpectedDurationMin = 30,
    [int]$PollSec = 15,
    [int]$StaleLogMin = 10,
    [string]$SessionStatePath = 'outputs\analysis\overnight_session_2026-05-10\SESSION_STATE.md'
)

$ErrorActionPreference = 'Continue'
$startTime = Get-Date

# --- Fixed marker lists (literal substrings; case-sensitive) ----------------
$successMarkers = @(
    '[lat] report ->',
    '[bio] report ->',
    '[rd12] report ->',
    '[viz] report ->',
    '[sweep] report ->',
    '[a:r0:step1b] report ->',
    'wrote summary ->',
    'training loop returned without exception',
    'MAX EPOCHS NUM',
    'epoch 100/100',
    'finished'
)
$errorMarkers = @(
    'Traceback',
    'RuntimeError',
    'CUDA out of memory',
    'out of memory',
    'failed to allocate',
    'PhysX',
    'User allocator returned NULL',
    'NaN'
)

# --- Helpers -----------------------------------------------------------------
function Write-State {
    param([string]$Msg)
    $ts = (Get-Date).ToString('yyyy-MM-dd HH:mm:ss')
    $line = "[watchdog $ts] $Msg"
    if ($SessionStatePath -and (Test-Path $SessionStatePath)) {
        try { Add-Content -Path $SessionStatePath -Value $line -Encoding UTF8 } catch { }
    }
    Write-Host $line
}

function Find-ChildPython {
    param([int]$LauncherPid)
    # Returns CIM Win32_Process objects of python.exe whose ParentProcessId
    # is the launcher's PID, or whose grandparent is the launcher (cmd.exe
    # often spawns python.exe through a wrapper).
    $allPython = Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue
    $direct = $allPython | Where-Object { $_.ParentProcessId -eq $LauncherPid }
    if ($direct) { return $direct }
    # Fallback: walk parent chain. Build a parent map.
    $allProc = Get-CimInstance Win32_Process -ErrorAction SilentlyContinue
    $byPid = @{}
    foreach ($p in $allProc) { $byPid[[int]$p.ProcessId] = $p }
    $descendants = @()
    foreach ($py in $allPython) {
        $cur = $py
        $depth = 0
        while ($cur -and $depth -lt 8) {
            if ($cur.ParentProcessId -eq $LauncherPid) {
                $descendants += $py; break
            }
            $cur = $byPid[[int]$cur.ParentProcessId]
            $depth++
        }
    }
    return $descendants
}

function Get-GpuStats {
    try {
        $line = (& nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.free --format=csv,noheader,nounits 2>$null) -join "`n"
        $first = ($line -split "`n")[0]
        $parts = $first -split ','
        return @{
            util_pct  = [int]($parts[0].Trim())
            mem_used  = [int]($parts[1].Trim())
            mem_free  = [int]($parts[2].Trim())
            ok = $true
        }
    } catch {
        return @{ util_pct = -1; mem_used = -1; mem_free = -1; ok = $false }
    }
}

function Test-MarkerIn {
    param([string[]]$Lines, [string[]]$Markers)
    # NOTE: use .Contains() for literal substring match. PowerShell's -like
    # treats `[chars]` as a character class, so a marker like '[lat] report ->'
    # would never match its literal occurrence in the log.
    foreach ($m in $Markers) {
        $hit = $Lines | Where-Object { $_ -and $_.Contains($m) } | Select-Object -Last 1
        if ($hit) { return @{ marker = $m; line = $hit } }
    }
    return $null
}

# --- Pre-flight --------------------------------------------------------------
$logDir = Split-Path $LogFile -Parent
if ($logDir -and -not (Test-Path $logDir)) {
    New-Item -ItemType Directory -Path $logDir -Force | Out-Null
}
# Truncate log
"" | Out-File -FilePath $LogFile -Encoding utf8 -Force

Write-State "=========================================="
Write-State "watchdog start. cwd=$(Get-Location)"
Write-State "cmd:  $Cmd"
Write-State "log:  $LogFile"
Write-State "expected_duration_min=$ExpectedDurationMin  poll_sec=$PollSec  stale_log_min=$StaleLogMin"
if ($ExpectedReport) { Write-State "expected_report: $ExpectedReport" }

# --- Launch -----------------------------------------------------------------
# Use cmd.exe /c so we can append-redirect stdout+stderr to LogFile reliably
# (cmd's > redirection writes ANSI by default, which Bash grep reads fine).
$logFileAbs = (Resolve-Path -LiteralPath $LogFile -ErrorAction SilentlyContinue).Path
if (-not $logFileAbs) { $logFileAbs = (Join-Path -Path (Get-Location).Path -ChildPath $LogFile) }
# Force line-buffered Python stdout so the watchdog's grep can see markers in
# real time (cmd's `>` redirect causes Python to block-buffer 4-8 KB by
# default, which means terminal markers may not appear in the log until the
# process exits — the exact failure we want to AVOID for live monitoring).
$cmdLineFull = "set PYTHONUNBUFFERED=1 && $Cmd >> `"$logFileAbs`" 2>&1"

$launcher = Start-Process -FilePath "cmd.exe" `
    -ArgumentList "/c", $cmdLineFull `
    -PassThru -WindowStyle Hidden
Write-State "PYTHONUNBUFFERED=1 set in launcher cmd to force line-buffered stdout"
$launcherPid = $launcher.Id
Write-State "launcher cmd.exe PID = $launcherPid"

# --- Watchdog loop -----------------------------------------------------------
$deadline = $startTime.AddMinutes([math]::Ceiling($ExpectedDurationMin * 1.5))
Write-State "deadline = $($deadline.ToString('yyyy-MM-dd HH:mm:ss'))  (= 1.5x ExpectedDurationMin)"

$lastCpu = 0.0
$cpuFlatTicks = 0
$result = 'unknown'
$resultDetails = ''
$tickN = 0

while ($true) {
    Start-Sleep -Seconds $PollSec
    $tickN++
    $now = Get-Date

    # Process state
    $launcherAlive = $false
    try { Get-Process -Id $launcherPid -ErrorAction Stop | Out-Null; $launcherAlive = $true } catch { $launcherAlive = $false }
    $myPython = Find-ChildPython -LauncherPid $launcherPid
    $pyCount = ($myPython | Measure-Object).Count
    $pyCpu = ($myPython | Measure-Object -Property CPU -Sum).Sum
    if ($null -eq $pyCpu) { $pyCpu = 0.0 }
    # WMI doesn't expose CPU directly the same way; use Get-Process for CPU
    $pyProcLive = $myPython | ForEach-Object {
        try { Get-Process -Id $_.ProcessId -ErrorAction Stop } catch { $null }
    } | Where-Object { $null -ne $_ }
    $pyCpuSec = ($pyProcLive | Measure-Object -Property CPU -Sum).Sum
    if ($null -eq $pyCpuSec) { $pyCpuSec = 0.0 }
    $pyWsMb = ($pyProcLive | Measure-Object -Property WorkingSet64 -Sum).Sum / 1MB
    if ($null -eq $pyWsMb) { $pyWsMb = 0 }

    # Log stats
    $logExists = Test-Path $LogFile
    $logSize = if ($logExists) { (Get-Item $LogFile).Length } else { 0 }
    $logLwt = if ($logExists) { (Get-Item $LogFile).LastWriteTime } else { $null }

    # GPU
    $gpu = Get-GpuStats

    # Tick status line
    Write-State ("tick #{0} t+{1}s  launcher={2}  py={3} cpu={4:N1}s ws={5:N1}MB  log={6}B lwt={7}  gpu={8}% mem={9}MiB" -f
        $tickN, [math]::Round(($now-$startTime).TotalSeconds), $launcherAlive,
        $pyCount, $pyCpuSec, $pyWsMb,
        $logSize, ($(if ($logLwt) { $logLwt.ToString('HH:mm:ss') } else { 'na' })),
        $gpu.util_pct, $gpu.mem_used)

    # Terminal markers in log tail (last 200 lines)
    if ($logExists -and $logSize -gt 0) {
        $tail = Get-Content -Path $LogFile -Tail 200 -ErrorAction SilentlyContinue
        if ($tail) {
            $err = Test-MarkerIn -Lines $tail -Markers $errorMarkers
            if ($err) {
                Write-State "ERROR_MARKER: $($err.marker) :: $($err.line)"
                # Kill PID-narrowed
                foreach ($py in $myPython) {
                    Write-State "stopping python PID $($py.ProcessId) (error)"
                    try { Stop-Process -Id $py.ProcessId -Force -ErrorAction SilentlyContinue } catch {}
                }
                if ($launcherAlive) { try { Stop-Process -Id $launcherPid -Force -ErrorAction SilentlyContinue } catch {} }
                $result = 'error'; $resultDetails = "$($err.marker): $($err.line)"
                break
            }
            $suc = Test-MarkerIn -Lines $tail -Markers $successMarkers
            if ($suc) {
                Write-State "SUCCESS_MARKER: $($suc.marker) :: $($suc.line)"
                # Wait up to 10s for clean exit
                $waitDeadline = (Get-Date).AddSeconds(10)
                $cleanExit = $false
                while ((Get-Date) -lt $waitDeadline) {
                    Start-Sleep -Seconds 2
                    $remaining = Find-ChildPython -LauncherPid $launcherPid
                    if (-not $remaining -or ($remaining | Measure-Object).Count -eq 0) {
                        $cleanExit = $true; break
                    }
                }
                if (-not $cleanExit) {
                    Write-State "shutdown hang detected (success marker but child python alive after 10s). Killing PID-narrowed set."
                    $myPython2 = Find-ChildPython -LauncherPid $launcherPid
                    foreach ($py in $myPython2) {
                        Write-State "stopping python PID $($py.ProcessId) (hang on success)"
                        try { Stop-Process -Id $py.ProcessId -Force -ErrorAction SilentlyContinue } catch {}
                    }
                    Start-Sleep -Seconds 3
                }
                if ($launcherAlive) { try { Stop-Process -Id $launcherPid -Force -ErrorAction SilentlyContinue } catch {} }

                # Verify expected report
                if ($ExpectedReport -and -not (Test-Path $ExpectedReport)) {
                    Write-State "WARNING: success marker hit but ExpectedReport '$ExpectedReport' not found"
                    $result = 'success_but_no_report'
                } else {
                    $result = 'success'
                }
                $resultDetails = "$($suc.marker): $($suc.line)"
                break
            }
        }
    }

    # Silent-stall detection
    if ($logLwt) {
        $logAgeMin = ($now - $logLwt).TotalMinutes
        $cpuDelta = $pyCpuSec - $lastCpu
        $cpuFlat = $cpuDelta -lt 5
        if ($cpuFlat) { $cpuFlatTicks++ } else { $cpuFlatTicks = 0 }
        if ($logAgeMin -gt $StaleLogMin -and $gpu.util_pct -ge 0 -and $gpu.util_pct -lt 5 -and $cpuFlat) {
            Write-State "SILENT_STALL: log stale $([math]::Round($logAgeMin,1)) min, GPU $($gpu.util_pct)%, py_cpu Δ$([math]::Round($cpuDelta,1))s. Killing PID-narrowed set."
            foreach ($py in $myPython) {
                Write-State "stopping python PID $($py.ProcessId) (stall)"
                try { Stop-Process -Id $py.ProcessId -Force -ErrorAction SilentlyContinue } catch {}
            }
            if ($launcherAlive) { try { Stop-Process -Id $launcherPid -Force -ErrorAction SilentlyContinue } catch {} }
            $result = 'hang'
            $resultDetails = "log stale $([math]::Round($logAgeMin,1)) min, GPU $($gpu.util_pct)%, py_cpu Δ$([math]::Round($cpuDelta,1))s"
            break
        }
    }

    # Deadline
    if ($now -gt $deadline) {
        Write-State "DEADLINE_EXCEEDED: $($deadline.ToString('HH:mm:ss')). Killing PID-narrowed set."
        foreach ($py in $myPython) {
            Write-State "stopping python PID $($py.ProcessId) (deadline)"
            try { Stop-Process -Id $py.ProcessId -Force -ErrorAction SilentlyContinue } catch {}
        }
        if ($launcherAlive) { try { Stop-Process -Id $launcherPid -Force -ErrorAction SilentlyContinue } catch {} }
        $result = 'deadline'
        $resultDetails = "exceeded ${ExpectedDurationMin}min × 1.5"
        break
    }

    # Process died unexpectedly (no markers, no stall, just gone)
    if (-not $launcherAlive -and $pyCount -eq 0) {
        Write-State "process exited without marker (launcher gone, no python). Checking for late-flush markers."
        # One last log check after a small grace period
        Start-Sleep -Seconds 3
        $tail2 = Get-Content -Path $LogFile -Tail 200 -ErrorAction SilentlyContinue
        $err2 = if ($tail2) { Test-MarkerIn -Lines $tail2 -Markers $errorMarkers } else { $null }
        $suc2 = if ($tail2 -and -not $err2) { Test-MarkerIn -Lines $tail2 -Markers $successMarkers } else { $null }
        if ($err2) {
            $result = 'error_post_exit'; $resultDetails = "$($err2.marker): $($err2.line)"
        } elseif ($suc2) {
            if ($ExpectedReport -and -not (Test-Path $ExpectedReport)) {
                $result = 'success_but_no_report'
            } else {
                $result = 'success_post_exit'
            }
            $resultDetails = "$($suc2.marker): $($suc2.line)"
        } else {
            $result = 'unexpected_exit'; $resultDetails = "process gone, no markers"
        }
        break
    }

    $lastCpu = $pyCpuSec
}

# Final GPU check after kill
Start-Sleep -Seconds 2
$gpuFinal = Get-GpuStats
Write-State "FINAL GPU mem_used=$($gpuFinal.mem_used)MiB free=$($gpuFinal.mem_free)MiB"
Write-State "RESULT result=$result  details='$resultDetails'  duration=$([math]::Round(((Get-Date)-$startTime).TotalMinutes,1))min"
Write-State "=========================================="

if ($result -eq 'success' -or $result -eq 'success_post_exit') { exit 0 } else { exit 1 }
