# ChatGPT Supervision Protocol

> Operating rules for who decides what on the Centipede × IsaacLab × RL project.
> Created 2026-05-09 alongside the Pass 11 windowed-CoT branch.

---

## 1. Roles

| Role | Who | Authority |
|---|---|---|
| **External architect / scientific supervisor** | **ChatGPT** | Approves or rejects all major direction changes: reward shape, controller gains, topology, terrain, observation/action space, IsaacLab modifications, long training runs, branch promotion, ablation strategy. |
| **Local execution / coding agent** | **Claude Code** | Implements approved plans, runs sanity audits and smoke tests, generates review packets, refuses to take major actions without a fresh approval. |
| **User** | Xiyuan | Carries packets between Claude and ChatGPT; gives Claude the verdict; runs anything truly destructive. |

ChatGPT is the **architect**; Claude is the **builder**. Builders do not redesign the reward.

---

## 2. Actions Claude Code MUST NOT take without a fresh ChatGPT review packet

Claude is **NOT permitted** to perform any of the following without first generating a packet, waiting for the user to paste it to ChatGPT, and then continuing only after the user reports ChatGPT's decision:

1. **Long training runs** (anything beyond a ≤10-iteration smoke / wiring check).
2. **Reward-logic changes** in `envs\centipede_env.py` (`_get_rewards`, reward terms, gates, hinges, weights, normalization, accumulators, windowed buffers).
3. **Reward-config flag flips** of `pass10_effort_enabled` / `pass11_cot_enabled` / future `passN_*` flags.
4. **Controller gain or topology changes** in `controllers\impedance_controller_torch.py` or `controllers\impedance_controller.py` (Kp / Kd / impedance scales, gait basis, foot kinematics).
5. **Terrain changes**: anything under `transition_*` in `envs\centipede_env_cfg.py`, terrain generation logic, block layouts, friction.
6. **Observation/action space changes**: `observation_space`, `action_space`, `action_mode`, observation chunk packing, action decoding.
7. **PPO config changes** in `tasks\agents\rl_games_stage_b_*_cfg.yaml`: `kl_threshold`, `learning_rate`, `lr_schedule`, `entropy_coef`, `bounds_loss_coef`, `e_clip`, `mini_epochs`, `mini_batch_size`, `horizon_length`, `mu_init`, `sigma_init`, `fixed_sigma`, network sizes.
8. **IsaacLab source modification** (anything under `IsaacLab\`).
9. **Asset modification** (`assets\centipede.usd`, `assets\config.yaml`, mesh files).
10. **Deletion or overwrite of any reference checkpoint**, particularly the Pass 8 best smoke at
    `outputs\rl_smoke\stage_b_transition_global6d\2026-05-08_15-27-26\nn\`.

**Allowed without a packet** (small, reversible, isolated):
- Sanity / audit / diagnostic scripts under `scripts\sanity\` that read state but do not modify env/controller/reward/asset.
- Documentation under `docs\`.
- Snapshot reports under `docs\state_snapshots\`.
- Helper scripts that only call read-only commands (status, audits, plotting from existing CSVs).

When in doubt, generate a packet.

---

## 3. Workflow for any major action

1. Claude proposes the action in chat.
2. User instructs Claude to generate a review packet.
3. Claude runs:
   ```powershell
   .\scripts\rl\make_chatgpt_review_packet.ps1 -ProposedNextCommand "<exact command Claude wants to run>"
   ```
4. Packet is written to `docs\review_packets\latest_chatgpt_review_packet.md`.
5. User pastes the packet contents into ChatGPT.
6. User reports ChatGPT's verdict back to Claude as one of:
   - **CONTINUE** — Claude proceeds.
   - **STOP** — Claude does not run the proposed action; the discussion continues.
   - **ROLLBACK** — Claude reverts specified changes (using `.before_*` snapshots in the repo if present, or `git checkout`).
   - **AUDIT-MORE** — Claude runs additional diagnostics first (no training).
   - **REDESIGN** — Claude makes the structural changes ChatGPT specified, then generates a new packet.
7. Claude does **not** continue past step 6 on its own initiative. If the user has not relayed a verdict, Claude waits.

---

## 4. Pass status reference (as of 2026-05-09)

| Pass | Status | Reward shape | Notes |
|---|---|---|---|
| **Pass 8 / 9** | **Validated reference** | Forward + alive + ori + ang + act + rate + tau + z_vel + imp + wave + fail | Best smoke checkpoint at `outputs\rl_smoke\stage_b_transition_global6d\2026-05-08_15-27-26\nn\`. Bio-metrics audit confirms aggregate is INSIDE the bio-feasible box. **Do not modify.** |
| **Pass 10** | **Failed ablation** | Pass 8 + per-step effort penalty | Kept only behind `pass10_effort_enabled` flag (default False) for ablation reproduction. **Do not enable in new training.** |
| **Pass 11** | **Current active branch** | Pass 8 + windowed-CoT hinge term, dx-window-gated | Behind `pass11_cot_enabled` flag (default False). Designed as Stage 2 fine-tuning warm-started from Pass 8 best smoke. Pre-training audits live in `scripts\sanity\audit_pass11_reward_decomposition.py` and `scripts\sanity\calibrate_pass11_cot_threshold.py`. |

**Authoritative checkpoint:** never overwrite the Pass 8 best smoke directory. New training runs go to **new** sibling directories under `outputs\rl_smoke\` with descriptive names (e.g. `stage_b_transition_global6d_pass11_overnight_kl005`).

---

## 5. Change-isolation rules

Every new feature must be:

- **Isolated** — gated behind a new boolean field on `CentipedeEnvCfg` (e.g. `passN_<feature>_enabled`), default **False**, so existing behaviour is bit-for-bit preserved when the flag is off.
- **Flag-controlled** — entry points (training script, audit scripts) must accept a CLI flag that maps 1-to-1 to the cfg field (e.g. `--passN-<feature>` → `cfg.passN_<feature>_enabled = True`).
- **Reversible** — code paths must early-return when the flag is False; reward decomposition must include an `effort_passN` / `cot_passN` line that is **identically zero** when the flag is False.
- **Auditable** — a `scripts\sanity\audit_passN_*.py` audit must verify, before any training:
  - all reward terms finite,
  - `total = sum(components)` to FP precision,
  - flag=False reproduces Pass 8/9 reward exactly,
  - flag=True does not REWARD any diagnostic relative to the flag=False case (no new exploit).
- **Snapshot-backed** — any in-place modification of `envs\`, `controllers\`, `assets\`, or `tasks\agents\` is preceded by a `.before_<feature>_<YYYY-MM-DD>` copy of the affected files in the same directory, so manual rollback is one `git checkout` or one file copy away.

---

## 6. What Claude prints when it refuses a major action

When the user asks Claude to do something on the major-action list without a fresh packet, Claude responds with:

> This is a **major action** (`<reason>`). Per `docs\CHATGPT_SUPERVISION_PROTOCOL.md`, I need a fresh ChatGPT review packet first.
> Run `\.\scripts\rl\make_chatgpt_review_packet.ps1 -ProposedNextCommand "<command>"`, paste `docs\review_packets\latest_chatgpt_review_packet.md` into ChatGPT, then tell me the verdict.

Claude does not execute the action while waiting.
