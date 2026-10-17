# Pass 12 B4 RELAXED — recommended training config

For all future Pass 12 B4 RELAXED training, **use the PPO-E config**:

```
tasks\agents\rl_games_stage_b_smoke_pass12_B4_RECOMMENDED.yaml
```

This is a copy of `tasks\agents\rl_games_stage_b_smoke_pass12_ppo_miniepochs2_eclip01.yaml` (the canonical PPO-E variant). Both files have identical contents. The `_RECOMMENDED` filename is the human-readable alias for new training.

The only knobs that differ from the old default `rl_games_stage_b_smoke_cfg.yaml`:
- `mini_epochs: 2` (was 4)
- `e_clip: 0.1` (was 0.2)

Everything else (learning rate, kl_threshold, network, sigma init, ...) is unchanged. See `outputs\analysis\ppo_E_adoption_note.md` for the full rationale and validation evidence.

## Standard training command

```powershell
.\IsaacLab\isaaclab.bat -p scripts\rl\train_stage_b_smoke.py `
  --num-envs 16 --max-iterations 100 --headless --seed 42 `
  --pass11-cot --pass12-cot-bounded --pass12-functional-guard `
  --checkpoint outputs\rl_smoke\stage_b_transition_global6d\2026-05-08_15-27-26\nn\centipede_stage_b_transition_global6d_smoke.pth `
  --out-dir outputs\rl_smoke\<your-run-name> `
  --agent-cfg tasks\agents\rl_games_stage_b_smoke_pass12_B4_RECOMMENDED.yaml
```

## What this fixes

The default PPO config let realized KL run 38-108× the configured kl_threshold (= 0.005) during overnight training, which produced a 41 % reward best→final drift in the 100-iter overnight run. PPO-E reduced this to 4.7 % at 100 iters — a 89 % reduction in drift — and produced a new all-time best reward of −15.021 at ep_55 of the validation run. See `outputs\analysis\ppo_drift_diagnosis.md` and `outputs\analysis\NEXT_STAGE_AUTONOMOUS_RESULTS_REPORT.md` for full evidence.

## What stays unchanged

- **Pass 12 B4 RELAXED reward**: do NOT modify reward without explicit ChatGPT review.
- **Old default PPO config** (`rl_games_stage_b_smoke_cfg.yaml`): preserved for reproducibility of pre-2026-05-10 runs.
- **Pass 8 reference checkpoint** (`outputs\rl_smoke\stage_b_transition_global6d\2026-05-08_15-27-26\nn\centipede_stage_b_transition_global6d_smoke.pth`): preserved as warm-start anchor.
