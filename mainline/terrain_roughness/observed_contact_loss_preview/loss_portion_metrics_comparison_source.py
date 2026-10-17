"""Three requested aggregation views of already completed contact trajectories."""
from pathlib import Path
import os
import json
import hashlib

for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[key] = "1"

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, PercentFormatter

ROOT = Path("X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness")
OUT = ROOT / "observed_contact_loss_preview"
INPUT = OUT / "PREVIEW.json"
PREVIOUS = OUT / "displacement_vs_instantaneous_loss_portion.json"
STEM = "loss_portion_metrics_comparison"
WAVES = [1.5, 2, 2.5, 3, 3.5]
COLORS = ["#2474b5", "#df7c13", "#26915a", "#bd4545", "#8862b2"]
METRICS = {
    "instantaneous": {"title": "A. Average per 10 ms snapshot", "weight": "Each valid snapshot has equal weight",
                      "formula": "100 * mean_t(missing_feet[t]/expected_flat_feet[t]); skip expected=0",
                      "file_stem": "displacement_vs_instantaneous_loss_portion"},
    "cycle": {"title": "B. Average per 1 s CPG cycle", "weight": "Each of the six cycles has equal weight",
              "formula": "100 * mean_j(sum_t_in_cycle_j(missing_feet[t])/sum_t_in_cycle_j(expected_flat_feet[t]))",
              "file_stem": "displacement_vs_cycle_loss_portion"},
    "pooled": {"title": "C. Whole-window ratio", "weight": "Weight by expected contact samples",
               "formula": "100 * sum_t(missing_feet[t])/sum_t(expected_flat_feet[t])",
               "file_stem": "displacement_vs_pooled_loss_portion"},
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def stats(values):
    return {"n": len(values), "mean": float(np.mean(values)),
            "std": float(np.std(values, ddof=1)) if len(values) > 1 else None,
            "values": [float(v) for v in values]}


data = read(INPUT)
previous = read(PREVIOUS)
assert previous["input"]["sha256"] == sha(INPUT)
for path, digest in data["source_hashes"].items():
    assert sha(path) == digest, path
for stem in [STEM, METRICS["cycle"]["file_stem"], METRICS["pooled"]["file_stem"]]:
    for suffix in (".json", ".png", ".svg", "_source.py"):
        assert not (OUT / (stem + suffix)).exists(), "Refuse to overwrite earlier comparison"

sources = {str(INPUT): sha(INPUT), str(PREVIOUS): sha(PREVIOUS)}
old_raw = {(c["wave"], c["height_scale_source_condition"], c["spawn_index"]): c
           for c in previous["raw_complete_trials"]}
cache = {}


def contact(cell):
    path = str(Path(cell["run_path"]) / "cluster_contacts.npz")
    assert path in data["source_hashes"]
    if path not in cache:
        with np.load(path) as z:
            assert z["foot_names"].tolist() == [f"foot_{i//2}_{i%2}" for i in range(38)]
            t, force = z["t"].copy(), z["force"].copy()
        assert np.isfinite(force).all()
        np.testing.assert_allclose(t, np.arange(1, 601) / 100, atol=1e-12, rtol=0)
        cache[path] = (t, force > data["threshold_N"])
    t, present = cache[path]
    return t, present[:, cell["environment_index"], :]


flats = {c["wave"]: c for c in data["cells"] if c["height_scale"] == 0}
refs = {w: contact(flats[w]) for w in WAVES}
period_evidence = []
command_cache = {}
for wave in WAVES:
    manifest_path = Path(flats[wave]["run_path"]) / "manifest.json"
    manifest = read(manifest_path)
    row = manifest["rows"][flats[wave]["environment_index"]]
    assert row["wave"] == wave
    command_path = str(Path(row["command_file"]))
    if command_path not in command_cache:
        assert sha(command_path) == row["command_sha256"]
        with np.load(command_path) as z:
            assert float(z["dt_s"]) == 0.001
            joint = z["joint_names"].tolist().index("legbase_pitch_9_L")
            command_cache[command_path] = z["position"][6000:8000, :, joint].copy()
        sources[command_path] = row["command_sha256"]
    signal = command_cache[command_path][:, row["command_index"]]
    frequency = float(np.fft.rfftfreq(2000, .001)[np.argmax(abs(np.fft.rfft(signal - signal.mean())))])
    assert frequency == 1.0
    sources[str(manifest_path)] = sha(manifest_path)
    period_evidence.append({"wave": wave, "command_file": command_path,
                            "command_index": row["command_index"], "dominant_leg_target_Hz": frequency})

raw = []
for cell in data["cells"]:
    t, observed = contact(cell)
    tf, expected = refs[cell["wave"]]
    np.testing.assert_array_equal(t, tf)
    e = expected.sum(axis=1)
    m = (expected & ~observed).sum(axis=1)
    assert np.all(m <= e)
    valid = e > 0
    instantaneous = float((m[valid] / e[valid]).mean())
    cycles = []
    for j in range(6):
        select = (t > j + 1e-10) & (t <= j + 1 + 1e-10)
        assert int(select.sum()) == 100
        denominator, numerator = int(e[select].sum()), int(m[select].sum())
        assert denominator > 0
        cycles.append({"interval_s": [j, j + 1], "missing_samples": numerator,
                       "expected_samples": denominator, "fraction": numerator / denominator})
    cycle_mean = float(np.mean([cycle["fraction"] for cycle in cycles]))
    pooled = float(m.sum() / e.sum())
    np.testing.assert_allclose(np.average([c["fraction"] for c in cycles],
                                       weights=[c["expected_samples"] for c in cycles]), pooled, atol=1e-12)
    key = (cell["wave"], cell["height_scale"], cell["spawn_index"])
    np.testing.assert_allclose(instantaneous, old_raw[key]["instantaneous_loss_mean_fraction"], atol=1e-12)
    np.testing.assert_allclose(pooled, cell["windows"]["0_6"]["loss"], atol=1e-12)
    metrics = {"instantaneous": 100 * instantaneous, "cycle": 100 * cycle_mean, "pooled": 100 * pooled}
    assert all(0 <= v <= 100 for v in metrics.values())
    if cell["height_scale"] == 0:
        assert all(v == 0 for v in metrics.values())
    raw.append({"wave": cell["wave"], "height_scale_source_condition": cell["height_scale"],
                "spawn_index": cell["spawn_index"], "metric_percent": metrics, "cycles": cycles,
                "valid_snapshot_count": int(valid.sum()),
                "forward_displacement_mm": cell["forward_0_6_mm"]})

index = {(c["wave"], c["height_scale_source_condition"], c["spawn_index"]): c for c in raw}
groups = []
for summary in data["summaries"]:
    wave, height = summary["wave"], summary["height_scale"]
    trials = [index[(wave, height, spawn)] for spawn in summary["retained_spawn_indices"]]
    ys = stats([c["forward_displacement_mm"] for c in trials])
    np.testing.assert_allclose(ys["mean"], summary["original_forward_0_6_mm"]["mean"], atol=1e-10, rtol=0)
    if ys["std"] is not None:
        np.testing.assert_allclose(ys["std"], summary["original_forward_0_6_mm"]["std"], atol=1e-10, rtol=0)
    groups.append({"wave": wave, "height_scale_source_condition": height,
                   "retained_spawn_indices": summary["retained_spawn_indices"],
                   "forward_displacement_mm": ys,
                   "metric_percent": {k: stats([c["metric_percent"][k] for c in trials]) for k in METRICS}})
assert len(raw) == 297 and len(groups) == 35
assert sum(g["forward_displacement_mm"]["n"] for g in groups) == 267

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11.5, "svg.fonttype": "none"})


def draw(ax, metric, comparison=False):
    for wave, color in zip(WAVES, COLORS):
        rows = sorted((g for g in groups if g["wave"] == wave), key=lambda g: g["height_scale_source_condition"])
        xx = [g["metric_percent"][metric]["mean"] for g in rows]
        yy = [g["forward_displacement_mm"]["mean"] for g in rows]
        xsd = [g["metric_percent"][metric]["std"] or 0 for g in rows]
        ysd = [g["forward_displacement_mm"]["std"] or 0 for g in rows]
        ax.errorbar(xx, yy, xerr=xsd, yerr=ysd, fmt="none", ecolor=color, alpha=.23,
                    elinewidth=1.3, capsize=2.5, capthick=1.2, zorder=1)
        ax.plot(xx, yy, "-o", color=color, lw=2, ms=4.6 if comparison else 5.8, label=f"Wave {wave:g}", zorder=3)
    ax.axhline(0, color="#818891", lw=.9, alpha=.45, zorder=0)
    ax.grid(alpha=.18)
    ax.set_axisbelow(True)
    ax.set_xlim(-1.5, 100)
    ax.set_ylim(-20, 245)
    ax.xaxis.set_major_locator(MultipleLocator(20 if comparison else 10))
    ax.xaxis.set_major_formatter(PercentFormatter(xmax=100, decimals=0))
    ax.set_xlabel("Expected-support loss (%)", labelpad=9)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


outputs = {}
for metric in ("cycle", "pooled"):
    fig, ax = plt.subplots(figsize=(10.7, 6.9))
    fig.subplots_adjust(left=.11, right=.955, bottom=.24, top=.8)
    draw(ax, metric)
    ax.set_ylabel("Forward COM displacement (mm, 0–6 s)", labelpad=10)
    fig.suptitle(METRICS[metric]["title"], y=.955, fontsize=17)
    fig.legend(*ax.get_legend_handles_labels(), loc="upper center", bbox_to_anchor=(.54, .9), ncol=5, frameon=False)
    fig.text(.11, .13, METRICS[metric]["weight"] + "; same 0–6 s window.", fontsize=10.5)
    fig.text(.11, .07, "Same retained trials and displacement; pale bars = sample SD in both axes.", fontsize=10.5)
    stem = METRICS[metric]["file_stem"]
    for suffix in (".png", ".svg"):
        path = OUT / (stem + suffix)
        fig.savefig(path, dpi=180)
        outputs[metric + suffix] = {"path": str(path), "sha256": sha(path)}
    plt.close(fig)

for suffix in (".png", ".svg"):
    path = OUT / (METRICS["instantaneous"]["file_stem"] + suffix)
    assert sha(path) == previous["outputs"]["png" if suffix == ".png" else "svg"]["sha256"]
    outputs["instantaneous" + suffix] = {"path": str(path), "sha256": sha(path), "reused": True}

fig, axs = plt.subplots(1, 3, figsize=(18.5, 6.7), sharey=True)
fig.subplots_adjust(left=.055, right=.975, bottom=.22, top=.72, wspace=.12)
for ax, (metric, definition) in zip(axs, METRICS.items()):
    draw(ax, metric, comparison=True)
    ax.set_title(definition["title"] + "\n" + definition["weight"], fontsize=11.5, pad=13)
axs[0].set_ylabel("Forward COM displacement (mm)", labelpad=9)
fig.suptitle("Displacement vs. contact loss: three aggregation choices", y=.965, fontsize=17)
fig.legend(*axs[0].get_legend_handles_labels(), loc="upper center", bbox_to_anchor=(.5, .91), ncol=5, frameon=False)
fig.text(.055, .115, "Identical 0–6 s displacement, 267 retained trials, fixed amplitudes / PD. Pale bars: sample SD in both axes.", fontsize=11)
fig.text(.055, .055, "A: average the instantaneous fractions. B: pool within each 1 s interval, then average six fractions. C: pool all samples.", fontsize=10.5)
for suffix in (".png", ".svg"):
    path = OUT / (STEM + suffix)
    fig.savefig(path, dpi=180)
    outputs["comparison" + suffix] = {"path": str(path), "sha256": sha(path)}
plt.close(fig)
source_path = OUT / (STEM + "_source.py")
source_path.write_bytes(Path(__file__).read_bytes())
outputs["executed_source"] = {"path": str(source_path), "sha256": sha(source_path)}

report = {
    "status": "requested_three_metric_comparison_complete", "new_physics_steps": 0,
    "source_hashes": sources, "verified_native_contact_source_count": len(data["source_hashes"]),
    "window_s": [0, 6], "sample_dt_s": .01, "contact_force_threshold_N": data["threshold_N"],
    "definitions": METRICS,
    "nominal_cycle_boundaries_s": [[j, j+1] for j in range(6)],
    "cycle_boundary_convention": "100 saved snapshots in (j,j+1] per nominal1s CPG interval; all six included. No fitted shift or response-based alignment.",
    "period_reference": period_evidence,
    "statistics": "Same 267 original retained trials, original group means and sample SD of displacement. For each metric, first aggregate each trial then compute group mean/SD (ddof=1). One flat reference per wave, no estimated SD.",
    "checks": {"source_hashes_match": True, "instantaneous_metric_matches_selected_previous_plot": True,
               "pooled_metric_matches_original_preview": True, "weighted_cycle_identity_matches_pooled": True,
               "all_cycle_denominators_positive": True, "fractions_in_0_to_100_percent": True,
               "displacement_mean_sd_unchanged": True, "retained_trial_count": 267,
               "raw_complete_trials": 297, "raw_failed_trials_not_scored": data["raw_numerical_failure_count"]},
    "limitations": data["limitations"],
    "visual_review": "pending", "raw_complete_trials": raw, "groups": groups, "outputs": outputs,
}
(OUT / (STEM + ".json")).write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
print(json.dumps({"comparison": outputs["comparison.png"]["path"],
                  "height_1p5_group_loss_percent": {str(w): {k:v["mean"] for k,v in next(g["metric_percent"] for g in groups if g["wave"] == w and g["height_scale_source_condition"] == 1.5).items()} for w in WAVES}}, ensure_ascii=False))
