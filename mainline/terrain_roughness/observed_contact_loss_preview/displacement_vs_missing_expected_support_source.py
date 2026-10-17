"""Requested contact-count replot of existing results; no simulator or fitting."""
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
from matplotlib.ticker import FuncFormatter, MultipleLocator

ROOT = Path("X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness")
OUT = ROOT / "observed_contact_loss_preview"
INPUT = OUT / "PREVIEW.json"
STEM = "displacement_vs_missing_expected_support"
WAVES = [1.5, 2, 2.5, 3, 3.5]
COLORS = ["#2474b5", "#df7c13", "#26915a", "#bd4545", "#8862b2"]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(values):
    return {
        "n": len(values),
        "mean": float(np.mean(values)),
        "std": float(np.std(values, ddof=1)) if len(values) > 1 else None,
        "values": [float(value) for value in values],
    }


data = json.loads(INPUT.read_text(encoding="utf-8"))
assert data["new_physics_steps"] == 0
assert data["sample_dt_s"] == 0.01
for path, digest in data["source_hashes"].items():
    assert sha(path) == digest, path
for suffix in (".png", ".svg", ".json", "_source.py"):
    assert not (OUT / (STEM + suffix)).exists(), "Refuse to overwrite earlier output"

index = {(c["wave"], c["height_scale"], c["spawn_index"]): c for c in data["cells"]}
groups = []
for summary in data["summaries"]:
    wave, height = summary["wave"], summary["height_scale"]
    trials = [index[(wave, height, spawn)] for spawn in summary["retained_spawn_indices"]]
    missing, forward = [], []
    expected = set()
    for trial in trials:
        window = trial["windows"]["0_6"]
        assert window["missing_samples"] + window["retained_samples"] == window["expected_support_samples"]
        assert abs(window["loss"] * window["expected_support_samples"] - window["missing_samples"]) < 1e-9
        expected.add(window["expected_support_samples"])
        missing.append(window["missing_samples"])
        forward.append(trial["forward_0_6_mm"])
    assert len(expected) == 1
    xs, ys = stats(missing), stats(forward)
    original = summary["original_forward_0_6_mm"]
    assert xs["n"] == summary["n"]
    np.testing.assert_allclose(ys["mean"], original["mean"], atol=1e-10, rtol=0)
    if ys["std"] is not None:
        np.testing.assert_allclose(ys["std"], original["std"], atol=1e-10, rtol=0)
    np.testing.assert_allclose(xs["mean"] / next(iter(expected)), summary["contact_loss_0_6"]["mean"], atol=1e-12, rtol=0)
    if height == 0:
        assert xs["mean"] == 0 and xs["std"] is None
    groups.append({
        "wave": wave, "height_scale_source_condition": height,
        "retained_spawn_indices": summary["retained_spawn_indices"],
        "expected_support_samples_in_flat_reference": next(iter(expected)),
        "missing_expected_support_samples": xs,
        "forward_displacement_mm": ys,
    })

assert len(groups) == 35 and sum(g["missing_expected_support_samples"]["n"] for g in groups) == 267
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 12, "svg.fonttype": "none"})
fig, ax = plt.subplots(figsize=(10.7, 6.9))
fig.subplots_adjust(left=0.11, right=0.98, bottom=0.22, top=0.82)
for wave, color in zip(WAVES, COLORS):
    rows = sorted((g for g in groups if g["wave"] == wave), key=lambda g: g["height_scale_source_condition"])
    xx = [g["missing_expected_support_samples"]["mean"] for g in rows]
    yy = [g["forward_displacement_mm"]["mean"] for g in rows]
    xsd = [g["missing_expected_support_samples"]["std"] or 0 for g in rows]
    ysd = [g["forward_displacement_mm"]["std"] or 0 for g in rows]
    ax.errorbar(xx, yy, xerr=xsd, yerr=ysd, fmt="none", ecolor=color, alpha=0.23,
                elinewidth=1.5, capsize=3, capthick=1.3, zorder=1)
    ax.plot(xx, yy, "-o", color=color, lw=2.1, ms=5.8, label=f"Wave {wave:g}", zorder=3)
ax.axhline(0, color="#818891", lw=0.9, alpha=0.45, zorder=0)
ax.grid(alpha=0.18)
ax.set_axisbelow(True)
ax.set_xlim(-120, 7100)
ax.set_ylim(-20, 245)
ax.xaxis.set_major_locator(MultipleLocator(1000))
ax.xaxis.set_major_formatter(FuncFormatter(lambda value, pos: f"{value:,.0f}"))
ax.set_xlabel("Missing expected support samples (count, 0–6 s)", labelpad=10)
ax.set_ylabel("Forward COM displacement (mm, 0–6 s)", labelpad=10)
for side in ("top", "right"):
    ax.spines[side].set_visible(False)
fig.suptitle("Displacement vs. missing expected support", y=0.965, fontsize=17)
fig.legend(*ax.get_legend_handles_labels(), loc="upper center", bbox_to_anchor=(0.54, 0.91),
           ncol=5, frameon=False, handlelength=2, columnspacing=1.25)
fig.text(0.11, 0.104, "Each point: group mean. Pale horizontal / vertical bars: sample SD.", fontsize=10.5)
fig.text(0.11, 0.054, "One support sample = one foot at one 10 ms snapshot; same-phase flat reference.", fontsize=10.5)
fig.savefig(OUT / (STEM + ".png"), dpi=180)
fig.savefig(OUT / (STEM + ".svg"))
plt.close(fig)

(OUT / (STEM + "_source.py")).write_bytes(Path(__file__).read_bytes())
receipt = {
    "status": "existing_data_replot_complete",
    "new_physics_steps": 0,
    "input": {"path": str(INPUT), "sha256": sha(INPUT)},
    "verified_source_files": len(data["source_hashes"]),
    "window_s": [0, 6], "contact_snapshot_dt_s": 0.01,
    "contact_force_threshold_N": data["threshold_N"],
    "flat_reference": "One existing flat trial per wave; matching commanded CPG clock and foot ID.",
    "x_axis": "Number of foot-time samples with flat contact and absent rough contact, not a percentage.",
    "y_axis": "Six-second mass-weighted COM displacement projected on each trial's initial heading, mm.",
    "statistics": "Same 267 retained trials and per-group omissions as original displacement graph; mean and sample SD (ddof=1) in both axes. Single flat reference has no estimated SD.",
    "curve_order": "Source height conditions in increasing order, without showing terrain roughness on either axis; straight segments join measured group means, no curve fitting.",
    "interpretation": "Crossings here compare wave curves at observed missing-sample counts; they are not automatically the same shared-terrain crossovers from the earlier roughness-axis graph.",
    "flat_expected_sample_counts": {str(w): next(g["expected_support_samples_in_flat_reference"] for g in groups if g["wave"] == w) for w in WAVES},
    "groups": groups,
    "checks": {"source_hashes_match": True, "support_sample_partition": True,
               "original_displacement_mean_sd_unchanged": True, "count_to_fraction_identity": True,
               "group_count": len(groups), "retained_trial_count": 267,
               "raw_complete_trial_count": data["raw_complete_trials"],
               "existing_failed_trials_excluded_from_six_second_scores": data["raw_numerical_failure_count"]},
    "visual_review": "pending",
    "outputs": {name: {"path": str(OUT / (STEM + suffix)), "sha256": sha(OUT / (STEM + suffix))}
                for name, suffix in [("png", ".png"), ("svg", ".svg"), ("executed_source", "_source.py")]},
}
(OUT / (STEM + ".json")).write_text(json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
print(json.dumps({"outputs": [str(OUT / (STEM + suffix)) for suffix in (".png", ".svg", ".json")],
                  "groups": len(groups), "retained_trials": 267,
                  "flat_expected_sample_counts": receipt["flat_expected_sample_counts"]}))
