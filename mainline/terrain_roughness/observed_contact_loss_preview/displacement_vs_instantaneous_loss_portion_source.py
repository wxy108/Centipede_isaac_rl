"""User-selected mean of instantaneous missing-foot portions; existing data only."""
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
STEM = "displacement_vs_instantaneous_loss_portion"
WAVES = [1.5, 2, 2.5, 3, 3.5]
COLORS = ["#2474b5", "#df7c13", "#26915a", "#bd4545", "#8862b2"]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(values):
    return {
        "n": len(values), "mean": float(np.mean(values)),
        "std": float(np.std(values, ddof=1)) if len(values) > 1 else None,
        "values": [float(value) for value in values],
    }


data = json.loads(INPUT.read_text(encoding="utf-8"))
assert data["new_physics_steps"] == 0 and data["sample_dt_s"] == 0.01
for path, digest in data["source_hashes"].items():
    assert sha(path) == digest, path
prior_receipt = OUT / (STEM + ".json")
if prior_receipt.exists():
    prior = json.loads(prior_receipt.read_text(encoding="utf-8"))
    assert prior["input"]["sha256"] == sha(INPUT)
    assert prior["status"] == "user_selected_instantaneous_fraction_replot_complete"
    assert prior["user_selection"] == "每个10 ms时刻：缺失足数/该时刻flat接触足数，再对有效时刻平均"
else:
    for suffix in (".png", ".svg", "_source.py"):
        assert not (OUT / (STEM + suffix)).exists(), "Refuse to overwrite unrelated output"

cache = {}


def contact(cell):
    path = str(Path(cell["run_path"]) / "cluster_contacts.npz")
    assert path in data["source_hashes"]
    if path not in cache:
        with np.load(path) as z:
            assert z["foot_names"].tolist() == [f"foot_{i//2}_{i%2}" for i in range(38)]
            t = z["t"].copy()
            force = z["force"].copy()
        np.testing.assert_allclose(t, np.arange(1, 601) / 100, atol=1e-12, rtol=0)
        assert np.isfinite(force).all()
        cache[path] = (t, force > data["threshold_N"])
    t, contacts = cache[path]
    return t, contacts[:, cell["environment_index"], :]


flats = {c["wave"]: c for c in data["cells"] if c["height_scale"] == 0}
references = {w: contact(flats[w]) for w in WAVES}
raw = []
for cell in data["cells"]:
    t, observed = contact(cell)
    tf, expected = references[cell["wave"]]
    np.testing.assert_array_equal(t, tf)
    expected_count = expected.sum(axis=1)
    missing_count = (expected & ~observed).sum(axis=1)
    valid = expected_count > 0
    assert valid.any() and np.all(missing_count <= expected_count)
    fractions = missing_count[valid] / expected_count[valid]
    equal_time_mean = float(fractions.mean())
    pooled = float(missing_count.sum() / expected_count.sum())
    np.testing.assert_allclose(pooled, cell["windows"]["0_6"]["loss"], atol=1e-12, rtol=0)
    assert 0 <= equal_time_mean <= 1
    if cell["height_scale"] == 0:
        assert equal_time_mean == 0
    raw.append({
        "wave": cell["wave"], "height_scale_source_condition": cell["height_scale"],
        "spawn_index": cell["spawn_index"], "run_path": cell["run_path"],
        "environment_index": cell["environment_index"],
        "instantaneous_loss_mean_fraction": equal_time_mean,
        "instantaneous_loss_mean_percent": 100 * equal_time_mean,
        "instantaneous_loss_sum_fraction": float(fractions.sum()),
        "valid_snapshot_count": int(valid.sum()),
        "skipped_zero_expected_snapshot_count": int((~valid).sum()),
        "pooled_expected_sample_missing_fraction_comparison": pooled,
        "forward_displacement_mm": cell["forward_0_6_mm"],
    })

index = {(c["wave"], c["height_scale_source_condition"], c["spawn_index"]): c for c in raw}
groups = []
for summary in data["summaries"]:
    wave, height = summary["wave"], summary["height_scale"]
    trials = [index[(wave, height, spawn)] for spawn in summary["retained_spawn_indices"]]
    xs = stats([c["instantaneous_loss_mean_percent"] for c in trials])
    ys = stats([c["forward_displacement_mm"] for c in trials])
    old = summary["original_forward_0_6_mm"]
    np.testing.assert_allclose(ys["mean"], old["mean"], atol=1e-10, rtol=0)
    if ys["std"] is not None:
        np.testing.assert_allclose(ys["std"], old["std"], atol=1e-10, rtol=0)
    assert xs["n"] == summary["n"]
    groups.append({
        "wave": wave, "height_scale_source_condition": height,
        "retained_spawn_indices": summary["retained_spawn_indices"],
        "mean_instantaneous_loss_percent": xs, "forward_displacement_mm": ys,
        "old_pooled_missing_percent_for_comparison": 100 * summary["contact_loss_0_6"]["mean"],
    })

assert len(raw) == 297 and len(groups) == 35
assert sum(g["mean_instantaneous_loss_percent"]["n"] for g in groups) == 267
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 12, "svg.fonttype": "none"})
fig, ax = plt.subplots(figsize=(10.7, 7.1))
fig.subplots_adjust(left=0.11, right=0.955, bottom=0.27, top=0.82)
for wave, color in zip(WAVES, COLORS):
    rows = sorted((g for g in groups if g["wave"] == wave), key=lambda g: g["height_scale_source_condition"])
    xx = [g["mean_instantaneous_loss_percent"]["mean"] for g in rows]
    yy = [g["forward_displacement_mm"]["mean"] for g in rows]
    xsd = [g["mean_instantaneous_loss_percent"]["std"] or 0 for g in rows]
    ysd = [g["forward_displacement_mm"]["std"] or 0 for g in rows]
    ax.errorbar(xx, yy, xerr=xsd, yerr=ysd, fmt="none", ecolor=color, alpha=0.23,
                elinewidth=1.5, capsize=3, capthick=1.3, zorder=1)
    ax.plot(xx, yy, "-o", color=color, lw=2.1, ms=5.8, label=f"Wave {wave:g}", zorder=3)
ax.axhline(0, color="#818891", lw=0.9, alpha=0.45, zorder=0)
ax.grid(alpha=0.18)
ax.set_axisbelow(True)
ax.set_xlim(-1.5, 100)
ax.set_ylim(-20, 245)
ax.xaxis.set_major_locator(MultipleLocator(10))
ax.xaxis.set_major_formatter(PercentFormatter(xmax=100, decimals=0))
ax.set_xlabel("Mean instantaneous expected-support loss (%, 0–6 s)", labelpad=10)
ax.set_ylabel("Forward COM displacement (mm, 0–6 s)", labelpad=10)
for side in ("top", "right"):
    ax.spines[side].set_visible(False)
fig.suptitle("Displacement vs. expected-support loss portion", y=0.965, fontsize=17)
fig.legend(*ax.get_legend_handles_labels(), loc="upper center", bbox_to_anchor=(0.54, 0.91),
           ncol=5, frameon=False, handlelength=2, columnspacing=1.25)
fig.text(0.11, 0.15, "Each point: trial-group mean. Pale horizontal / vertical bars: sample SD.", fontsize=10.5)
fig.text(0.11, 0.10, "Each 10 ms snapshot: missing feet / expected flat-contact feet, then average.", fontsize=10.5)
fig.text(0.11, 0.05, "Snapshots with zero expected flat contact are omitted; each valid snapshot has equal weight.", fontsize=10.5)
fig.savefig(OUT / (STEM + ".png"), dpi=180)
fig.savefig(OUT / (STEM + ".svg"))
plt.close(fig)
(OUT / (STEM + "_source.py")).write_bytes(Path(__file__).read_bytes())

receipt = {
    "status": "user_selected_instantaneous_fraction_replot_complete",
    "user_selection": "每个10 ms时刻：缺失足数/该时刻flat接触足数，再对有效时刻平均",
    "new_physics_steps": 0, "input": {"path": str(INPUT), "sha256": sha(INPUT)},
    "verified_source_files": len(data["source_hashes"]), "window_s": [0, 6],
    "sample_dt_s": 0.01, "contact_force_threshold_N": data["threshold_N"],
    "metric": "For each trial: 100/N_valid * sum_t [sum_i C_flat(t,i)*(1-C_rough(t,i))/sum_i C_flat(t,i)], over t with sum_i C_flat(t,i)>0.",
    "weights": "All valid 10 ms snapshots have equal weight, irrespective of expected foot count; group mean/SD of per-trial portions, not a pooled contact-count ratio.",
    "statistics": "Same original retained spawn lists and displacement values; no new omissions. Sample SD (ddof=1) across trials in each group. One flat reference per wave, no estimated SD.",
    "curve_order": "Straight lines join group means in source condition order; no terrain roughness on either axis, no curve fit.",
    "comparison_scope": "A matched commanded CPG phase and foot-ID contact pattern difference, not calibrated IID erasure probability. Intersections compare equal observed loss portions, not necessarily equal terrain conditions.",
    "checks": {"contact_sources_hash_match": True, "snapshot_times_and_foot_ids_match": True,
               "missing_is_subset_of_flat_expected": True, "zero_denominator_skipped": True,
               "fractions_between_zero_and_one": True, "original_pooled_fraction_reproduced": True,
               "original_displacement_mean_sd_unchanged": True,
               "retained_trial_count": 267, "raw_complete_trial_count": 297,
               "raw_failed_trial_count_not_scored": data["raw_numerical_failure_count"]},
    "valid_snapshot_counts_by_wave": {str(w): next(c["valid_snapshot_count"] for c in raw if c["wave"] == w) for w in WAVES},
    "flat_reference_sampling_by_wave": {
        str(w): {"expected_foot_counts_by_snapshot": references[w][1].sum(axis=1).tolist(),
                 "skipped_zero_expected_times_s": references[w][0][references[w][1].sum(axis=1) == 0].tolist()}
        for w in WAVES},
    "visual_review": "pending", "raw_complete_trials": raw, "groups": groups,
    "outputs": {name: {"path": str(OUT / (STEM + suffix)), "sha256": sha(OUT / (STEM + suffix))}
                for name, suffix in [("png", ".png"), ("svg", ".svg"), ("executed_source", "_source.py")]},
}
(OUT / (STEM + ".json")).write_text(json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
print(json.dumps({"png": str(OUT / (STEM + ".png")),
                  "valid_snapshots": receipt["valid_snapshot_counts_by_wave"],
                  "groups": [{"wave": g["wave"], "height": g["height_scale_source_condition"],
                              "loss_percent": g["mean_instantaneous_loss_percent"]["mean"],
                              "old_pooled_percent": g["old_pooled_missing_percent_for_comparison"]}
                             for g in groups if g["height_scale_source_condition"] == 1.5]}, ensure_ascii=False))
