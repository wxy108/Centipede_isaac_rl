import pandas as pd
import matplotlib.pyplot as plt

filepath = r"C:\Users\wxy22\Documents\Codex\2026-09-15\ni\outputs\amplitude_booklet_2026-09-28\exact_amplitude_table.csv"

df = pd.read_csv(filepath)

plt.figure(figsize=(7, 5))

plt.plot(df["wave"], df["A_deg"], marker="o", linewidth=2, label="Body amplitude A")
plt.plot(df["wave"], df["B_deg"], marker="s", linewidth=2, label="Leg amplitude B")

plt.xlabel("Wave Number")
plt.ylabel("Amplitude (deg)")
plt.title("Amplitude vs. Wave Number")

plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()

plt.savefig(
    r"C:\Users\wxy22\Documents\Centipede_IsaacLab_RL\outputs\figures\amplitude_vs_wave.png",
    dpi=300,
    bbox_inches="tight"
)
plt.show()