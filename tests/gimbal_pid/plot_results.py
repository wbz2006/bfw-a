"""Plot the actual final acceptance traces from build/gimbal_pid."""

import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from run_sine import ROOT


directory = ROOT / "build/gimbal_pid"
figure, panels = plt.subplots(3, 2, figsize=(13, 9), sharex=True, constrained_layout=True)
for column, axis in enumerate(("yaw", "pitch")):
    name = f"{axis}_final_acceptance"
    data = np.genfromtxt(directory / f"{name}.csv", delimiter=",", names=True)
    result = json.loads((directory / f"{name}.json").read_text())
    metrics = result["metrics"][axis]
    time = data["time_s"]
    panels[0, column].plot(time, data[f"{axis}_ref"], "--", label="Target", linewidth=1.3)
    panels[0, column].plot(time, data[f"{axis}_measure"], label="Actual", linewidth=1)
    panels[0, column].set_title(
        f"{axis.title()}: +/-{result['amplitude_deg']:g} deg, {result['frequency_hz']:g} Hz\n"
        f"RMS {metrics['rms_error_deg']:.3f} deg; peak {metrics['peak_error_deg']:.3f} deg"
    )
    panels[0, column].set_ylabel("Angle (deg)")
    panels[0, column].legend(loc="upper right")
    panels[1, column].plot(time, data[f"{axis}_error"], linewidth=0.8)
    panels[1, column].axhline(1, color="tab:red", linestyle="--", linewidth=0.8)
    panels[1, column].axhline(-1, color="tab:red", linestyle="--", linewidth=0.8)
    panels[1, column].set_ylabel("Error (deg)")
    panels[2, column].plot(time, data[f"{axis}_output"] / 20000 * 100, linewidth=0.8)
    panels[2, column].set_ylabel("Output (% limit)")
    panels[2, column].set_xlabel("Time (s)")
    for panel in panels[:, column]:
        panel.axvspan(0, result["warmup_s"], color="grey", alpha=0.12)
        panel.axvspan(result["analysis_end_s"], result["duration_s"], color="grey", alpha=0.12)
        panel.grid(alpha=0.2)
figure.suptitle("Gimbal sine acceptance: measured board traces; shaded intervals excluded")
figure.savefig(directory / "acceptance.png", dpi=170)
figure.savefig(directory / "acceptance.svg")
print(directory / "acceptance.png")
