"""
Run baseline vs TTC-augmented controllers in a mixed platoon and plot results.
"""

import matplotlib
matplotlib.use("Agg")  # headless backend for saving files
import numpy as np
import matplotlib.pyplot as plt

from simulation import build_demo_platoon, summarize_metrics


def run_case(label: str, k_ttc: float):
    sim = build_demo_platoon(k_ttc=k_ttc)
    history = sim.run(T=30.0)
    metrics = summarize_metrics(history)
    print(f"{label} (k_ttc={k_ttc}) metrics: {metrics}")
    return history, metrics


def plot_signals(history, title: str):
    t = np.array(history["t"])
    v = np.array(history["v"])  # shape (time, vehicles)
    gaps = np.array(history["gap"], dtype=float)
    ttcs = np.array(
        [[np.nan if ttc is None else ttc for ttc in row] for row in history["ttc"]],
        dtype=float,
    )

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    for i in range(min(5, v.shape[1])):  # plot first few vehicles
        axes[0].plot(t, v[:, i], label=f"veh {i}")
    axes[0].set_ylabel("Speed (m/s)")
    axes[0].legend()
    axes[0].set_title(title)

    for i in range(1, min(5, gaps.shape[1])):  # gaps skip leader
        axes[1].plot(t, gaps[:, i], label=f"gap to {i-1}->{i}")
    axes[1].set_ylabel("Gap (m)")
    axes[1].legend()

    for i in range(1, min(5, ttcs.shape[1])):
        axes[2].plot(t, ttcs[:, i], label=f"TTC {i-1}->{i}")
    axes[2].set_ylabel("TTC (s)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend()
    axes[2].axhline(1.0, color="r", linestyle="--", linewidth=1, label="1s threshold")

    plt.tight_layout()
    return fig


if __name__ == "__main__":
    np.random.seed(0)
    # Sweep a few TTC gains for quick comparison
    k_ttc_values = [0.0, 1.0, 2.0, 3.0]
    histories = []
    for k in k_ttc_values:
        hist, _ = run_case(f"Case", k_ttc=k)
        histories.append((k, hist))

    # Plot first and last to see difference, save to plots/
    fig0 = plot_signals(histories[0][1], "Baseline (k_ttc=0.0)")
    figN = plot_signals(histories[-1][1], f"Improved (k_ttc={k_ttc_values[-1]})")
    fig0.savefig("plots/baseline_kttc0.png", dpi=150)
    figN.savefig(f"plots/improved_kttc{k_ttc_values[-1]}.png", dpi=150)
