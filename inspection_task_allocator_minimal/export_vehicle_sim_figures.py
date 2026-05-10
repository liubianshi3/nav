import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
FIG_DIR = RESULTS / "figures" / "vehicle_sim"
FIG_DIR.mkdir(parents=True, exist_ok=True)


def configure_chinese_font():
    candidates = [
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Noto Sans CJK TC",
        "Microsoft YaHei",
        "SimHei",
        "WenQuanYi Zen Hei",
        "PingFang SC",
        "Source Han Sans SC",
        "Arial Unicode MS",
    ]
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name]
            break
    else:
        # Keep matplotlib default if no CJK font is installed.
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def read_json(path):
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def main():
    configure_chinese_font()

    summary = read_csv(RESULTS / "vehicle_sim_summary.csv")
    records = read_json(RESULTS / "vehicle_sim_records.json")

    methods = ["AStarOnly", "Proposed-Balanced", "RH-v2-Light", "TSP-2opt", "Priority-Greedy"]
    metrics = ["vehicle_trajectory_length_mean", "vehicle_execution_time_mean", "high_priority_avg_response_time_mean", "heading_change_sum_mean"]
    labels = ["轨迹长度", "执行时间", "高优先级响应时间", "航向变化总和"]

    values = {m: {k: 0.0 for k in metrics} for m in methods}
    for row in summary:
        if row["method"] in values:
            for k in metrics:
                values[row["method"]][k] = float(row[k])

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes = axes.flatten()
    for ax, metric, label in zip(axes, metrics, labels):
        ys = [values[m][metric] for m in methods]
        ax.bar(methods, ys, color=["#4C72B0", "#55A868", "#C44E52", "#8172B3", "#CCB974"])
        ax.set_title(label, fontsize=12)
        ax.tick_params(axis="x", rotation=20)
        for x, y in zip(methods, ys):
            ax.text(x, y, f"{y:.2f}", ha="center", va="bottom", fontsize=8)
    fig.suptitle("小车运动学仿真主结果对比")
    fig.tight_layout()
    fig.savefig(FIG_DIR / "fig_vehicle_sim_main_compare.png", dpi=300)
    plt.close(fig)

    sample = next((r for r in records if r.get("seed") == 0 and r.get("method") == "RH-v2-Light"), None)
    if sample:
        fig, ax = plt.subplots(figsize=(8, 8))
        traj = sample.get("trajectory", [])
        if traj:
            xs = [p[0] for p in traj]
            ys = [p[1] for p in traj]
            ax.plot(xs, ys, linewidth=2, label="轨迹")
        ax.scatter([2], [2], c="red", s=80, marker="s", label="起点")
        ax.set_title("seed=0 下 RH-v2-Light 小车轨迹", fontsize=12)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(FIG_DIR / "fig_vehicle_sim_sample_trajectory.png", dpi=300)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 6))
    methods_t = ["AStarOnly", "Proposed-Balanced", "RH-v2-Light", "TSP-2opt", "Priority-Greedy"]
    for idx, method in enumerate(methods_t):
        rec = next((r for r in records if r.get("seed") == 0 and r.get("method") == method), None)
        if not rec:
            continue
        times = [e["finish_time"] for e in rec.get("execution_records", [])]
        ax.plot(times, [idx] * len(times), marker="o", linestyle="-", label=method)
    ax.set_yticks(range(len(methods_t)))
    ax.set_yticklabels(methods_t)
    ax.set_title("seed=0 下不同方法任务完成时间轴")
    ax.set_xlabel("完成时间/s")
    ax.grid(True, axis="x", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "fig_vehicle_sim_sequence_timeline.png", dpi=300)
    plt.close(fig)

    print(str(FIG_DIR))


if __name__ == "__main__":
    main()
