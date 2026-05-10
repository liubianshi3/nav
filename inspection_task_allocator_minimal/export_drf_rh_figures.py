import csv
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager


def configure_chinese_font():
    candidates = [
        "SimHei",
        "Microsoft YaHei",
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "WenQuanYi Zen Hei",
        "PingFang SC",
        "Source Han Sans SC",
        "Arial Unicode MS",
    ]
    available = {font.name for font in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name]
            break
    else:
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


def read_rows(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def annotate(ax, bars):
    top = ax.get_ylim()[1]
    for bar in bars:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2, h + top * 0.012, f"{h:.2f}", ha="center", va="bottom", fontsize=8)


def bar_grid(rows, methods, metrics, titles, ylabels, title, output_path):
    row_map = {row["method"]: row for row in rows}
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), dpi=300)
    fig.patch.set_facecolor("white")
    colors = ["#4C78A8", "#F58518", "#54A24B", "#E45756"]
    for ax, metric, sub_title, ylabel, color in zip(axes.flat, metrics, titles, ylabels, colors):
        values = [float(row_map[m][f"{metric}_mean"]) for m in methods if m in row_map]
        labels = [m for m in methods if m in row_map]
        bars = ax.bar(labels, values, color=color, edgecolor="black", linewidth=0.6)
        ax.set_title(sub_title, fontsize=12)
        ax.set_ylabel(ylabel)
        ax.tick_params(axis="x", rotation=20, labelsize=8)
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        ax.set_axisbelow(True)
        ax.set_ylim(0, max(values) * 1.20 if values and max(values) > 0 else 1)
        annotate(ax, bars)
    fig.suptitle(title, fontsize=16)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def fig_main(results_dir, out_dir):
    rows = read_rows(results_dir / "drf_rh_main_summary.csv")
    methods = [
        "AStarOnly",
        "RH-Proposed-v2",
        "Priority-Greedy",
        "Deadline-Greedy",
        "TSP-2opt",
        "DRF-RH-Full",
        "DRF-RH-Light",
    ]
    bar_grid(
        rows,
        methods,
        [
            "high_priority_avg_response_time",
            "high_priority_top5_rate",
            "total_path_length",
            "priority_response_efficiency",
        ],
        ["高优先级响应时间", "Top5 高优先级比例", "总路径长度", "单位路径响应效率"],
        ["时间 / s", "比例 / %", "路径长度", "效率"],
        "DRF-RH 主实验横向对比",
        out_dir / "fig_drf_main_compare.png",
    )


def fig_abnormal(results_dir, out_dir):
    rows = read_rows(results_dir / "drf_rh_abnormal_summary.csv")
    methods = [
        "AStarOnly",
        "Proposed-Balanced",
        "RH-v2-Full",
        "DRF-RH-Full",
        "DRF-RH-Light",
        "Priority-Greedy",
        "Deadline-Greedy",
    ]
    bar_grid(
        rows,
        methods,
        [
            "abnormal_avg_response_time",
            "abnormal_priority_rate",
            "high_priority_avg_response_time",
            "total_inspection_time",
        ],
        ["异常平均响应时间", "异常任务优先处理率", "高优先级响应时间", "总巡检时间"],
        ["时间 / s", "比例 / %", "时间 / s", "时间 / s"],
        "DRF-RH 异常反馈实验对比",
        out_dir / "fig_drf_abnormal_compare.png",
    )


def fig_ablation(results_dir, out_dir):
    rows = read_rows(results_dir / "drf_rh_ablation_summary.csv")
    methods = [row["method"] for row in rows]
    bar_grid(
        rows,
        methods,
        [
            "high_priority_avg_response_time",
            "high_priority_top5_rate",
            "priority_weighted_completion_time",
            "total_path_length",
        ],
        ["高优先级响应时间", "Top5 高优先级比例", "优先级加权完成时间", "总路径长度"],
        ["时间 / s", "比例 / %", "时间 / s", "路径长度"],
        "DRF-RH 内部结构消融实验",
        out_dir / "fig_drf_ablation.png",
    )


def fig_structured(results_dir, out_dir):
    rows = read_rows(results_dir / "drf_rh_structured_summary.csv")
    grouped = defaultdict(lambda: defaultdict(list))
    for row in rows:
        grouped[row["method"]]["total_path_length"].append(float(row["total_path_length_mean"]))
        grouped[row["method"]]["high_priority_avg_response_time"].append(float(row["high_priority_avg_response_time_mean"]))
    methods = ["AStarOnly", "RH-Proposed-v2", "Priority-Greedy", "TSP-2opt", "DRF-RH-Full", "DRF-RH-Light"]
    overall = []
    for method in methods:
        if method not in grouped:
            continue
        overall.append(
            {
                "method": method,
                "total_path_length_mean": sum(grouped[method]["total_path_length"]) / len(grouped[method]["total_path_length"]),
                "high_priority_avg_response_time_mean": sum(grouped[method]["high_priority_avg_response_time"]) / len(grouped[method]["high_priority_avg_response_time"]),
                "high_priority_top5_rate_mean": 0.0,
                "priority_response_efficiency_mean": 0.0,
            }
        )
    bar_grid(
        overall,
        [row["method"] for row in overall],
        [
            "total_path_length",
            "high_priority_avg_response_time",
            "total_path_length",
            "high_priority_avg_response_time",
        ],
        ["结构化地图总路径长度", "结构化地图高优先级响应", "总路径长度", "高优先级响应"],
        ["路径长度", "时间 / s", "路径长度", "时间 / s"],
        "结构化地图总体结果",
        out_dir / "fig_drf_structured_maps.png",
    )


def fig_significance(results_dir, out_dir):
    rows = [
        row
        for row in read_rows(results_dir / "drf_rh_significance.csv")
        if row.get("p_value") not in {"", None} and row.get("conclusion") != "skipped"
    ][:50]
    if not rows:
        print("No significance rows with p-value; skip heatmap.")
        return
    comparisons = sorted({row["comparison"] for row in rows})
    metrics = sorted({row["metric"] for row in rows})
    matrix = [[1.0 for _ in metrics] for _ in comparisons]
    for row in rows:
        i = comparisons.index(row["comparison"])
        j = metrics.index(row["metric"])
        matrix[i][j] = min(1.0, float(row["p_value"]))
    fig, ax = plt.subplots(figsize=(max(8, len(metrics) * 1.2), max(5, len(comparisons) * 0.35)), dpi=300)
    im = ax.imshow(matrix, cmap="viridis_r", vmin=0, vmax=0.1)
    ax.set_xticks(range(len(metrics)))
    ax.set_xticklabels(metrics, rotation=35, ha="right", fontsize=8)
    ax.set_yticks(range(len(comparisons)))
    ax.set_yticklabels(comparisons, fontsize=7)
    ax.set_title("DRF-RH 显著性检验 p-value 热力图", fontsize=14)
    fig.colorbar(im, ax=ax, fraction=0.03, pad=0.02)
    fig.tight_layout()
    fig.savefig(out_dir / "fig_drf_significance_heatmap.png", dpi=300, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def main():
    configure_chinese_font()
    base_dir = Path(__file__).resolve().parent
    results_dir = base_dir / "results"
    out_dir = results_dir / "figures" / "drf_rh"
    out_dir.mkdir(parents=True, exist_ok=True)
    fig_main(results_dir, out_dir)
    fig_abnormal(results_dir, out_dir)
    fig_ablation(results_dir, out_dir)
    fig_structured(results_dir, out_dir)
    fig_significance(results_dir, out_dir)
    print(f"Saved figures to: {out_dir}")


if __name__ == "__main__":
    main()
