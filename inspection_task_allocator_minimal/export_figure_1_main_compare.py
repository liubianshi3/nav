import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager


REQUIRED_FIELDS = [
    "method",
    "total_path_length_mean",
    "total_inspection_time_mean",
    "high_priority_avg_response_time_mean",
    "high_priority_top5_rate_mean",
]

METHOD_LABELS = {
    "NNF": "NNF",
    "AStarOnly": "AStarOnly",
    "Proposed-Balanced": "Proposed-Balanced",
    "RH-Proposed-v2": "RH-Proposed-v2",
}

PREFERRED_METHOD_ORDER = [
    "NNF",
    "AStarOnly",
    "Proposed-Balanced",
    "RH-Proposed-v2",
]


def configure_chinese_font():
    candidate_fonts = [
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
    for font_name in candidate_fonts:
        if font_name in available:
            plt.rcParams["font.sans-serif"] = [font_name]
            break
    else:
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


def read_summary(csv_path):
    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        missing = [field for field in REQUIRED_FIELDS if field not in fieldnames]
        if missing:
            raise ValueError(f"CSV 缺少必要字段: {', '.join(missing)}")
        rows = list(reader)
    return rows


def normalize_method_name(method_name):
    text = method_name.strip()
    if text == "RH-Proposed-v2":
        return "RH-Proposed-v2"
    if text == "NNF":
        return "NNF"
    if text == "AStarOnly":
        return "AStarOnly"
    if text == "Proposed-Balanced":
        return "Proposed-Balanced"
    return None


def extract_plot_data(rows):
    row_map = {}
    for row in rows:
        method = normalize_method_name(row["method"])
        if method is None:
            continue
        row_map[method] = row

    missing_methods = [method for method in PREFERRED_METHOD_ORDER if method not in row_map]
    if missing_methods:
        raise ValueError(f"CSV 缺少必要方法行: {', '.join(missing_methods)}")

    methods = PREFERRED_METHOD_ORDER
    labels = [METHOD_LABELS[method] for method in methods]
    metrics = {
        "total_path_length_mean": [float(row_map[method]["total_path_length_mean"]) for method in methods],
        "total_inspection_time_mean": [float(row_map[method]["total_inspection_time_mean"]) for method in methods],
        "high_priority_avg_response_time_mean": [
            float(row_map[method]["high_priority_avg_response_time_mean"]) for method in methods
        ],
        "high_priority_top5_rate_mean": [float(row_map[method]["high_priority_top5_rate_mean"]) for method in methods],
    }
    return labels, metrics


def annotate_bars(ax, bars):
    ymax = ax.get_ylim()[1]
    offset = ymax * 0.015
    for bar in bars:
        height = bar.get_height()
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            height + offset,
            f"{height:.2f}",
            ha="center",
            va="bottom",
            fontsize=10,
        )


def plot_figure(labels, metrics, output_path):
    configure_chinese_font()

    fig, axes = plt.subplots(2, 2, figsize=(12, 9), dpi=300)
    fig.patch.set_facecolor("white")

    plot_specs = [
        ("（a）总路径长度", "路径长度", metrics["total_path_length_mean"], "#4C78A8"),
        ("（b）总巡检时间", "时间 / s", metrics["total_inspection_time_mean"], "#F58518"),
        ("（c）高优先级任务平均响应时间", "时间 / s", metrics["high_priority_avg_response_time_mean"], "#54A24B"),
        ("（d）前5任务高优先级比例", "比例 / %", metrics["high_priority_top5_rate_mean"], "#E45756"),
    ]

    for ax, (title, ylabel, values, color) in zip(axes.flat, plot_specs):
        bars = ax.bar(labels, values, color=color, width=0.65, edgecolor="black", linewidth=0.6)
        ax.set_title(title, fontsize=13)
        ax.set_ylabel(ylabel, fontsize=11)
        ax.tick_params(axis="x", labelrotation=0, labelsize=10)
        ax.tick_params(axis="y", labelsize=10)
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        ax.set_axisbelow(True)
        ymax = max(values) * 1.18 if max(values) > 0 else 1.0
        ax.set_ylim(0, ymax)
        annotate_bars(ax, bars)

    fig.suptitle("主实验中不同方法的性能对比", fontsize=16)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def main():
    base_dir = Path(__file__).resolve().parent
    csv_path = base_dir / "results" / "rh_v2_compare_summary.csv"
    output_path = base_dir / "results" / "figures" / "fig_1_main_compare.png"

    rows = read_summary(csv_path)
    labels, metrics = extract_plot_data(rows)
    plot_figure(labels, metrics, output_path)

    print(f"读取的 CSV 路径: {csv_path}")
    print(f"成功保存的图片路径: {output_path}")
    print(f"参与绘图的方法名称: {', '.join(labels)}")
    print("4 个指标的实际数值摘要:")
    print(f"- 总路径长度: {[f'{value:.2f}' for value in metrics['total_path_length_mean']]}")
    print(f"- 总巡检时间: {[f'{value:.2f}' for value in metrics['total_inspection_time_mean']]}")
    print(
        f"- 高优先级任务平均响应时间: "
        f"{[f'{value:.2f}' for value in metrics['high_priority_avg_response_time_mean']]}"
    )
    print(
        f"- 前5任务高优先级比例: "
        f"{[f'{value:.2f}' for value in metrics['high_priority_top5_rate_mean']]}"
    )


if __name__ == "__main__":
    main()
