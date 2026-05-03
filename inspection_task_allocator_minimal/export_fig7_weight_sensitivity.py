import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager


REQUIRED_FIELDS = [
    "weight_group",
    "total_path_length_mean",
    "total_inspection_time_mean",
    "high_priority_avg_response_time_mean",
]

GROUP_ORDER = [
    "Default",
    "PathPriority",
    "TaskPriority",
    "Balanced",
    "WeakPathPenalty",
]


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


def read_rows(csv_path):
    with csv_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        missing = [field for field in REQUIRED_FIELDS if field not in fieldnames]
        if missing:
            raise ValueError(f"CSV 缺少必要字段: {', '.join(missing)}")
        return list(reader)


def extract_metrics(rows):
    row_map = {row["weight_group"].strip(): row for row in rows}
    missing_groups = [group for group in GROUP_ORDER if group not in row_map]
    if missing_groups:
        raise ValueError(f"CSV 缺少必要权重组行: {', '.join(missing_groups)}")
    return {
        "total_path_length_mean": [float(row_map[g]["total_path_length_mean"]) for g in GROUP_ORDER],
        "total_inspection_time_mean": [float(row_map[g]["total_inspection_time_mean"]) for g in GROUP_ORDER],
        "high_priority_avg_response_time_mean": [
            float(row_map[g]["high_priority_avg_response_time_mean"]) for g in GROUP_ORDER
        ],
    }


def annotate(ax, bars):
    ymax = ax.get_ylim()[1]
    offset = ymax * 0.015 if ymax > 0 else 0.05
    for bar in bars:
        height = bar.get_height()
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            height + offset,
            f"{height:.2f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )


def plot(csv_path, output_path):
    rows = read_rows(csv_path)
    metrics = extract_metrics(rows)
    configure_chinese_font()

    fig, axes = plt.subplots(1, 3, figsize=(15, 5.2), dpi=300)
    fig.patch.set_facecolor("white")
    specs = [
        ("（a）总路径长度", "路径长度", metrics["total_path_length_mean"], "#4C78A8"),
        ("（b）总巡检时间", "时间 / s", metrics["total_inspection_time_mean"], "#F58518"),
        ("（c）高优先级任务平均响应时间", "时间 / s", metrics["high_priority_avg_response_time_mean"], "#54A24B"),
    ]

    for ax, (title, ylabel, values, color) in zip(axes, specs):
        bars = ax.bar(GROUP_ORDER, values, width=0.65, color=color, edgecolor="black", linewidth=0.6)
        ax.set_title(title, fontsize=13)
        ax.set_ylabel(ylabel, fontsize=11)
        ax.tick_params(axis="x", labelrotation=20, labelsize=9)
        ax.tick_params(axis="y", labelsize=10)
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        ax.set_axisbelow(True)
        ax.set_ylim(0, max(values) * 1.18 if max(values) > 0 else 1.0)
        annotate(ax, bars)

    fig.suptitle("权重敏感性实验结果对比", fontsize=16)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300, facecolor="white", bbox_inches="tight")
    plt.close(fig)

    print(f"读取文件路径: {csv_path}")
    print(f"输出图片路径: {output_path}")
    print(f"使用列: {', '.join(REQUIRED_FIELDS)}")
    print("是否成功保存: 是")


def main():
    base_dir = Path(__file__).resolve().parent
    csv_path = base_dir / "results" / "weight_sensitivity_summary.csv"
    output_path = base_dir / "results" / "figures" / "fig7_weight_sensitivity.png"
    plot(csv_path, output_path)


if __name__ == "__main__":
    main()
