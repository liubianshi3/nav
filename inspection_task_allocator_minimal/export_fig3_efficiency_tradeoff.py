import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager


REQUIRED_FIELDS = [
    "method",
    "algorithm_runtime_ms_mean",
    "high_priority_avg_response_time_mean",
]

METHOD_ORDER = [
    "RH-v2-Full",
    "RH-v2-Medium",
    "RH-v2-Light",
    "RH-v2-Fast",
]

COLORS = {
    "RH-v2-Full": "#4C78A8",
    "RH-v2-Medium": "#F58518",
    "RH-v2-Light": "#54A24B",
    "RH-v2-Fast": "#E45756",
}


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


def plot(csv_path, output_path):
    rows = read_rows(csv_path)
    row_map = {row["method"].strip(): row for row in rows}
    missing_methods = [method for method in METHOD_ORDER if method not in row_map]
    if missing_methods:
        raise ValueError(f"CSV 缺少必要方法行: {', '.join(missing_methods)}")

    configure_chinese_font()
    fig, ax = plt.subplots(figsize=(8.8, 6.6), dpi=300)
    fig.patch.set_facecolor("white")

    for method in METHOD_ORDER:
        runtime = float(row_map[method]["algorithm_runtime_ms_mean"])
        response = float(row_map[method]["high_priority_avg_response_time_mean"])
        ax.scatter(runtime, response, s=110, color=COLORS[method], edgecolors="black", linewidths=0.7)
        ax.text(runtime + 3, response + 1.5, method, fontsize=10)

    ax.set_title("RH-v2 参数效率权衡关系", fontsize=15)
    ax.set_xlabel("算法运行时间 / ms", fontsize=12)
    ax.set_ylabel("高优先级任务平均响应时间 / s", fontsize=12)
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.set_axisbelow(True)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300, facecolor="white", bbox_inches="tight")
    plt.close(fig)

    print(f"读取文件路径: {csv_path}")
    print(f"输出图片路径: {output_path}")
    print(f"使用列: {', '.join(REQUIRED_FIELDS)}")
    print("是否成功保存: 是")


def main():
    base_dir = Path(__file__).resolve().parent
    csv_path = base_dir / "results" / "rh_v2_efficiency_summary.csv"
    output_path = base_dir / "results" / "figures" / "fig3_efficiency_tradeoff.png"
    plot(csv_path, output_path)


if __name__ == "__main__":
    main()
