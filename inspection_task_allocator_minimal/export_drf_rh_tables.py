import csv
from pathlib import Path


def read_rows(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def fmt(value):
    if value in {"", None}:
        return ""
    try:
        return f"{float(value):.2f}"
    except Exception:
        return str(value)


def markdown_table(headers, rows):
    lines = ["| " + " | ".join(headers) + " |"]
    lines.append("| " + " | ".join("---" for _ in headers) + " |")
    for row in rows:
        lines.append("| " + " | ".join(str(item) for item in row) + " |")
    return "\n".join(lines)


def row_by_method(rows):
    return {row["method"]: row for row in rows}


def main_table(summary):
    order = [
        "NNF",
        "AStarOnly",
        "Proposed-Balanced",
        "RH-Proposed-v2",
        "Priority-Greedy",
        "Deadline-Greedy",
        "TSP-2opt",
        "DRF-RH-Full",
        "DRF-RH-Light",
    ]
    rows = row_by_method(summary)
    out = []
    for method in order:
        row = rows.get(method)
        if not row:
            continue
        out.append(
            [
                method,
                fmt(row["total_path_length_mean"]),
                fmt(row["total_inspection_time_mean"]),
                fmt(row["high_priority_avg_response_time_mean"]),
                fmt(row["priority_weighted_completion_time_mean"]),
                fmt(row["high_priority_top5_rate_mean"]),
                fmt(row.get("priority_response_efficiency_mean", "")),
                fmt(row["algorithm_runtime_ms_mean"]),
            ]
        )
    return markdown_table(
        ["方法", "总路径长度", "总巡检时间/s", "高优先级响应/s", "优先级加权完成/s", "Top5比例/%", "单位路径响应效率", "运行时间/ms"],
        out,
    )


def abnormal_table(summary):
    order = [
        "AStarOnly",
        "Proposed-Balanced",
        "RH-v2-Full",
        "RH-v2-Light",
        "DRF-RH-Full",
        "DRF-RH-Light",
        "Priority-Greedy",
        "Deadline-Greedy",
    ]
    rows = row_by_method(summary)
    out = []
    for method in order:
        row = rows.get(method)
        if not row:
            continue
        out.append(
            [
                method,
                fmt(row["abnormal_priority_rate_mean"]),
                fmt(row["abnormal_avg_response_time_mean"]),
                fmt(row["high_priority_avg_response_time_mean"]),
                fmt(row["total_inspection_time_mean"]),
                fmt(row["algorithm_runtime_ms_mean"]),
            ]
        )
    return markdown_table(
        ["方法", "异常任务优先处理率/%", "异常平均响应/s", "高优先级响应/s", "总巡检时间/s", "运行时间/ms"],
        out,
    )


def ablation_table(summary):
    rows = []
    for row in summary:
        rows.append(
            [
                row["method"],
                fmt(row["high_priority_avg_response_time_mean"]),
                fmt(row["priority_weighted_completion_time_mean"]),
                fmt(row["high_priority_top5_rate_mean"]),
                fmt(row["total_path_length_mean"]),
                fmt(row["total_inspection_time_mean"]),
                fmt(row["algorithm_runtime_ms_mean"]),
            ]
        )
    return markdown_table(
        ["方法", "高优先级响应/s", "优先级加权完成/s", "Top5比例/%", "总路径长度", "总巡检时间/s", "运行时间/ms"],
        rows,
    )


def structured_table(summary):
    rows = []
    for row in summary:
        rows.append(
            [
                row["map_type"],
                row["method"],
                fmt(row["total_path_length_mean"]),
                fmt(row["total_inspection_time_mean"]),
                fmt(row["high_priority_avg_response_time_mean"]),
                fmt(row["high_priority_top5_rate_mean"]),
            ]
        )
    return markdown_table(
        ["地图类型", "方法", "总路径长度", "总巡检时间/s", "高优先级响应/s", "Top5比例/%"],
        rows,
    )


def significance_table(rows):
    selected = [row for row in rows if row["conclusion"] != "skipped"][:40]
    out = []
    for row in selected:
        out.append(
            [
                row["experiment"],
                row["comparison"],
                row["metric"],
                fmt(row["mean_diff"]),
                fmt(row["p_value"]) if row["p_value"] != "" else "",
                row["conclusion"],
            ]
        )
    return markdown_table(
        ["实验", "比较", "指标", "mean_diff", "p_value", "结论"],
        out,
    )


def main():
    base_dir = Path(__file__).resolve().parent
    results_dir = base_dir / "results"
    main_summary = read_rows(results_dir / "drf_rh_main_summary.csv")
    abnormal_summary = read_rows(results_dir / "drf_rh_abnormal_summary.csv")
    ablation_summary = read_rows(results_dir / "drf_rh_ablation_summary.csv")
    structured_summary = read_rows(results_dir / "drf_rh_structured_summary.csv")
    significance = read_rows(results_dir / "drf_rh_significance.csv")

    content = [
        "# DRF-RH 增强实验论文表格",
        "",
        "## 表 A DRF-RH 主实验横向对比结果",
        "",
        main_table(main_summary),
        "",
        "## 表 B DRF-RH 异常反馈实验结果",
        "",
        abnormal_table(abnormal_summary),
        "",
        "## 表 C DRF-RH 内部结构消融结果",
        "",
        ablation_table(ablation_summary),
        "",
        "## 表 D 结构化地图实验结果",
        "",
        structured_table(structured_summary),
        "",
        "## 表 E 显著性检验摘要",
        "",
        significance_table(significance),
        "",
        "## 说明",
        "",
        "1. DRF-RH 是在 RH-Proposed-v2 基础上引入动态异常风险场和响应期限约束的新方法。",
        "2. DRF-RH 的目标不是全局最短路径，而是提高关键任务和异常任务响应。",
        "3. TSP-2opt 用于代表路径效率优先的序列优化基线。",
        "4. Priority-Greedy 和 Deadline-Greedy 用于证明 DRF-RH 不只是简单 priority/deadline 贪心。",
        "5. 内部消融用于证明 DRF-RH 的结构性贡献。",
    ]
    output_path = results_dir / "drf_rh_paper_tables.md"
    output_path.write_text("\n".join(content) + "\n", encoding="utf-8")
    print(f"Saved: {output_path}")


if __name__ == "__main__":
    main()
