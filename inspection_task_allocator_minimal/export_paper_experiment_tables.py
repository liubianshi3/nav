import csv
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
OUTPUT = RESULTS / "paper_experiment_tables.md"

BALANCED_SUMMARY = RESULTS / "batch_compare_balanced_summary.csv"
WEIGHT_SUMMARY = RESULTS / "weight_sensitivity_summary.csv"
ABNORMAL_SUMMARY = RESULTS / "abnormal_feedback_summary.csv"


def read_csv_by_method(path, key_name):
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return {row[key_name]: row for row in rows}


def fmt(x):
    return f"{float(x):.2f}"


def change(a, b):
    a = float(a)
    b = float(b)
    if b == 0:
        return 0.0
    return (a - b) / b * 100.0


def main():
    balanced = read_csv_by_method(BALANCED_SUMMARY, "method")
    weights = read_csv_by_method(WEIGHT_SUMMARY, "weight_group")
    abnormal = read_csv_by_method(ABNORMAL_SUMMARY, "method")

    lines = []
    lines.append("## 表 1 Balanced 权重下不同方法的主实验性能对比")
    lines.append("")
    lines.append("| 方法 | 完成任务数均值 | 总路径长度均值 | 总巡检时间均值/s | 高优先级任务平均响应时间/s |")
    lines.append("| --- | --- | --- | --- | --- |")
    for method in ["FS", "NNF", "AStarOnly", "Proposed-Balanced"]:
        r = balanced[method]
        lines.append(
            f"| {method} | {fmt(r['completed_task_num_mean'])} | {fmt(r['total_path_length_mean'])} | {fmt(r['total_inspection_time_mean'])} | {fmt(r['high_priority_avg_response_time_mean'])} |"
        )

    lines.append("")
    lines.append("## 表 2 Proposed-Balanced 相比 AStarOnly 的主实验指标变化率")
    lines.append("")
    lines.append("| 指标 | AStarOnly | Proposed-Balanced | 变化率/% |")
    lines.append("| --- | --- | --- | --- |")
    for label, key in [("总路径长度", "total_path_length_mean"), ("总巡检时间", "total_inspection_time_mean"), ("高优先级任务平均响应时间", "high_priority_avg_response_time_mean")]:
        a = float(balanced["AStarOnly"][key])
        p = float(balanced["Proposed-Balanced"][key])
        lines.append(f"| {label} | {fmt(a)} | {fmt(p)} | {change(p, a):.2f} |")

    lines.append("")
    lines.append("## 表 3 不同权重组下 Proposed 方法性能对比")
    lines.append("")
    lines.append("| 权重组 | 完成任务数均值 | 总路径长度均值 | 总巡检时间均值/s | 高优先级任务平均响应时间/s |")
    lines.append("| --- | --- | --- | --- | --- |")
    for wg in ["Default", "PathPriority", "TaskPriority", "Balanced", "WeakPathPenalty"]:
        r = weights[wg]
        lines.append(
            f"| {wg} | {fmt(r['completed_task_num_mean'])} | {fmt(r['total_path_length_mean'])} | {fmt(r['total_inspection_time_mean'])} | {fmt(r['high_priority_avg_response_time_mean'])} |"
        )

    lines.append("")
    lines.append("## 表 4 各权重组相对 Default 的指标变化率")
    lines.append("")
    lines.append("| 权重组 | 总路径长度变化率/% | 总巡检时间变化率/% | 高优先级响应时间变化率/% |")
    lines.append("| --- | --- | --- | --- |")
    default = weights["Default"]
    for wg in ["PathPriority", "TaskPriority", "Balanced", "WeakPathPenalty"]:
        r = weights[wg]
        lines.append(
            f"| {wg} | {change(r['total_path_length_mean'], default['total_path_length_mean']):.2f} | {change(r['total_inspection_time_mean'], default['total_inspection_time_mean']):.2f} | {change(r['high_priority_avg_response_time_mean'], default['high_priority_avg_response_time_mean']):.2f} |"
        )

    lines.append("")
    lines.append("## 表 5 异常反馈实验结果")
    lines.append("")
    lines.append("| 方法 | 完成任务数均值 | 总路径长度均值 | 总巡检时间均值/s | 高优先级响应时间/s | 异常任务优先处理率/% | 异常平均响应时间/s | 重规划次数均值 |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for method in ["FS", "NNF", "AStarOnly", "Proposed-Balanced"]:
        r = abnormal[method]
        lines.append(
            f"| {method} | {fmt(r['completed_task_num_mean'])} | {fmt(r['total_path_length_mean'])} | {fmt(r['total_inspection_time_mean'])} | {fmt(r['high_priority_avg_response_time_mean'])} | {fmt(r['abnormal_priority_rate_mean'])} | {fmt(r['abnormal_avg_response_time_mean'])} | {fmt(r['replanning_count_mean'])} |"
        )

    lines.append("")
    lines.append("## 表 6 Proposed-Balanced 相比 AStarOnly 的异常实验指标变化率")
    lines.append("")
    lines.append("| 指标 | AStarOnly | Proposed-Balanced | 变化率/% |")
    lines.append("| --- | --- | --- | --- |")
    for label, key in [
        ("总路径长度", "total_path_length_mean"),
        ("总巡检时间", "total_inspection_time_mean"),
        ("高优先级任务平均响应时间", "high_priority_avg_response_time_mean"),
        ("异常任务优先处理率", "abnormal_priority_rate_mean"),
        ("异常平均响应时间", "abnormal_avg_response_time_mean"),
    ]:
        a = float(abnormal["AStarOnly"][key])
        p = float(abnormal["Proposed-Balanced"][key])
        lines.append(f"| {label} | {fmt(a)} | {fmt(p)} | {change(p, a):.2f} |")

    lines.append("")
    lines.append("## 表格使用说明")
    lines.append("")
    lines.append("1. 以上表格均基于二维栅格仿真实验结果生成，并非 ROS2/Nav2 或真实四足机器人实机实验结果。")
    lines.append("2. “变化率/%”统一按 `(Proposed-Balanced - 对比方法) / 对比方法 × 100%` 或 `(当前权重组 - Default) / Default × 100%` 计算。")
    lines.append("3. 正变化率仅表示数值增大，负变化率仅表示数值减小，不直接代表性能提升或降低。")
    lines.append("4. 在总路径长度和总巡检时间指标上，数值越小通常表示路径效率更高。")
    lines.append("5. 在高优先级任务平均响应时间和异常平均响应时间指标上，数值越小表示响应更快。")
    lines.append("6. 在异常任务优先处理率指标上，数值越大表示异常任务更容易在触发后短期内被优先处理。")
    lines.append("7. 当前结果支持的结论是：Proposed-Balanced 通过增加一定路径代价和总巡检时间，换取更短的高优先级任务平均响应时间和更高的异常任务优先处理率。")
    lines.append("8. 当前结果不支持“Proposed-Balanced 路径最短”或“Proposed-Balanced 总巡检时间最短”的表述。")

    OUTPUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {OUTPUT}")


if __name__ == "__main__":
    main()
