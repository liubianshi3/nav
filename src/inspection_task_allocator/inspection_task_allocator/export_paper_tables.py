import os
import sys

import pandas as pd


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(ROOT_DIR, "results")
SUMMARY_BY_TASK_NUM = os.path.join(RESULTS_DIR, "summary_by_task_num.csv")
SUMMARY_BY_OBSTACLE_RATIO = os.path.join(RESULTS_DIR, "summary_by_obstacle_ratio.csv")
SUMMARY_OVERALL = os.path.join(RESULTS_DIR, "summary_overall.csv")
OUTPUT_MD = os.path.join(RESULTS_DIR, "paper_tables.md")
METHOD_ORDER = ["FS", "NNF", "AStarOnly", "Proposed"]


def require_file(path, label):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"未找到{label}文件：{path}\n"
            "请先运行统计脚本生成 summary CSV。"
        )


def format_df(df, index_name=None):
    formatted = df.copy()
    for col in formatted.columns:
        if col != index_name:
            formatted[col] = formatted[col].map(lambda x: f"{x:.2f}" if pd.notna(x) else "-")
    return formatted


def markdown_table(df):
    columns = [str(col) for col in df.columns]
    rows = ["| " + " | ".join(columns) + " |"]
    rows.append("| " + " | ".join(["---"] * len(columns)) + " |")
    for _, row in df.iterrows():
        rows.append("| " + " | ".join(str(value) for value in row.tolist()) + " |")
    return "\n".join(rows)


def dataframe_table_section(df, index_name, title):
    df = df.copy()
    method_cols = [method for method in METHOD_ORDER if method in df.columns]
    if index_name in df.columns:
        df = df[[index_name, *method_cols]]
    else:
        df = df[method_cols]
    df = format_df(df, index_name=index_name)
    md = [f"## {title}", "", markdown_table(df), ""]
    return "\n".join(md)


def overall_table_section(df):
    keep_cols = [
        "method",
        "total_path_length_mean",
        "total_inspection_time_mean",
        "high_priority_avg_response_time_mean",
        "algorithm_runtime_ms_mean",
    ]
    df = df[keep_cols].copy()
    for col in keep_cols[1:]:
        df[col] = df[col].map(lambda x: f"{x:.2f}" if pd.notna(x) else "-")
    return "\n".join([
        "## 表 5 各方法整体性能对比",
        "",
        markdown_table(df),
        "",
    ])


def improvement_markdown(task_summary):
    rows = ["## Proposed 相比 AStarOnly 的提升比例", "", "| 任务数量 | 总路径长度降低/% | 总巡检时间降低/% | 高优先级响应时间降低/% |", "|---:|---:|---:|---:|"]
    for task_num in sorted(task_summary["task_num"].unique()):
        subset = task_summary[task_summary["task_num"] == task_num]
        proposed = subset[subset["method"] == "Proposed"].iloc[0]
        astar = subset[subset["method"] == "AStarOnly"].iloc[0]
        path_red = (astar["total_path_length_mean"] - proposed["total_path_length_mean"]) / astar["total_path_length_mean"] * 100.0
        time_red = (astar["total_inspection_time_mean"] - proposed["total_inspection_time_mean"]) / astar["total_inspection_time_mean"] * 100.0
        resp_red = (astar["high_priority_avg_response_time_mean"] - proposed["high_priority_avg_response_time_mean"]) / astar["high_priority_avg_response_time_mean"] * 100.0
        rows.append(f"| {int(task_num)} | {path_red:.2f} | {time_red:.2f} | {resp_red:.2f} |")
    rows.append("")
    return "\n".join(rows)


def main():
    require_file(SUMMARY_BY_TASK_NUM, "summary_by_task_num.csv")
    require_file(SUMMARY_BY_OBSTACLE_RATIO, "summary_by_obstacle_ratio.csv")
    require_file(SUMMARY_OVERALL, "summary_overall.csv")

    task_summary = pd.read_csv(SUMMARY_BY_TASK_NUM)
    obstacle_summary = pd.read_csv(SUMMARY_BY_OBSTACLE_RATIO)
    overall_summary = pd.read_csv(SUMMARY_OVERALL)

    pieces = [
        dataframe_table_section(task_summary.pivot(index="task_num", columns="method", values="total_path_length_mean").reset_index(), "task_num", "表 1 不同任务数量下总路径长度对比"),
        dataframe_table_section(task_summary.pivot(index="task_num", columns="method", values="total_inspection_time_mean").reset_index(), "task_num", "表 2 不同任务数量下总巡检时间对比"),
        dataframe_table_section(task_summary.pivot(index="task_num", columns="method", values="high_priority_avg_response_time_mean").reset_index(), "task_num", "表 3 不同任务数量下高优先级任务平均响应时间对比"),
        dataframe_table_section(obstacle_summary.pivot(index="obstacle_ratio", columns="method", values="total_path_length_mean").reset_index(), "obstacle_ratio", "表 4 不同障碍物比例下总路径长度对比"),
        overall_table_section(overall_summary),
        improvement_markdown(task_summary),
    ]

    os.makedirs(RESULTS_DIR, exist_ok=True)
    with open(OUTPUT_MD, "w", encoding="utf-8") as f:
        f.write("\n".join(pieces))

    print(f"paper_tables.md 保存路径: {OUTPUT_MD}")
    print("共导出了 6 个表格")


if __name__ == "__main__":
    try:
        main()
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
