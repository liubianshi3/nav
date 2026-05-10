import os
import sys

import pandas as pd


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(ROOT_DIR, "results")
INPUT_CSV = os.path.join(RESULTS_DIR, "all_methods_results.csv")
SUMMARY_BY_TASK_NUM = os.path.join(RESULTS_DIR, "summary_by_task_num.csv")
SUMMARY_BY_OBSTACLE_RATIO = os.path.join(RESULTS_DIR, "summary_by_obstacle_ratio.csv")
SUMMARY_OVERALL = os.path.join(RESULTS_DIR, "summary_overall.csv")

METHOD_ORDER = ["FS", "NNF", "AStarOnly", "Proposed"]
METRICS = [
    "total_path_length",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "algorithm_runtime_ms",
]


def require_input_csv(path):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"未找到实验结果文件：{path}\n"
            "请先运行：\n"
            "python3 src/inspection_task_allocator/inspection_task_allocator/experiment_runner.py"
        )


def summarize_grouped(df, group_cols):
    grouped = df.groupby(group_cols, sort=False)
    rows = []
    for keys, group in grouped:
        if not isinstance(keys, tuple):
            keys = (keys,)
        row = {col: value for col, value in zip(group_cols, keys)}
        for metric in METRICS:
            row[f"{metric}_mean"] = group[metric].mean()
            row[f"{metric}_std"] = group[metric].std(ddof=1)
        rows.append(row)
    result = pd.DataFrame(rows)
    if not result.empty and "method" in result.columns:
        result["method"] = pd.Categorical(result["method"], categories=METHOD_ORDER, ordered=True)
        result = result.sort_values(group_cols).reset_index(drop=True)
    elif not result.empty:
        result = result.sort_values(group_cols).reset_index(drop=True)
    return result


def print_pivot(title, df, index_col, value_col):
    pivot = df.pivot(index=index_col, columns="method", values=value_col)
    pivot = pivot.reindex(columns=METHOD_ORDER)
    print(f"\n{title}")
    print(pivot.to_string(float_format=lambda x: f"{x:.3f}"))


def print_improvement_table(df):
    print("\nProposed 相比 AStarOnly 的提升比例")
    for task_num in sorted(df["task_num"].unique()):
        subset = df[df["task_num"] == task_num]
        proposed = subset[subset["method"] == "Proposed"].mean(numeric_only=True)
        astar_only = subset[subset["method"] == "AStarOnly"].mean(numeric_only=True)
        if astar_only.empty or proposed.empty:
            continue
        path_reduction = (astar_only["total_path_length"] - proposed["total_path_length"]) / astar_only[
            "total_path_length"
        ] * 100.0
        time_reduction = (astar_only["total_inspection_time"] - proposed["total_inspection_time"]) / astar_only[
            "total_inspection_time"
        ] * 100.0
        response_reduction = (astar_only["high_priority_avg_response_time"] - proposed["high_priority_avg_response_time"]) / astar_only[
            "high_priority_avg_response_time"
        ] * 100.0
        print(f"\nTask num = {task_num}:")
        print("Proposed vs AStarOnly:")
        print(f"- total_path_length reduced by {path_reduction:.2f}%")
        print(f"- total_inspection_time reduced by {time_reduction:.2f}%")
        print(f"- high_priority_avg_response_time reduced by {response_reduction:.2f}%")


def main():
    require_input_csv(INPUT_CSV)
    os.makedirs(RESULTS_DIR, exist_ok=True)

    df = pd.read_csv(INPUT_CSV)
    required_cols = {"task_num", "method", "obstacle_ratio", *METRICS}
    missing = required_cols - set(df.columns)
    if missing:
        raise ValueError(f"输入CSV缺少必要字段: {sorted(missing)}")

    task_summary = summarize_grouped(df, ["task_num", "method"])
    obstacle_summary = summarize_grouped(df, ["obstacle_ratio", "method"])
    overall_summary = summarize_grouped(df, ["method"])

    task_summary.to_csv(SUMMARY_BY_TASK_NUM, index=False)
    obstacle_summary.to_csv(SUMMARY_BY_OBSTACLE_RATIO, index=False)
    overall_summary.to_csv(SUMMARY_OVERALL, index=False)

    print_pivot(
        "不同任务数量下 total_path_length 的均值对比表",
        task_summary,
        index_col="task_num",
        value_col="total_path_length_mean",
    )
    print_pivot(
        "不同任务数量下 total_inspection_time 的均值对比表",
        task_summary,
        index_col="task_num",
        value_col="total_inspection_time_mean",
    )
    print_pivot(
        "不同任务数量下 high_priority_avg_response_time 的均值对比表",
        task_summary,
        index_col="task_num",
        value_col="high_priority_avg_response_time_mean",
    )
    print_pivot(
        "不同障碍物比例下 total_path_length 的均值对比表",
        obstacle_summary,
        index_col="obstacle_ratio",
        value_col="total_path_length_mean",
    )
    print_improvement_table(df)

    print(f"\nsummary_by_task_num.csv -> {SUMMARY_BY_TASK_NUM}")
    print(f"summary_by_obstacle_ratio.csv -> {SUMMARY_BY_OBSTACLE_RATIO}")
    print(f"summary_overall.csv -> {SUMMARY_OVERALL}")


if __name__ == "__main__":
    try:
        main()
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
