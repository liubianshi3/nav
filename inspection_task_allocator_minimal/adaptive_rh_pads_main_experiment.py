from pathlib import Path

from adaptive_experiment_utils import (
    create_grid_map,
    create_tasks,
    print_change_block,
    print_counts,
    print_summary_table,
    run_method,
    save_csv,
    summarize,
)


METHODS = [
    "AStarOnly",
    "TSP-2opt",
    "Greedy-PADS",
    "Priority-Greedy",
    "RH-PADS",
    "RH-PADS-L",
    "A-RH-PADS",
    "A-RH-PADS-L",
]

FIELDNAMES = [
    "seed",
    "method",
    "completed_task_num",
    "total_path_length",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "high_priority_top5_rate",
    "algorithm_runtime_ms",
    "lambda_mean",
    "lambda_min",
    "lambda_max",
    "lambda_std",
    "task_sequence",
]

SUMMARY_METRICS = [
    "completed_task_num",
    "total_path_length",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "high_priority_top5_rate",
    "algorithm_runtime_ms",
    "lambda_mean",
    "lambda_min",
    "lambda_max",
    "lambda_std",
]

CHANGE_METRICS = [
    "total_path_length",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "high_priority_top5_rate",
]


def main():
    width = 30
    height = 30
    obstacle_ratio = 0.2
    task_num = 20
    start_pos = (2, 2)
    robot_speed = 0.6
    inspection_time = 5.0
    seeds = list(range(30))

    results_dir = Path(__file__).resolve().parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    results_path = results_dir / "adaptive_rh_pads_main_results.csv"
    summary_path = results_dir / "adaptive_rh_pads_main_summary.csv"

    rows = []
    for seed in seeds:
        grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
        tasks = create_tasks(grid_map, start_pos, task_num, seed)
        for method in METHODS:
            result = run_method(
                method,
                grid_map,
                tasks,
                start_pos,
                robot_speed,
                inspection_time,
            )
            rows.append(
                {
                    "seed": seed,
                    "method": method,
                    "completed_task_num": result["completed_task_num"],
                    "total_path_length": result["total_path_length"],
                    "total_inspection_time": result["total_inspection_time"],
                    "high_priority_avg_response_time": result["high_priority_avg_response_time"],
                    "priority_weighted_completion_time": result["priority_weighted_completion_time"],
                    "high_priority_top5_rate": result["high_priority_top5_rate"],
                    "algorithm_runtime_ms": result["algorithm_runtime_ms"],
                    "lambda_mean": result.get("lambda_mean", 0.0),
                    "lambda_min": result.get("lambda_min", 0.0),
                    "lambda_max": result.get("lambda_max", 0.0),
                    "lambda_std": result.get("lambda_std", 0.0),
                    "task_sequence": "->".join(result["task_sequence"]),
                }
            )

    save_csv(rows, results_path, FIELDNAMES)
    summary_rows = summarize(rows, METHODS, SUMMARY_METRICS)
    summary_fieldnames = ["method"]
    for metric in SUMMARY_METRICS:
        summary_fieldnames.extend([f"{metric}_mean", f"{metric}_std"])
    save_csv(summary_rows, summary_path, summary_fieldnames)

    print("Expected rows: 30 seeds x 8 methods = 240 rows")
    print_counts(rows, METHODS)
    print_summary_table(summary_rows, CHANGE_METRICS)
    summary_map = {row["method"]: row for row in summary_rows}
    print_change_block(summary_map, "A-RH-PADS", "RH-PADS", CHANGE_METRICS)
    print_change_block(summary_map, "A-RH-PADS-L", "RH-PADS-L", CHANGE_METRICS)
    print_change_block(summary_map, "A-RH-PADS", "AStarOnly", CHANGE_METRICS)
    print_change_block(summary_map, "A-RH-PADS", "TSP-2opt", CHANGE_METRICS)
    print_change_block(summary_map, "A-RH-PADS", "Priority-Greedy", CHANGE_METRICS)
    print("Lower is better for path, time and response metrics; top5 rate is higher-is-better.")
    print(f"Results saved to: {results_path}")
    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    main()
