from pathlib import Path

from drf_rh_experiment_utils import (
    create_grid_map,
    create_tasks,
    percent_change,
    print_summary_table,
    run_allocator,
    summarize,
    write_csv,
)


METHODS = [
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

METRICS = [
    "completed_task_num",
    "total_path_length",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "high_priority_top5_rate",
    "algorithm_runtime_ms",
    "path_redundancy_ratio",
    "priority_response_efficiency",
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

    rows = []
    for seed in seeds:
        grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
        tasks = create_tasks(grid_map, start_pos, task_num, seed)
        seed_results = {}
        for method in METHODS:
            result = run_allocator(method, grid_map, tasks, start_pos, robot_speed, inspection_time)
            seed_results[method] = result

        astar_path = seed_results["AStarOnly"]["total_path_length"]
        for method in METHODS:
            result = seed_results[method]
            total_path = result["total_path_length"]
            path_redundancy = total_path / astar_path if astar_path > 0 else 0.0
            efficiency = (
                result["high_priority_top5_rate"] / total_path
                if total_path > 0
                else 0.0
            )
            rows.append(
                {
                    "seed": seed,
                    "method": method,
                    "completed_task_num": result["completed_task_num"],
                    "total_path_length": total_path,
                    "total_inspection_time": result["total_inspection_time"],
                    "high_priority_avg_response_time": result["high_priority_avg_response_time"],
                    "priority_weighted_completion_time": result["priority_weighted_completion_time"],
                    "high_priority_top5_rate": result["high_priority_top5_rate"],
                    "algorithm_runtime_ms": result["algorithm_runtime_ms"],
                    "path_redundancy_ratio": path_redundancy,
                    "priority_response_efficiency": efficiency,
                    "task_sequence": "->".join(result["task_sequence"]),
                }
            )

    base_dir = Path(__file__).resolve().parent
    results_path = base_dir / "results" / "drf_rh_main_results.csv"
    summary_path = base_dir / "results" / "drf_rh_main_summary.csv"
    result_fields = [
        "seed",
        "method",
        "completed_task_num",
        "total_path_length",
        "total_inspection_time",
        "high_priority_avg_response_time",
        "priority_weighted_completion_time",
        "high_priority_top5_rate",
        "algorithm_runtime_ms",
        "path_redundancy_ratio",
        "priority_response_efficiency",
        "task_sequence",
    ]
    write_csv(results_path, rows, result_fields)
    summary = summarize(rows, ["method"], METRICS)
    write_csv(summary_path, summary, list(summary[0].keys()))

    counts = {method: sum(1 for row in rows if row["method"] == method) for method in METHODS}
    print(f"Total rows: {len(rows)}")
    print(counts)
    print_summary_table(summary)

    row_map = {row["method"]: row for row in summary}
    for target in ["RH-Proposed-v2", "Priority-Greedy", "Deadline-Greedy", "TSP-2opt"]:
        print(f"DRF-RH-Full vs {target}:")
        for metric in [
            "total_path_length",
            "total_inspection_time",
            "high_priority_avg_response_time",
            "priority_weighted_completion_time",
            "high_priority_top5_rate",
            "priority_response_efficiency",
            "algorithm_runtime_ms",
        ]:
            change = percent_change(
                float(row_map["DRF-RH-Full"][f"{metric}_mean"]),
                float(row_map[target][f"{metric}_mean"]),
            )
            print(f"- {metric}: {change:.2f}%")
    print("DRF-RH-Light vs RH-Proposed-v2:")
    for metric in [
        "total_path_length",
        "total_inspection_time",
        "high_priority_avg_response_time",
        "priority_weighted_completion_time",
        "high_priority_top5_rate",
        "priority_response_efficiency",
        "algorithm_runtime_ms",
    ]:
        change = percent_change(
            float(row_map["DRF-RH-Light"][f"{metric}_mean"]),
            float(row_map["RH-Proposed-v2"][f"{metric}_mean"]),
        )
        print(f"- {metric}: {change:.2f}%")
    print(f"Saved: {results_path}")
    print(f"Saved: {summary_path}")


if __name__ == "__main__":
    main()
