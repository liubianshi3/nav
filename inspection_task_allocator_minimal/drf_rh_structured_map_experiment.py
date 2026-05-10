from pathlib import Path

from drf_rh_experiment_utils import (
    percent_change,
    print_summary_table,
    run_allocator,
    summarize,
    write_csv,
)
from structured_maps import (
    generate_bottleneck_map,
    generate_corridor_map,
    generate_room_corridor_map,
    render_map_with_tasks,
    sample_tasks_on_free_cells,
)


METHODS = [
    "AStarOnly",
    "Proposed-Balanced",
    "RH-Proposed-v2",
    "Priority-Greedy",
    "TSP-2opt",
    "DRF-RH-Full",
    "DRF-RH-Light",
]

MAP_BUILDERS = {
    "corridor": generate_corridor_map,
    "room_corridor": generate_room_corridor_map,
    "bottleneck": generate_bottleneck_map,
}

METRICS = [
    "completed_task_num",
    "total_path_length",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "high_priority_top5_rate",
    "algorithm_runtime_ms",
]


def main():
    task_num = 20
    robot_speed = 0.6
    inspection_time = 5.0
    seeds = list(range(20))
    rows = []
    base_dir = Path(__file__).resolve().parent
    figure_dir = base_dir / "results" / "figures" / "structured_maps"

    for map_type, builder in MAP_BUILDERS.items():
        example_grid, example_start = builder()
        example_tasks = sample_tasks_on_free_cells(example_grid, task_num, 0, example_start)
        render_map_with_tasks(
            example_grid,
            example_tasks,
            example_start,
            figure_dir / f"{map_type}_map_example.png",
        )

        for seed in seeds:
            grid_map, start_pos = builder()
            tasks = sample_tasks_on_free_cells(grid_map, task_num, seed, start_pos)
            for method in METHODS:
                result = run_allocator(method, grid_map, tasks, start_pos, robot_speed, inspection_time)
                rows.append(
                    {
                        "map_type": map_type,
                        "seed": seed,
                        "method": method,
                        "completed_task_num": result["completed_task_num"],
                        "total_path_length": result["total_path_length"],
                        "total_inspection_time": result["total_inspection_time"],
                        "high_priority_avg_response_time": result["high_priority_avg_response_time"],
                        "priority_weighted_completion_time": result["priority_weighted_completion_time"],
                        "high_priority_top5_rate": result["high_priority_top5_rate"],
                        "algorithm_runtime_ms": result["algorithm_runtime_ms"],
                        "task_sequence": "->".join(result["task_sequence"]),
                    }
                )

    results_path = base_dir / "results" / "drf_rh_structured_results.csv"
    summary_path = base_dir / "results" / "drf_rh_structured_summary.csv"
    fields = [
        "map_type",
        "seed",
        "method",
        "completed_task_num",
        "total_path_length",
        "total_inspection_time",
        "high_priority_avg_response_time",
        "priority_weighted_completion_time",
        "high_priority_top5_rate",
        "algorithm_runtime_ms",
        "task_sequence",
    ]
    write_csv(results_path, rows, fields)
    summary = summarize(rows, ["map_type", "method"], METRICS)
    write_csv(summary_path, summary, list(summary[0].keys()))

    print(f"Total rows: {len(rows)}")
    for map_type in MAP_BUILDERS:
        print(f"Map type: {map_type}")
        items = [row for row in summary if row["map_type"] == map_type]
        print_summary_table(items)
        row_map = {row["method"]: row for row in items}
        for target in ["RH-Proposed-v2", "TSP-2opt", "Priority-Greedy"]:
            print(f"DRF-RH-Full vs {target}:")
            for metric in [
                "total_path_length",
                "total_inspection_time",
                "high_priority_avg_response_time",
                "high_priority_top5_rate",
            ]:
                change = percent_change(
                    float(row_map["DRF-RH-Full"][f"{metric}_mean"]),
                    float(row_map[target][f"{metric}_mean"]),
                )
                print(f"- {metric}: {change:.2f}%")
    print(f"Saved: {results_path}")
    print(f"Saved: {summary_path}")
    print(f"Saved maps: {figure_dir}")


if __name__ == "__main__":
    main()
