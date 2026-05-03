from pathlib import Path

from drf_rh_allocator import DRFRHAllocator
from drf_rh_experiment_utils import (
    BALANCED_WEIGHTS,
    create_grid_map,
    create_tasks,
    percent_change,
    print_summary_table,
    summarize,
    write_csv,
)
import copy
import time


VARIANTS = {
    "DRF-RH-Full": {},
    "DRF-RH-Horizon1": {"horizon": 1},
    "DRF-RH-BaseScoreOnly": {
        "use_path_time_penalty": False,
        "use_priority_completion": False,
        "use_abnormal_completion": False,
        "use_deadline_penalty": False,
        "use_topk_bonus": False,
    },
    "DRF-RH-NoPathTimePenalty": {"use_path_time_penalty": False},
    "DRF-RH-NoPriorityCompletion": {"use_priority_completion": False},
    "DRF-RH-NoDeadlinePenalty": {"use_deadline_penalty": False},
    "DRF-RH-NoTopKBonus": {"use_topk_bonus": False},
    "DRF-RH-NoHybridPool": {"use_hybrid_pool": False},
    "DRF-RH-NoDynamicRiskField": {"use_dynamic_risk_field": False},
}

METRICS = [
    "completed_task_num",
    "total_path_length",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "high_priority_top5_rate",
    "algorithm_runtime_ms",
    "priority_response_efficiency",
]


def run_variant(name, params, grid_map, tasks, start_pos, robot_speed, inspection_time):
    tasks_copy = copy.deepcopy(tasks)
    config = {
        "horizon": 4,
        "beam_width": 8,
        "candidate_pool_size": 10,
    }
    config.update(params)
    allocator = DRFRHAllocator(
        grid_map,
        tasks_copy,
        start_pos,
        robot_speed=robot_speed,
        inspection_time=inspection_time,
        **BALANCED_WEIGHTS,
        **config,
    )
    started = time.perf_counter()
    result = allocator.run()
    runtime_ms = (time.perf_counter() - started) * 1000.0
    total_path = result["total_path_length"]
    result["algorithm_runtime_ms"] = runtime_ms
    result["priority_response_efficiency"] = (
        result["high_priority_top5_rate"] / total_path if total_path > 0 else 0.0
    )
    return result


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
        for method, params in VARIANTS.items():
            result = run_variant(method, params, grid_map, tasks, start_pos, robot_speed, inspection_time)
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
                    "priority_response_efficiency": result["priority_response_efficiency"],
                    "task_sequence": "->".join(result["task_sequence"]),
                }
            )

    base_dir = Path(__file__).resolve().parent
    results_path = base_dir / "results" / "drf_rh_ablation_results.csv"
    summary_path = base_dir / "results" / "drf_rh_ablation_summary.csv"
    fields = [
        "seed",
        "method",
        "completed_task_num",
        "total_path_length",
        "total_inspection_time",
        "high_priority_avg_response_time",
        "priority_weighted_completion_time",
        "high_priority_top5_rate",
        "algorithm_runtime_ms",
        "priority_response_efficiency",
        "task_sequence",
    ]
    write_csv(results_path, rows, fields)
    summary = summarize(rows, ["method"], METRICS)
    write_csv(summary_path, summary, list(summary[0].keys()))
    print(f"Total rows: {len(rows)}")
    print({m: sum(1 for row in rows if row['method'] == m) for m in VARIANTS})
    print_summary_table(summary)

    row_map = {row["method"]: row for row in summary}
    for method in VARIANTS:
        if method == "DRF-RH-Full":
            continue
        print(f"DRF-RH-Full vs {method}:")
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
                float(row_map[method][f"{metric}_mean"]),
            )
            print(f"- {metric}: {change:.2f}%")
    print(f"Saved: {results_path}")
    print(f"Saved: {summary_path}")


if __name__ == "__main__":
    main()
