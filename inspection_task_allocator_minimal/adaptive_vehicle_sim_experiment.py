import copy
import csv
import json
import statistics
from pathlib import Path

from adaptive_experiment_utils import (
    compute_high_priority_avg_response_time,
    compute_priority_weighted_completion_time,
    create_grid_map,
    create_tasks,
    print_change_block,
    print_counts,
    print_summary_table,
    save_csv,
    summarize,
)
from adaptive_rh_pads_allocator import AdaptiveRHPADSAllocator
from task_model import InspectionTask
from vehicle_model import DifferentialDriveVehicle
from vehicle_path_follower import PurePursuitLikeFollower
from vehicle_simulator import VehicleMethodAdapter, grid_to_vehicle_point
from astar_planner import AStarPlanner


METHODS = [
    "A-RH-PADS-L",
    "RH-PADS-L",
    "AStarOnly",
    "TSP-2opt",
    "Greedy-PADS",
    "Priority-Greedy",
]

ADAPTER_METHOD_MAP = {
    "AStarOnly": "AStarOnly",
    "TSP-2opt": "TSP-2opt",
    "Greedy-PADS": "Proposed-Balanced",
    "Priority-Greedy": "Priority-Greedy",
    "RH-PADS-L": "RH-v2-Light",
}

FIELDNAMES = [
    "seed",
    "method",
    "completed_task_num",
    "total_path_length",
    "total_planned_path_length",
    "vehicle_trajectory_length",
    "vehicle_execution_time",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "heading_change_sum",
    "goal_success_rate",
    "trajectory_to_plan_ratio",
    "lambda_mean",
    "lambda_min",
    "lambda_max",
    "lambda_std",
    "failed_task_num",
    "unreachable_task_num",
    "follower_failed_num",
    "adapter_failed_num",
    "task_sequence",
]

SUMMARY_METRICS = [
    "completed_task_num",
    "total_path_length",
    "total_planned_path_length",
    "vehicle_trajectory_length",
    "vehicle_execution_time",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "heading_change_sum",
    "goal_success_rate",
    "trajectory_to_plan_ratio",
    "lambda_mean",
    "lambda_min",
    "lambda_max",
    "lambda_std",
    "failed_task_num",
    "unreachable_task_num",
    "follower_failed_num",
    "adapter_failed_num",
]


def lambda_stats(lambda_sequence):
    if not lambda_sequence:
        return 0.0, 0.0, 0.0, 0.0
    return (
        statistics.mean(lambda_sequence),
        min(lambda_sequence),
        max(lambda_sequence),
        statistics.stdev(lambda_sequence) if len(lambda_sequence) > 1 else 0.0,
    )


def select_adaptive_task(allocator, current_grid_pos, current_time):
    selected_task, record = allocator.select_next_task(current_grid_pos, current_time)
    if selected_task is None:
        return None, {
            "method": "A-RH-PADS-L",
            "selected_task_id": None,
            "error_message": "A_RH_PADS_L_no_reachable_task",
        }
    path_info = record.pop("path_info")
    return selected_task, {
        "method": "A-RH-PADS-L",
        "selected_task_id": selected_task.task_id,
        "score": record["adaptive_sequence_score"],
        "estimated_distance": record["planned_path_length"],
        "planned_path_length": record["planned_path_length"],
        "priority": selected_task.priority,
        "risk": selected_task.risk,
        "abnormal_weight": selected_task.abnormal_weight,
        "path": path_info["path"],
        "reachable": True,
        "turn_count": path_info["turn_count"],
        "obstacle_nearby_count": path_info["obstacle_nearby_count"],
        **record,
    }


def run_vehicle_method(method, grid_map, tasks, start_pos, seed):
    tasks_copy = copy.deepcopy(tasks)
    planner = AStarPlanner(grid_map)
    adaptive_allocator = None
    adapter = None
    if method == "A-RH-PADS-L":
        adaptive_allocator = AdaptiveRHPADSAllocator(
            grid_map=grid_map,
            tasks=tasks_copy,
            start_pos=start_pos,
            robot_speed=0.6,
            inspection_time=5.0,
            horizon=3,
            beam_width=5,
            candidate_pool_size=6,
        )
    else:
        adapter = VehicleMethodAdapter(
            method_name=ADAPTER_METHOD_MAP[method],
            grid_map=grid_map,
            tasks=tasks_copy,
            start_pos=start_pos,
            robot_speed=0.6,
            inspection_time=5.0,
            high_priority_threshold=0.7,
            seed=seed,
        )

    vehicle = DifferentialDriveVehicle(
        start_pos[0],
        start_pos[1],
        0.0,
        v_max=0.6,
        omega_max=1.0,
        dt=0.1,
        radius=0.2,
    )
    follower = PurePursuitLikeFollower(vehicle)

    current_time = 0.0
    current_grid_pos = start_pos
    task_sequence = []
    execution_records = []
    trajectory = []
    total_planned_path_length = 0.0
    vehicle_trajectory_length = 0.0
    vehicle_execution_time = 0.0
    heading_change_sum = 0.0
    goal_success_count = 0
    failed_task_num = 0
    unreachable_task_num = 0
    follower_failed_num = 0
    adapter_failed_num = 0
    task_finish_times = {}
    lambda_sequence = []

    while True:
        pending = [task for task in tasks_copy if task.status == 0]
        if not pending:
            break

        if adaptive_allocator is not None:
            selected_task, record = select_adaptive_task(
                adaptive_allocator,
                current_grid_pos,
                current_time,
            )
        else:
            selected_task, record = adapter.select_next_task(
                current_grid_pos,
                current_time,
                task_sequence,
            )
            record["method"] = method

        if selected_task is None:
            adapter_failed_num += 1
            failed_task_num += 1
            execution_records.append(
                {
                    **record,
                    "order": len(task_sequence) + 1,
                    "selected_task_id": None,
                    "planned_path_length": 0.0,
                    "vehicle_trajectory_length": 0.0,
                    "vehicle_execution_time": 0.0,
                    "success": False,
                    "finish_time": current_time,
                }
            )
            break

        if not isinstance(selected_task, InspectionTask) or selected_task.status == 1:
            adapter_failed_num += 1
            failed_task_num += 1
            execution_records.append(
                {
                    **record,
                    "order": len(task_sequence) + 1,
                    "selected_task_id": getattr(selected_task, "task_id", None),
                    "planned_path_length": 0.0,
                    "vehicle_trajectory_length": 0.0,
                    "vehicle_execution_time": 0.0,
                    "success": False,
                    "finish_time": current_time,
                    "error_message": "invalid_or_completed_task",
                }
            )
            break

        plan = planner.plan(current_grid_pos, selected_task.position)
        if not plan.get("reachable", False):
            unreachable_task_num += 1
            failed_task_num += 1
            selected_task.status = 2
            execution_records.append(
                {
                    **record,
                    "order": len(task_sequence) + 1,
                    "selected_task_id": selected_task.task_id,
                    "planned_path_length": 0.0,
                    "vehicle_trajectory_length": 0.0,
                    "vehicle_execution_time": 0.0,
                    "success": False,
                    "finish_time": current_time,
                    "error_message": "A* unreachable",
                }
            )
            continue

        path = [grid_to_vehicle_point(point) for point in plan["path"]]
        follower.vehicle.reset(*grid_to_vehicle_point(current_grid_pos), theta=vehicle.theta)
        follow_result = follower.follow_path(path)

        total_planned_path_length += plan["path_length"]
        vehicle_trajectory_length += follow_result["trajectory_length"]
        vehicle_execution_time += follow_result["execution_time"]
        heading_change_sum += follow_result["heading_change_sum"]
        trajectory.extend(follow_result["trajectory"])

        record.update(
            {
                "planned_path_length": plan["path_length"],
                "vehicle_trajectory_length": follow_result["trajectory_length"],
                "vehicle_execution_time": follow_result["execution_time"],
                "trajectory_to_plan_ratio": follow_result["trajectory_to_plan_ratio"],
                "final_vehicle_position": follow_result["final_position"],
                "success": follow_result["success"],
                "heading_change_sum": follow_result["heading_change_sum"],
                "error_message": follow_result["error_message"] or record.get("error_message", ""),
                "warning": follow_result.get("warning", ""),
            }
        )

        if not follow_result["success"]:
            follower_failed_num += 1
            failed_task_num += 1
            selected_task.status = 2
            execution_records.append(
                {
                    **record,
                    "order": len(task_sequence) + 1,
                    "selected_task_id": selected_task.task_id,
                    "finish_time": current_time,
                }
            )
            continue

        current_time += follow_result["execution_time"] + 5.0
        selected_task.status = 1
        current_grid_pos = selected_task.position
        vehicle.reset(*follow_result["final_position"], theta=vehicle.theta)
        task_sequence.append(selected_task.task_id)
        task_finish_times[selected_task.task_id] = current_time
        goal_success_count += 1

        if adaptive_allocator is not None:
            adaptive_allocator.current_pos = current_grid_pos
            adaptive_allocator.total_path_length += plan["path_length"]
            adaptive_allocator.total_inspection_time = current_time
            adaptive_allocator.task_sequence.append(selected_task.task_id)
            adaptive_allocator.task_finish_times[selected_task.task_id] = current_time
            lambda_sequence.append(record.get("lambda_t", 0.0))
            adaptive_allocator.lambda_sequence.append(record.get("lambda_t", 0.0))

        execution_records.append(
            {
                **record,
                "order": len(task_sequence),
                "selected_task_id": selected_task.task_id,
                "finish_time": current_time,
            }
        )

    high_priority_avg_response_time = compute_high_priority_avg_response_time(
        tasks_copy,
        task_finish_times,
        threshold=0.7,
    )
    priority_weighted_completion_time = compute_priority_weighted_completion_time(
        tasks_copy,
        task_finish_times,
    )
    goal_success_rate = goal_success_count / len(task_sequence) * 100.0 if task_sequence else 0.0
    trajectory_to_plan_ratio = (
        vehicle_trajectory_length / total_planned_path_length
        if total_planned_path_length > 0
        else 0.0
    )
    lambda_mean, lambda_min, lambda_max, lambda_std = lambda_stats(lambda_sequence)

    return {
        "method": method,
        "completed_task_num": len(task_sequence),
        "total_path_length": total_planned_path_length,
        "total_planned_path_length": total_planned_path_length,
        "vehicle_trajectory_length": vehicle_trajectory_length,
        "vehicle_execution_time": vehicle_execution_time,
        "total_inspection_time": current_time,
        "high_priority_avg_response_time": high_priority_avg_response_time,
        "priority_weighted_completion_time": priority_weighted_completion_time,
        "heading_change_sum": heading_change_sum,
        "goal_success_rate": goal_success_rate,
        "trajectory_to_plan_ratio": trajectory_to_plan_ratio,
        "lambda_mean": lambda_mean,
        "lambda_min": lambda_min,
        "lambda_max": lambda_max,
        "lambda_std": lambda_std,
        "failed_task_num": failed_task_num,
        "unreachable_task_num": unreachable_task_num,
        "follower_failed_num": follower_failed_num,
        "adapter_failed_num": adapter_failed_num,
        "task_sequence": task_sequence,
        "execution_records": execution_records,
        "trajectory": trajectory,
    }


def save_json(rows, path):
    with path.open("w", encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=2)


def main():
    width = 30
    height = 30
    obstacle_ratio = 0.2
    task_num = 20
    start_pos = (2, 2)
    seeds = list(range(30))

    results_dir = Path(__file__).resolve().parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    results_path = results_dir / "adaptive_vehicle_sim_results.csv"
    summary_path = results_dir / "adaptive_vehicle_sim_summary.csv"
    records_path = results_dir / "adaptive_vehicle_sim_records.json"

    rows = []
    records = []
    for seed in seeds:
        grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
        tasks = create_tasks(grid_map, start_pos, task_num, seed)
        for method in METHODS:
            result = run_vehicle_method(method, grid_map, tasks, start_pos, seed)
            rows.append(
                {
                    "seed": seed,
                    "method": method,
                    "completed_task_num": result["completed_task_num"],
                    "total_path_length": result["total_path_length"],
                    "total_planned_path_length": result["total_planned_path_length"],
                    "vehicle_trajectory_length": result["vehicle_trajectory_length"],
                    "vehicle_execution_time": result["vehicle_execution_time"],
                    "total_inspection_time": result["total_inspection_time"],
                    "high_priority_avg_response_time": result["high_priority_avg_response_time"],
                    "priority_weighted_completion_time": result["priority_weighted_completion_time"],
                    "heading_change_sum": result["heading_change_sum"],
                    "goal_success_rate": result["goal_success_rate"],
                    "trajectory_to_plan_ratio": result["trajectory_to_plan_ratio"],
                    "lambda_mean": result["lambda_mean"],
                    "lambda_min": result["lambda_min"],
                    "lambda_max": result["lambda_max"],
                    "lambda_std": result["lambda_std"],
                    "failed_task_num": result["failed_task_num"],
                    "unreachable_task_num": result["unreachable_task_num"],
                    "follower_failed_num": result["follower_failed_num"],
                    "adapter_failed_num": result["adapter_failed_num"],
                    "task_sequence": "->".join(result["task_sequence"]),
                }
            )
            records.append({"seed": seed, **result})

    save_csv(rows, results_path, FIELDNAMES)
    summary_rows = summarize(rows, METHODS, SUMMARY_METRICS)
    summary_fieldnames = ["method"]
    for metric in SUMMARY_METRICS:
        summary_fieldnames.extend([f"{metric}_mean", f"{metric}_std"])
    save_csv(summary_rows, summary_path, summary_fieldnames)
    save_json(records, records_path)

    print("Expected rows: 30 seeds x 6 methods = 180 rows")
    print_counts(rows, METHODS)
    print_summary_table(
        summary_rows,
        [
            "completed_task_num",
            "total_planned_path_length",
            "vehicle_trajectory_length",
            "vehicle_execution_time",
            "high_priority_avg_response_time",
            "goal_success_rate",
        ],
    )
    summary_map = {row["method"]: row for row in summary_rows}
    compare_metrics = [
        "total_planned_path_length",
        "vehicle_trajectory_length",
        "vehicle_execution_time",
        "high_priority_avg_response_time",
    ]
    print_change_block(summary_map, "A-RH-PADS-L", "RH-PADS-L", compare_metrics)
    print_change_block(summary_map, "A-RH-PADS-L", "TSP-2opt", compare_metrics)
    print_change_block(summary_map, "A-RH-PADS-L", "Priority-Greedy", compare_metrics)
    print(f"Results saved to: {results_path}")
    print(f"Summary saved to: {summary_path}")
    print(f"Records saved to: {records_path}")


if __name__ == "__main__":
    main()
