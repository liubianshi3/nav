#!/usr/bin/env python3
import argparse
import traceback

from vehicle_sim_experiment import create_grid_map, create_tasks
from vehicle_simulator import VehicleTaskExecutionSimulator, VehicleMethodAdapter, vehicle_to_grid_point
from vehicle_model import DifferentialDriveVehicle
from vehicle_path_follower import PurePursuitLikeFollower
from astar_planner import AStarPlanner


def run_debug(seed, method):
    width = 30
    height = 30
    obstacle_ratio = 0.2
    task_num = 20
    start_pos = (2, 2)
    grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
    methods = [method] if method != "all" else ["AStarOnly", "Proposed-Balanced", "RH-v2-Light", "TSP-2opt", "Priority-Greedy"]
    planner = AStarPlanner(grid_map)

    for m in methods:
        tasks = create_tasks(grid_map, start_pos, task_num, seed)
        print(f"\n===== method={m} seed={seed} =====")
        try:
            task_copy = [t for t in tasks]
            adapter = VehicleMethodAdapter(m, grid_map, task_copy, start_pos, seed=seed)
            vehicle = DifferentialDriveVehicle(start_pos[0], start_pos[1], 0.0, v_max=0.6, omega_max=1.0, dt=0.1, radius=0.2)
            follower = PurePursuitLikeFollower(vehicle)
            current_grid_pos = start_pos
            current_time = 0.0
            completed_sequence = []
            step = 0
            while step < 30:
                pending = [t for t in task_copy if t.status == 0]
                print(f"--- method={m} seed={seed} order={step+1} current_grid_pos={current_grid_pos} current_vehicle_pos={vehicle.position()} pending_task_count={len(pending)}")
                if not pending:
                    print("no pending tasks")
                    break
                selected_task, record = adapter.select_next_task(current_grid_pos, current_time, completed_sequence)
                if selected_task is None:
                    print(f"adapter returned None | error_message={record.get('error_message')}")
                    break
                if not hasattr(selected_task, "task_id"):
                    print("selected task is not object")
                    break
                if selected_task.status == 1:
                    print("selected task already completed")
                    break
                plan = planner.plan(current_grid_pos, selected_task.position)
                path = plan["path"]
                path_length = plan["path_length"]
                if not plan["reachable"]:
                    print("A* unreachable")
                    break
                print(f"selected_task_id={selected_task.task_id} selected_task_position={selected_task.position} planned_path_found=True planned_path_length={path_length}")
                print(f"path_point_count={len(path)} path_first_5={path[:5]} path_last_5={path[-5:]}")
                follower.vehicle.reset(float(current_grid_pos[0]), float(current_grid_pos[1]), 0.0)
                result = follower.follow_path([tuple(map(float, p)) for p in path])
                ratio = result.get("trajectory_to_plan_ratio", 0.0)
                dist_to_goal = vehicle.distance_to(selected_task.position)
                print(f"follower_success={result['success']} vehicle_trajectory_length={result['trajectory_length']} vehicle_execution_time={result['execution_time']} trajectory_to_plan_ratio={ratio}")
                print(f"final_vehicle_position={result['final_position']} distance_to_goal={dist_to_goal} heading_change_sum={result['heading_change_sum']} error_message={result['error_message']}")
                if not result["success"]:
                    print("follower failed")
                    break
                selected_task.status = 1
                current_grid_pos = selected_task.position
                completed_sequence.append(selected_task.task_id)
                current_time += result["execution_time"] + 5.0
                step += 1
            print(f"summary method={m} completed={len(completed_sequence)} sequence={completed_sequence}")
        except Exception:
            print("exception traceback")
            traceback.print_exc()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--method", type=str, default="all")
    parser.add_argument("--max-steps", type=int, default=30)
    args = parser.parse_args()
    run_debug(args.seed, args.method)


if __name__ == "__main__":
    main()
