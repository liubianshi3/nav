import copy
import random

from astar_planner import AStarPlanner
from task_model import InspectionTask
from vehicle_model import DifferentialDriveVehicle
from vehicle_path_follower import PurePursuitLikeFollower


def vehicle_to_grid_point(pos):
    return (int(round(pos[0])), int(round(pos[1])))


def grid_to_vehicle_point(p):
    return (float(p[0]), float(p[1]))


class VehicleMethodAdapter:
    def __init__(self, method_name, grid_map, tasks, start_pos, robot_speed=0.6, inspection_time=5.0, high_priority_threshold=0.7, seed=0):
        self.method_name = method_name
        self.grid_map = grid_map
        self.tasks = tasks
        self.start_pos = start_pos
        self.robot_speed = robot_speed
        self.inspection_time = inspection_time
        self.high_priority_threshold = high_priority_threshold
        self.seed = seed
        self.rng = random.Random(seed + 5000)
        self.planner = AStarPlanner(grid_map)
        self._tsp_sequence = None
        self._tsp_index = 0

    def _pending_tasks(self):
        return [t for t in self.tasks if t.status == 0]

    def _plan(self, start, goal):
        return self.planner.plan(start, goal)

    def _make_record(self, **kwargs):
        record = {
            "method": self.method_name,
            "selected_task_id": None,
            "score": None,
            "estimated_distance": None,
            "planned_path_length": None,
            "priority": None,
            "risk": None,
            "abnormal_weight": None,
            "error_message": "",
        }
        record.update(kwargs)
        return record

    def _select_astar_only(self, current_grid_pos):
        best_task = None
        best_record = None
        for task in self._pending_tasks():
            plan = self._plan(current_grid_pos, task.position)
            if not plan["reachable"]:
                continue
            path_length = plan["path_length"]
            score = -float(path_length)
            if best_task is None or path_length < best_record["planned_path_length"]:
                best_task = task
                best_record = self._make_record(
                    selected_task_id=task.task_id,
                    score=score,
                    estimated_distance=float(path_length),
                    planned_path_length=float(path_length),
                    priority=task.priority,
                    risk=task.risk,
                    abnormal_weight=task.abnormal_weight,
                )
                best_record["path"] = plan["path"]
                best_record["reachable"] = True
                best_record["turn_count"] = plan["turn_count"]
                best_record["obstacle_nearby_count"] = plan["obstacle_nearby_count"]
        if best_task is None:
            return None, self._make_record(error_message="AStarOnly_no_reachable_task")
        return best_task, best_record

    def _build_tsp_sequence(self):
        remaining = list(self.tasks)
        seq = []
        pos = self.start_pos
        while remaining:
            candidates = []
            for task in remaining:
                plan = self._plan(pos, task.position)
                if plan["reachable"]:
                    candidates.append((plan["path_length"], task.task_id, task))
            if not candidates:
                break
            candidates.sort(key=lambda x: (x[0], x[1]))
            _, _, task = candidates[0]
            seq.append(task.task_id)
            pos = task.position
            remaining = [t for t in remaining if t.task_id != task.task_id]
        self._tsp_sequence = seq
        self._tsp_index = 0

    def _select_tsp_2opt(self, current_grid_pos):
        if self._tsp_sequence is None:
            self._build_tsp_sequence()
        pending_ids = {t.task_id for t in self._pending_tasks()}
        while self._tsp_index < len(self._tsp_sequence):
            task_id = self._tsp_sequence[self._tsp_index]
            self._tsp_index += 1
            if task_id not in pending_ids:
                continue
            task = next((t for t in self.tasks if t.task_id == task_id and t.status == 0), None)
            if task is None:
                continue
            plan = self._plan(current_grid_pos, task.position)
            if not plan["reachable"]:
                continue
            record = self._make_record(
                selected_task_id=task.task_id,
                score=-float(self._tsp_index),
                estimated_distance=float(plan["path_length"]),
                planned_path_length=float(plan["path_length"]),
                priority=task.priority,
                risk=task.risk,
                abnormal_weight=task.abnormal_weight,
            )
            record["tsp_sequence"] = list(self._tsp_sequence)
            record["sequence_index"] = self._tsp_index - 1
            record["path"] = plan["path"]
            record["reachable"] = True
            record["turn_count"] = plan["turn_count"]
            record["obstacle_nearby_count"] = plan["obstacle_nearby_count"]
            return task, record
        return None, self._make_record(error_message="TSP_2opt_no_reachable_pending_task")

    def _select_priority_greedy(self, current_grid_pos):
        reachable = []
        for task in self._pending_tasks():
            plan = self._plan(current_grid_pos, task.position)
            if plan["reachable"]:
                reachable.append((task, plan))
        if not reachable:
            return None, self._make_record(error_message="PriorityGreedy_no_reachable_task")
        l_min = min(item[1]["path_length"] for item in reachable)
        l_max = max(item[1]["path_length"] for item in reachable)
        denom = max(1e-6, l_max - l_min)
        best_task = None
        best_record = None
        best_score = -float("inf")
        for task, plan in reachable:
            distance_norm = (plan["path_length"] - l_min) / denom
            score = 0.45 * task.priority + 0.25 * task.risk + 0.15 * task.abnormal_weight - 0.15 * distance_norm
            if score > best_score:
                best_score = score
                best_task = task
                best_record = self._make_record(
                    selected_task_id=task.task_id,
                    score=score,
                    estimated_distance=float(plan["path_length"]),
                    planned_path_length=float(plan["path_length"]),
                    priority=task.priority,
                    risk=task.risk,
                    abnormal_weight=task.abnormal_weight,
                )
                best_record["path"] = plan["path"]
                best_record["reachable"] = True
                best_record["turn_count"] = plan["turn_count"]
                best_record["obstacle_nearby_count"] = plan["obstacle_nearby_count"]
        return best_task, best_record

    def _select_proposed(self, current_grid_pos):
        reachable = []
        for task in self._pending_tasks():
            plan = self._plan(current_grid_pos, task.position)
            if plan["reachable"]:
                reachable.append((task, plan))
        if not reachable:
            return None, self._make_record(error_message="Proposed_no_reachable_task")
        l_min = min(item[1]["path_length"] for item in reachable)
        l_max = max(item[1]["path_length"] for item in reachable)
        denom = max(1e-6, l_max - l_min)
        best_task = None
        best_record = None
        best_score = -float("inf")
        for task, plan in reachable:
            distance_norm = (plan["path_length"] - l_min) / denom
            score = 0.22 * task.priority + 0.18 * task.risk + 0.15 * task.abnormal_weight - 0.45 * distance_norm
            if score > best_score:
                best_score = score
                best_task = task
                best_record = self._make_record(
                    selected_task_id=task.task_id,
                    score=score,
                    estimated_distance=float(plan["path_length"]),
                    planned_path_length=float(plan["path_length"]),
                    priority=task.priority,
                    risk=task.risk,
                    abnormal_weight=task.abnormal_weight,
                )
                best_record["path"] = plan["path"]
                best_record["reachable"] = True
                best_record["turn_count"] = plan["turn_count"]
                best_record["obstacle_nearby_count"] = plan["obstacle_nearby_count"]
        return best_task, best_record

    def _select_rh_light(self, current_grid_pos):
        reachable = []
        for task in self._pending_tasks():
            plan = self._plan(current_grid_pos, task.position)
            if plan["reachable"]:
                reachable.append((task, plan))
        if not reachable:
            return None, self._make_record(error_message="RHv2Light_no_reachable_task")

        l_min = min(item[1]["path_length"] for item in reachable)
        l_max = max(item[1]["path_length"] for item in reachable)
        denom = max(1e-6, l_max - l_min)

        candidate_records = []
        for task, plan in reachable:
            distance_norm = (plan["path_length"] - l_min) / denom
            base_score = 0.22 * task.priority + 0.18 * task.risk + 0.15 * task.abnormal_weight - 0.27 * distance_norm
            candidate_records.append({
                "task": task,
                "path": plan["path"],
                "path_length": plan["path_length"],
                "turn_count": plan["turn_count"],
                "obstacle_nearby_count": plan["obstacle_nearby_count"],
                "distance_norm": distance_norm,
                "base_score": base_score,
            })

        pool = {}
        top_k = 2
        for key_fn in [
            lambda r: (-r["base_score"], r["task"].task_id),
            lambda r: (r["path_length"], r["task"].task_id),
            lambda r: (-r["task"].priority, r["task"].task_id),
            lambda r: (-r["task"].abnormal_weight, r["task"].task_id),
        ]:
            for rec in sorted(candidate_records, key=key_fn)[:top_k]:
                pool[rec["task"].task_id] = rec
        pool = list(pool.values())[:6]
        if not pool:
            pool = candidate_records[:6]

        def seq_score(sequence):
            discount = 0.92
            path_scale = max(1.0, 3 * 30)
            time_scale = max(1.0, 3 * ((30 / self.robot_speed) + self.inspection_time))
            cumulative_path = 0.0
            cumulative_time = 0.0
            score = 0.0
            priority_weighted_completion = 0.0
            for k, rec in enumerate(sequence):
                cumulative_path += rec["path_length"]
                cumulative_time += rec["path_length"] / self.robot_speed + self.inspection_time
                score += (discount ** k) * rec["base_score"]
                priority_weighted_completion += rec["task"].priority * (cumulative_time / time_scale)
            cumulative_path_norm = cumulative_path / path_scale
            topk = sequence[: min(3, len(sequence))]
            topk_high_priority_bonus = 0.15 * (sum(1 for rec in topk if rec["task"].priority >= 0.7) / max(1, len(topk)))
            return score - 0.20 * cumulative_path_norm - 0.25 * priority_weighted_completion + topk_high_priority_bonus

        beam = [([], current_grid_pos, pool, 0.0)]
        for _ in range(3):
            new_beam = []
            for seq, pos, remaining, _ in beam:
                if not remaining:
                    new_beam.append((seq, pos, remaining, seq_score(seq)))
                    continue
                ranked = sorted(remaining, key=lambda r: (-r["base_score"], r["path_length"], r["task"].task_id))[:5]
                for rec in ranked:
                    new_seq = seq + [rec]
                    new_remaining = [r for r in remaining if r["task"].task_id != rec["task"].task_id]
                    new_beam.append((new_seq, rec["task"].position, new_remaining, seq_score(new_seq)))
            if not new_beam:
                break
            new_beam.sort(key=lambda item: (-item[3], [r["task"].task_id for r in item[0]]))
            beam = new_beam[:5]
        if not beam:
            return None, self._make_record(error_message="RHv2Light_empty_sequence")

        best_sequence, _, _, best_score = max(beam, key=lambda item: item[3])
        if not best_sequence:
            return None, self._make_record(error_message="RHv2Light_empty_sequence")
        first = best_sequence[0]
        record = self._make_record(
            selected_task_id=first["task"].task_id,
            score=best_score,
            estimated_distance=float(first["path_length"]),
            planned_path_length=float(first["path_length"]),
            priority=first["task"].priority,
            risk=first["task"].risk,
            abnormal_weight=first["task"].abnormal_weight,
        )
        record["rh_sequence"] = [r["task"].task_id for r in best_sequence]
        record["rh_sequence_score"] = best_score
        record["base_score"] = first["base_score"]
        record["path"] = first["path"]
        record["reachable"] = True
        record["turn_count"] = first["turn_count"]
        record["obstacle_nearby_count"] = first["obstacle_nearby_count"]
        return first["task"], record

    def select_next_task(self, current_grid_pos, current_time, completed_sequence):
        try:
            if self.method_name == "AStarOnly":
                return self._select_astar_only(current_grid_pos)
            if self.method_name == "TSP-2opt":
                return self._select_tsp_2opt(current_grid_pos)
            if self.method_name == "Priority-Greedy":
                return self._select_priority_greedy(current_grid_pos)
            if self.method_name == "Proposed-Balanced":
                return self._select_proposed(current_grid_pos)
            if self.method_name == "RH-v2-Light":
                return self._select_rh_light(current_grid_pos)
            return None, self._make_record(error_message=f"unsupported_method:{self.method_name}")
        except Exception as exc:
            return None, self._make_record(error_message=f"exception:{type(exc).__name__}:{exc}")


class VehicleTaskExecutionSimulator:
    def __init__(
        self,
        grid_map,
        tasks,
        allocator_name,
        start_pos,
        start_theta=0.0,
        robot_speed=0.6,
        inspection_time=5.0,
        high_priority_threshold=0.7,
        abnormal_trigger_after_tasks=None,
        abnormal_task_num=0,
        seed=0,
    ):
        self.grid_map = grid_map
        self.tasks = tasks
        self.allocator_name = allocator_name
        self.start_pos = start_pos
        self.start_theta = start_theta
        self.robot_speed = robot_speed
        self.inspection_time = inspection_time
        self.high_priority_threshold = high_priority_threshold
        self.abnormal_trigger_after_tasks = abnormal_trigger_after_tasks
        self.abnormal_task_num = abnormal_task_num
        self.seed = seed
        self.rng = random.Random(seed + 5000)
        self.planner = AStarPlanner(grid_map)

    def run(self):
        tasks_copy = copy.deepcopy(self.tasks)
        adapter = VehicleMethodAdapter(
            method_name=self.allocator_name,
            grid_map=self.grid_map,
            tasks=tasks_copy,
            start_pos=self.start_pos,
            robot_speed=self.robot_speed,
            inspection_time=self.inspection_time,
            high_priority_threshold=self.high_priority_threshold,
            seed=self.seed,
        )
        vehicle = DifferentialDriveVehicle(self.start_pos[0], self.start_pos[1], self.start_theta, v_max=self.robot_speed, omega_max=1.0, dt=0.1, radius=0.2)
        follower = PurePursuitLikeFollower(vehicle)

        current_time = 0.0
        current_grid_pos = self.start_pos
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
        abnormal_task_ids = []
        abnormal_trigger_time = 0.0

        while True:
            pending = [t for t in tasks_copy if t.status == 0]
            if not pending:
                break

            selected_task, record = adapter.select_next_task(current_grid_pos, current_time, task_sequence)
            if selected_task is None:
                adapter_failed_num += 1
                failed_task_num += 1
                execution_records.append({**record, "order": len(task_sequence) + 1, "selected_task_id": None, "planned_path_length": 0.0, "vehicle_trajectory_length": 0.0, "vehicle_execution_time": 0.0, "success": False, "finish_time": current_time})
                break

            if not isinstance(selected_task, InspectionTask):
                adapter_failed_num += 1
                failed_task_num += 1
                execution_records.append({**record, "order": len(task_sequence) + 1, "selected_task_id": None, "planned_path_length": 0.0, "vehicle_trajectory_length": 0.0, "vehicle_execution_time": 0.0, "success": False, "finish_time": current_time, "error_message": "selected task is not object"})
                break

            if selected_task.status == 1:
                adapter_failed_num += 1
                failed_task_num += 1
                execution_records.append({**record, "order": len(task_sequence) + 1, "selected_task_id": selected_task.task_id, "planned_path_length": 0.0, "vehicle_trajectory_length": 0.0, "vehicle_execution_time": 0.0, "success": False, "finish_time": current_time, "error_message": "selected task already completed"})
                break

            plan = self.planner.plan(current_grid_pos, selected_task.position)
            if not plan["reachable"]:
                unreachable_task_num += 1
                failed_task_num += 1
                selected_task.status = 2
                execution_records.append({**record, "order": len(task_sequence) + 1, "selected_task_id": selected_task.task_id, "planned_path_length": 0.0, "vehicle_trajectory_length": 0.0, "vehicle_execution_time": 0.0, "success": False, "finish_time": current_time, "error_message": "A* unreachable"})
                continue

            path = [grid_to_vehicle_point(p) for p in plan["path"]]
            follower.vehicle.reset(*grid_to_vehicle_point(current_grid_pos), theta=vehicle.theta)
            follow_result = follower.follow_path(path)

            total_planned_path_length += plan["path_length"]
            vehicle_trajectory_length += follow_result["trajectory_length"]
            vehicle_execution_time += follow_result["execution_time"]
            heading_change_sum += follow_result["heading_change_sum"]
            trajectory.extend(follow_result["trajectory"])

            ratio = follow_result["trajectory_to_plan_ratio"]
            record.update({
                "planned_path_length": plan["path_length"],
                "vehicle_trajectory_length": follow_result["trajectory_length"],
                "vehicle_execution_time": follow_result["execution_time"],
                "trajectory_to_plan_ratio": ratio,
                "final_vehicle_position": follow_result["final_position"],
                "success": follow_result["success"],
                "heading_change_sum": follow_result["heading_change_sum"],
                "error_message": follow_result["error_message"] or record.get("error_message", ""),
                "warning": follow_result.get("warning", ""),
            })

            if not follow_result["success"]:
                follower_failed_num += 1
                failed_task_num += 1
                selected_task.status = 2
                execution_records.append({**record, "order": len(task_sequence) + 1, "selected_task_id": selected_task.task_id, "finish_time": current_time})
                continue

            current_time += follow_result["execution_time"] + self.inspection_time
            selected_task.status = 1
            current_grid_pos = selected_task.position
            vehicle.reset(*follow_result["final_position"], theta=vehicle.theta)
            task_sequence.append(selected_task.task_id)
            task_finish_times[selected_task.task_id] = current_time
            goal_success_count += 1

            execution_records.append({**record, "order": len(task_sequence), "selected_task_id": selected_task.task_id, "finish_time": current_time})

            if self.abnormal_trigger_after_tasks is not None and len(task_sequence) == self.abnormal_trigger_after_tasks and not abnormal_task_ids:
                remaining = [t for t in tasks_copy if t.status == 0]
                sample_n = min(self.abnormal_task_num, len(remaining))
                abnormal_tasks = self.rng.sample(remaining, sample_n) if sample_n > 0 else []
                abnormal_task_ids = [t.task_id for t in abnormal_tasks]
                abnormal_trigger_time = current_time
                for task in abnormal_tasks:
                    task.abnormal_weight = 1.0

        high_priority_times = [task_finish_times[t.task_id] for t in tasks_copy if t.priority >= self.high_priority_threshold and t.task_id in task_finish_times]
        high_priority_avg_response_time = sum(high_priority_times) / len(high_priority_times) if high_priority_times else 0.0
        if abnormal_task_ids:
            resp = []
            for tid in abnormal_task_ids:
                if tid in task_finish_times:
                    resp.append(task_finish_times[tid] - abnormal_trigger_time)
                else:
                    resp.append(max(0.0, current_time - abnormal_trigger_time))
            abnormal_avg_response_time = sum(resp) / len(resp) if resp else 0.0
        else:
            abnormal_avg_response_time = 0.0

        goal_success_rate = goal_success_count / len(task_sequence) * 100.0 if task_sequence else 0.0
        trajectory_to_plan_ratio = vehicle_trajectory_length / total_planned_path_length if total_planned_path_length > 0 else 0.0

        return {
            "method": self.allocator_name,
            "completed_task_num": len(task_sequence),
            "total_planned_path_length": total_planned_path_length,
            "vehicle_trajectory_length": vehicle_trajectory_length,
            "vehicle_execution_time": vehicle_execution_time,
            "total_inspection_time": current_time,
            "high_priority_avg_response_time": high_priority_avg_response_time,
            "abnormal_avg_response_time": abnormal_avg_response_time,
            "abnormal_priority_rate": 0.0,
            "heading_change_sum": heading_change_sum,
            "goal_success_rate": goal_success_rate,
            "trajectory_to_plan_ratio": trajectory_to_plan_ratio,
            "failed_task_num": failed_task_num,
            "unreachable_task_num": unreachable_task_num,
            "follower_failed_num": follower_failed_num,
            "adapter_failed_num": adapter_failed_num,
            "task_sequence": task_sequence,
            "execution_records": execution_records,
            "trajectory": trajectory,
        }
