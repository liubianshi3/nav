import math
import statistics

from astar_planner import AStarPlanner


class AdaptiveRHPADSAllocator:
    """Adaptive Receding-Horizon Priority-Aware Dynamic Scheduler."""

    def __init__(
        self,
        grid_map,
        tasks,
        start_pos,
        robot_speed=0.6,
        inspection_time=5.0,
        horizon=4,
        beam_width=8,
        candidate_pool_size=10,
        discount=0.92,
        lambda_min=0.25,
        lambda_max=0.85,
        k0=-0.2,
        k_u=2.0,
        k_a=1.5,
        k_d=1.2,
        fixed_lambda=None,
        use_urgency_pressure=True,
        use_abnormal_pressure=True,
        use_path_pressure=True,
        no_finish_time_response=False,
        epsilon=1e-6,
    ):
        self.grid_map = grid_map
        self.tasks = tasks
        self.start_pos = start_pos
        self.current_pos = start_pos
        self.robot_speed = robot_speed
        self.inspection_time = inspection_time
        self.horizon = horizon
        self.beam_width = beam_width
        self.candidate_pool_size = candidate_pool_size
        self.discount = discount
        self.lambda_lower = lambda_min
        self.lambda_upper = lambda_max
        self.k0 = k0
        self.k_u = k_u
        self.k_a = k_a
        self.k_d = k_d
        self.fixed_lambda = fixed_lambda
        self.use_urgency_pressure = use_urgency_pressure
        self.use_abnormal_pressure = use_abnormal_pressure
        self.use_path_pressure = use_path_pressure
        self.no_finish_time_response = no_finish_time_response
        self.epsilon = epsilon

        self.planner = AStarPlanner(grid_map)
        self.path_cache = {}
        self.grid_height = len(grid_map)
        self.grid_width = len(grid_map[0]) if self.grid_height else 0

        self.task_sequence = []
        self.selection_records = []
        self.total_path_length = 0.0
        self.total_inspection_time = 0.0
        self.task_finish_times = {}
        self.lambda_sequence = []

    def get_unfinished_tasks(self):
        return [task for task in self.tasks if task.status == 0]

    def cached_plan(self, start, goal):
        key = (start, goal)
        if key not in self.path_cache:
            self.path_cache[key] = self.planner.plan(start, goal)
        return self.path_cache[key]

    @staticmethod
    def clip(value, lower=0.0, upper=1.0):
        return max(lower, min(upper, value))

    @staticmethod
    def sigmoid(value):
        if value >= 0:
            z = math.exp(-value)
            return 1.0 / (1.0 + z)
        z = math.exp(value)
        return z / (1.0 + z)

    def compute_urgency(self, task):
        return self.clip(
            0.45 * task.priority
            + 0.35 * task.risk
            + 0.20 * task.abnormal_weight,
            0.0,
            1.0,
        )

    def compute_urgency_pressure(self, pending_tasks):
        if not pending_tasks:
            return 0.0
        urgencies = sorted(
            [self.compute_urgency(task) for task in pending_tasks],
            reverse=True,
        )
        max_urgency = urgencies[0]
        top_count = min(5, len(urgencies))
        mean_top_urgency = sum(urgencies[:top_count]) / top_count
        return 0.6 * max_urgency + 0.4 * mean_top_urgency

    def compute_abnormal_pressure(self, pending_tasks):
        if not pending_tasks:
            return 0.0
        return max(task.abnormal_weight for task in pending_tasks)

    def compute_path_pressure(self, current_pos, pending_tasks):
        distances = []
        for task in pending_tasks:
            path_info = self.cached_plan(current_pos, task.position)
            if path_info.get("reachable", False):
                distances.append(float(path_info["path_length"]))
        if not distances:
            return 1.0
        d_min = min(distances)
        d_max = max(distances)
        if d_max - d_min <= self.epsilon:
            return 0.5
        normalized = [
            (distance - d_min) / (d_max - d_min + self.epsilon)
            for distance in distances
        ]
        return sum(normalized) / len(normalized)

    def compute_lambda(self, current_pos, pending_tasks):
        u_t = self.compute_urgency_pressure(pending_tasks)
        a_t = self.compute_abnormal_pressure(pending_tasks)
        d_t = self.compute_path_pressure(current_pos, pending_tasks)
        if self.fixed_lambda is not None:
            lambda_t = self.clip(self.fixed_lambda, self.lambda_lower, self.lambda_upper)
            return lambda_t, u_t, a_t, d_t

        u_component = u_t if self.use_urgency_pressure else 0.0
        a_component = a_t if self.use_abnormal_pressure else 0.0
        d_component = d_t if self.use_path_pressure else 0.0
        raw = (
            self.k0
            + self.k_u * u_component
            + self.k_a * a_component
            - self.k_d * d_component
        )
        lambda_t = self.lambda_lower + (self.lambda_upper - self.lambda_lower) * self.sigmoid(raw)
        return self.clip(lambda_t, self.lambda_lower, self.lambda_upper), u_t, a_t, d_t

    def compute_base_score(self, task, distance_norm, path_info):
        path_length = max(float(path_info.get("path_length", 0.0)), self.epsilon)
        complexity_cost = (
            path_info.get("obstacle_nearby_count", 0)
            + 0.5 * path_info.get("turn_count", 0)
        ) / path_length
        return (
            0.22 * task.priority
            + 0.18 * task.risk
            + 0.15 * task.abnormal_weight
            - 0.27 * distance_norm
            - 0.12 * complexity_cost
        )

    def evaluate_reachable_candidates(self, current_pos, candidate_tasks):
        reachable = []
        for task in candidate_tasks:
            path_info = self.cached_plan(current_pos, task.position)
            if path_info.get("reachable", False):
                reachable.append((task, path_info))

        if not reachable:
            return []

        path_lengths = [float(path_info["path_length"]) for _, path_info in reachable]
        l_min = min(path_lengths)
        l_max = max(path_lengths)
        denom = l_max - l_min + self.epsilon
        records = []
        for task, path_info in reachable:
            path_length = float(path_info["path_length"])
            distance_norm = (path_length - l_min) / denom if l_max > l_min else 0.0
            urgency = self.compute_urgency(task)
            base_score = self.compute_base_score(task, distance_norm, path_info)
            records.append(
                {
                    "task": task,
                    "path_info": path_info,
                    "path_length": path_length,
                    "turn_count": float(path_info.get("turn_count", 0)),
                    "obstacle_nearby_count": float(path_info.get("obstacle_nearby_count", 0)),
                    "distance_norm": distance_norm,
                    "urgency": urgency,
                    "urgency_distance_ratio": urgency / (path_length + 1.0),
                    "base_score": base_score,
                }
            )
        return records

    def build_candidate_pool(self, candidate_records):
        if not candidate_records:
            return []

        top_k = max(2, min(self.candidate_pool_size, math.ceil(self.candidate_pool_size / 3)))
        selected = {}

        sorters = [
            lambda record: (-record["urgency"], record["task"].task_id),
            lambda record: (record["path_length"], record["task"].task_id),
            lambda record: (-record["urgency_distance_ratio"], record["task"].task_id),
            lambda record: (-record["task"].abnormal_weight, record["task"].task_id),
            lambda record: (-record["base_score"], record["task"].task_id),
        ]
        for sorter in sorters:
            for record in sorted(candidate_records, key=sorter)[:top_k]:
                selected[record["task"].task_id] = record

        urgency_values = [record["urgency"] for record in candidate_records]
        path_values = [record["path_length"] for record in candidate_records]
        base_values = [record["base_score"] for record in candidate_records]

        def norm(value, values):
            low = min(values)
            high = max(values)
            if high - low <= self.epsilon:
                return 0.0
            return (value - low) / (high - low)

        for record in candidate_records:
            record["adaptive_screen_score"] = (
                0.40 * norm(record["urgency"], urgency_values)
                - 0.25 * norm(record["path_length"], path_values)
                + 0.20 * norm(record["base_score"], base_values)
                + 0.10 * record["task"].abnormal_weight
                + 0.05 * record["urgency_distance_ratio"]
            )

        for record in sorted(
            candidate_records,
            key=lambda item: (-item["adaptive_screen_score"], item["task"].task_id),
        ):
            if len(selected) >= self.candidate_pool_size:
                break
            selected[record["task"].task_id] = record

        return sorted(
            selected.values(),
            key=lambda item: (-item["adaptive_screen_score"], item["task"].task_id),
        )[: self.candidate_pool_size]

    def compute_scales(self, initial_candidates, pending_count):
        usable_horizon = max(1, min(self.horizon, pending_count))
        if initial_candidates:
            mean_path = sum(record["path_length"] for record in initial_candidates) / len(initial_candidates)
            mean_step_time = sum(
                record["path_length"] / self.robot_speed + self.inspection_time
                for record in initial_candidates
            ) / len(initial_candidates)
            mean_turn = sum(record["turn_count"] for record in initial_candidates) / len(initial_candidates)
        else:
            mean_path = (self.grid_width + self.grid_height) / 2.0
            mean_step_time = mean_path / self.robot_speed + self.inspection_time
            mean_turn = 1.0
        return {
            "path_scale": max(1.0, mean_path * usable_horizon),
            "time_scale": max(1.0, mean_step_time * usable_horizon),
            "heading_scale": max(1.0, mean_turn * usable_horizon),
        }

    def score_sequence(self, sequence_records, lambda_t, current_time, scales):
        if not sequence_records:
            return 0.0, 0.0, 0.0, 0.0

        cumulative_path = 0.0
        cumulative_time = 0.0
        cumulative_heading = 0.0
        response_score = 0.0
        for k, record in enumerate(sequence_records):
            travel_time = record["path_length"] / self.robot_speed
            step_time = travel_time + self.inspection_time
            cumulative_path += record["path_length"]
            cumulative_time += step_time
            cumulative_heading += record["turn_count"]
            if self.no_finish_time_response:
                response_term = record["urgency"]
            else:
                finish_time_norm = (current_time + cumulative_time) / scales["time_scale"]
                response_term = record["urgency"] / (1.0 + finish_time_norm)
            response_score += (self.discount ** k) * response_term

        path_cost_norm = cumulative_path / scales["path_scale"]
        turn_cost_norm = cumulative_heading / scales["heading_scale"]
        execution_time_norm = cumulative_time / scales["time_scale"]
        cost_score = (
            0.70 * path_cost_norm
            + 0.20 * turn_cost_norm
            + 0.10 * execution_time_norm
        )
        sequence_score = lambda_t * response_score - (1.0 - lambda_t) * cost_score
        predicted_finish_time = current_time + cumulative_time
        return sequence_score, response_score, cost_score, predicted_finish_time

    @staticmethod
    def sequence_ids(sequence_records):
        return "->".join(record["task"].task_id for record in sequence_records)

    def beam_search(self, current_pos, pending_tasks, lambda_t, current_time, scales):
        beam = [
            {
                "pos": current_pos,
                "remaining_tasks": list(pending_tasks),
                "sequence_records": [],
                "score": 0.0,
                "response_score": 0.0,
                "cost_score": 0.0,
                "predicted_finish_time": current_time,
            }
        ]

        for _ in range(self.horizon):
            next_beam = []
            for state in beam:
                if not state["remaining_tasks"]:
                    next_beam.append(state)
                    continue
                candidates = self.evaluate_reachable_candidates(
                    state["pos"],
                    state["remaining_tasks"],
                )
                candidate_pool = self.build_candidate_pool(candidates)
                if not candidate_pool:
                    next_beam.append(state)
                    continue
                for candidate in candidate_pool:
                    task_id = candidate["task"].task_id
                    sequence = state["sequence_records"] + [candidate]
                    remaining = [
                        task for task in state["remaining_tasks"]
                        if task.task_id != task_id
                    ]
                    score, response, cost, predicted_finish = self.score_sequence(
                        sequence,
                        lambda_t,
                        current_time,
                        scales,
                    )
                    next_beam.append(
                        {
                            "pos": candidate["task"].position,
                            "remaining_tasks": remaining,
                            "sequence_records": sequence,
                            "score": score,
                            "response_score": response,
                            "cost_score": cost,
                            "predicted_finish_time": predicted_finish,
                        }
                    )
            if not next_beam:
                break
            beam = sorted(
                next_beam,
                key=lambda state: (-state["score"], self.sequence_ids(state["sequence_records"])),
            )[: self.beam_width]

        if not beam:
            return None
        return sorted(
            beam,
            key=lambda state: (-state["score"], self.sequence_ids(state["sequence_records"])),
        )[0]

    def select_next_task(self, current_pos, current_time=0.0):
        pending_tasks = self.get_unfinished_tasks()
        if not pending_tasks:
            return None, None

        lambda_t, u_t, a_t, d_t = self.compute_lambda(current_pos, pending_tasks)
        initial_candidates = self.evaluate_reachable_candidates(current_pos, pending_tasks)
        candidate_pool = self.build_candidate_pool(initial_candidates)
        if not candidate_pool:
            return None, None

        scales = self.compute_scales(initial_candidates, len(pending_tasks))
        best_state = self.beam_search(
            current_pos,
            pending_tasks,
            lambda_t,
            current_time,
            scales,
        )
        if not best_state or not best_state["sequence_records"]:
            return None, None

        sequence = best_state["sequence_records"]
        first = sequence[0]
        selected_task = first["task"]
        first_finish_time = current_time + first["path_length"] / self.robot_speed + self.inspection_time
        record = {
            "selected_task_id": selected_task.task_id,
            "task_id": selected_task.task_id,
            "adaptive_sequence": self.sequence_ids(sequence),
            "adaptive_sequence_score": best_state["score"],
            "lambda_t": lambda_t,
            "U_t": u_t,
            "A_t": a_t,
            "D_t": d_t,
            "response_score": best_state["response_score"],
            "cost_score": best_state["cost_score"],
            "urgency": first["urgency"],
            "planned_path_length": first["path_length"],
            "path_length": first["path_length"],
            "predicted_finish_time": first_finish_time,
            "candidate_pool_size": len(candidate_pool),
            "beam_width": self.beam_width,
            "horizon": self.horizon,
            "priority": selected_task.priority,
            "risk": selected_task.risk,
            "abnormal_weight": selected_task.abnormal_weight,
            "turn_count": first["turn_count"],
            "obstacle_nearby_count": first["obstacle_nearby_count"],
            "base_score": first["base_score"],
            "path_info": first["path_info"],
        }
        return selected_task, record

    def compute_high_priority_avg_response_time(self):
        finish_times = [
            self.task_finish_times[task.task_id]
            for task in self.tasks
            if task.priority >= 0.75 and task.task_id in self.task_finish_times
        ]
        return sum(finish_times) / len(finish_times) if finish_times else 0.0

    def compute_priority_weighted_completion_time(self):
        weighted_sum = 0.0
        priority_sum = 0.0
        for task in self.tasks:
            if task.task_id in self.task_finish_times:
                weighted_sum += task.priority * self.task_finish_times[task.task_id]
                priority_sum += task.priority
        return weighted_sum / priority_sum if priority_sum else 0.0

    def compute_high_priority_top5_rate(self):
        if not self.task_sequence:
            return 0.0
        task_lookup = {task.task_id: task for task in self.tasks}
        top5 = self.task_sequence[:5]
        count = sum(1 for task_id in top5 if task_lookup[task_id].priority >= 0.75)
        return count / min(5, len(top5)) * 100.0

    def execute_task(self, selected_task, record):
        path_info = record["path_info"]
        path_length = float(path_info["path_length"])
        travel_time = path_length / self.robot_speed
        finish_time = self.total_inspection_time + travel_time + self.inspection_time

        self.total_path_length += path_length
        self.total_inspection_time = finish_time
        selected_task.mark_completed()
        self.current_pos = selected_task.position
        self.task_sequence.append(selected_task.task_id)
        self.task_finish_times[selected_task.task_id] = finish_time
        self.lambda_sequence.append(record["lambda_t"])

        stored_record = {key: value for key, value in record.items() if key != "path_info"}
        stored_record["finish_time"] = finish_time
        stored_record["travel_time"] = travel_time
        self.selection_records.append(stored_record)

    def lambda_stats(self):
        if not self.lambda_sequence:
            return 0.0, 0.0, 0.0, 0.0
        if len(self.lambda_sequence) > 1:
            lambda_std = statistics.stdev(self.lambda_sequence)
        else:
            lambda_std = 0.0
        return (
            statistics.mean(self.lambda_sequence),
            lambda_std,
            min(self.lambda_sequence),
            max(self.lambda_sequence),
        )

    def run(self):
        while self.get_unfinished_tasks():
            selected_task, record = self.select_next_task(
                self.current_pos,
                self.total_inspection_time,
            )
            if selected_task is None:
                print("No reachable unfinished tasks. Stop A-RH-PADS.")
                break
            self.execute_task(selected_task, record)

        lambda_mean, lambda_std, lambda_min, lambda_max = self.lambda_stats()
        return {
            "task_sequence": self.task_sequence,
            "completed_task_num": len(self.task_sequence),
            "total_path_length": self.total_path_length,
            "total_inspection_time": self.total_inspection_time,
            "high_priority_avg_response_time": self.compute_high_priority_avg_response_time(),
            "priority_weighted_completion_time": self.compute_priority_weighted_completion_time(),
            "high_priority_top5_rate": self.compute_high_priority_top5_rate(),
            "selection_records": self.selection_records,
            "lambda_mean": lambda_mean,
            "lambda_std": lambda_std,
            "lambda_min": lambda_min,
            "lambda_max": lambda_max,
            "lambda_sequence": list(self.lambda_sequence),
        }
