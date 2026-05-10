import math

from astar_planner import AStarPlanner


class RHProposedAllocatorV2:
    def __init__(
        self,
        grid_map,
        tasks,
        start_pos,
        robot_speed=0.6,
        inspection_time=5.0,
        alpha=0.22,
        beta=0.18,
        lambda_abnormal=0.15,
        gamma=0.27,
        delta=0.12,
        eta=0.06,
        mu=0.5,
        epsilon=1e-6,
        horizon=4,
        beam_width=8,
        candidate_pool_size=10,
        discount=0.90,
        sequence_path_weight=0.35,
        sequence_time_weight=0.20,
        priority_completion_weight=0.25,
        abnormal_completion_weight=0.15,
        topk_priority_bonus=0.08,
    ):
        self.grid_map = grid_map
        self.tasks = tasks
        self.start_pos = start_pos
        self.current_pos = start_pos
        self.robot_speed = robot_speed
        self.inspection_time = inspection_time
        self.alpha = alpha
        self.beta = beta
        self.lambda_abnormal = lambda_abnormal
        self.gamma = gamma
        self.delta = delta
        self.eta = eta
        self.mu = mu
        self.epsilon = epsilon
        self.horizon = horizon
        self.beam_width = beam_width
        self.candidate_pool_size = candidate_pool_size
        self.discount = discount
        self.sequence_path_weight = sequence_path_weight
        self.sequence_time_weight = sequence_time_weight
        self.priority_completion_weight = priority_completion_weight
        self.abnormal_completion_weight = abnormal_completion_weight
        self.topk_priority_bonus = topk_priority_bonus
        self.planner = AStarPlanner(grid_map)
        self.grid_height = len(grid_map)
        self.grid_width = len(grid_map[0]) if self.grid_height > 0 else 0

        self.task_sequence = []
        self.selection_records = []
        self.total_path_length = 0.0
        self.total_inspection_time = 0.0
        self.task_finish_times = {}

    def get_unfinished_tasks(self):
        return [task for task in self.tasks if task.status == 0]

    def compute_complexity_cost(self, path_info):
        if not path_info.get("reachable", False):
            return math.inf
        path_length = path_info.get("path_length", 0.0)
        turn_count = path_info.get("turn_count", 0)
        obstacle_nearby_count = path_info.get("obstacle_nearby_count", 0)
        return (obstacle_nearby_count + self.mu * turn_count) / (path_length + self.epsilon)

    def compute_energy_cost(self, distance_cost, complexity_cost):
        terrain_factor = 0.0
        return 0.6 * distance_cost + 0.3 * complexity_cost + 0.1 * terrain_factor

    def evaluate_single_step_candidates(self, current_pos, candidate_tasks):
        reachable_records = []
        for task in candidate_tasks:
            path_info = self.planner.plan(current_pos, task.position)
            if path_info.get("reachable", False):
                reachable_records.append((task, path_info))

        if not reachable_records:
            return []

        path_lengths = [path_info["path_length"] for _, path_info in reachable_records]
        l_min = min(path_lengths)
        l_max = max(path_lengths)
        denominator = l_max - l_min + self.epsilon

        candidate_records = []
        for task, path_info in reachable_records:
            path_length = path_info["path_length"]
            distance_cost = (path_length - l_min) / denominator
            complexity_cost = self.compute_complexity_cost(path_info)
            energy_cost = self.compute_energy_cost(distance_cost, complexity_cost)
            base_score = (
                self.alpha * task.priority
                + self.beta * task.risk
                + self.lambda_abnormal * task.abnormal_weight
                - self.gamma * distance_cost
                - self.delta * complexity_cost
                - self.eta * energy_cost
            )
            candidate_records.append(
                {
                    "task": task,
                    "path_info": path_info,
                    "path_length": path_length,
                    "distance_cost": distance_cost,
                    "complexity_cost": complexity_cost,
                    "energy_cost": energy_cost,
                    "base_score": base_score,
                }
            )

        return candidate_records

    def _normalize_value(self, value, value_min, value_max):
        if value_max - value_min <= self.epsilon:
            return 0.0
        return (value - value_min) / (value_max - value_min)

    def build_hybrid_candidate_pool(self, candidates):
        if not candidates:
            return []

        top_k = max(3, self.candidate_pool_size // 3)
        selected = {}

        for record in sorted(
            candidates, key=lambda item: (-item["base_score"], item["task"].task_id)
        )[:top_k]:
            selected[record["task"].task_id] = record
        for record in sorted(
            candidates, key=lambda item: (item["path_length"], item["task"].task_id)
        )[:top_k]:
            selected[record["task"].task_id] = record
        for record in sorted(
            candidates, key=lambda item: (-item["task"].priority, item["task"].task_id)
        )[:top_k]:
            selected[record["task"].task_id] = record
        for record in sorted(
            candidates, key=lambda item: (-item["task"].abnormal_weight, item["task"].task_id)
        )[:top_k]:
            selected[record["task"].task_id] = record

        base_scores = [record["base_score"] for record in candidates]
        base_min = min(base_scores)
        base_max = max(base_scores)
        path_lengths = [record["path_length"] for record in candidates]
        path_min = min(path_lengths)
        path_max = max(path_lengths)

        for record in candidates:
            normalized_base_score = self._normalize_value(record["base_score"], base_min, base_max)
            normalized_path_length = self._normalize_value(record["path_length"], path_min, path_max)
            record["hybrid_screen_score"] = (
                0.45 * normalized_base_score
                - 0.30 * normalized_path_length
                + 0.15 * record["task"].priority
                + 0.10 * record["task"].abnormal_weight
            )

        hybrid_sorted = sorted(
            candidates,
            key=lambda item: (-item["hybrid_screen_score"], item["task"].task_id),
        )

        if len(selected) < self.candidate_pool_size:
            for record in hybrid_sorted:
                task_id = record["task"].task_id
                if task_id not in selected:
                    selected[task_id] = record
                if len(selected) >= self.candidate_pool_size:
                    break

        selected_records = list(selected.values())
        if len(selected_records) > self.candidate_pool_size:
            selected_records = sorted(
                selected_records,
                key=lambda item: (-item["hybrid_screen_score"], item["task"].task_id),
            )[: self.candidate_pool_size]
        else:
            selected_records = sorted(
                selected_records,
                key=lambda item: (-item["hybrid_screen_score"], item["task"].task_id),
            )

        return selected_records

    def sequence_score_v2(self, sequence_records):
        if not sequence_records:
            return 0.0

        cumulative_path = 0.0
        cumulative_time = 0.0
        benefit = 0.0
        priority_completion_sum = 0.0
        abnormal_completion_sum = 0.0

        path_scale = max(1.0, self.horizon * (self.grid_width + self.grid_height) / 2.0)
        time_scale = max(
            1.0,
            self.horizon
            * (((self.grid_width + self.grid_height) / 2.0) / self.robot_speed + self.inspection_time),
        )

        for k, record in enumerate(sequence_records):
            task = record["task"]
            path_length_k = record["path_length"]
            travel_time_k = path_length_k / self.robot_speed
            step_time_k = travel_time_k + self.inspection_time
            cumulative_path += path_length_k
            cumulative_time += step_time_k
            benefit += (self.discount ** k) * (
                self.alpha * task.priority
                + self.beta * task.risk
                + self.lambda_abnormal * task.abnormal_weight
            )

            finish_time_i_norm = cumulative_time / time_scale
            priority_completion_sum += task.priority * finish_time_i_norm
            abnormal_completion_sum += task.abnormal_weight * finish_time_i_norm

        path_norm = cumulative_path / path_scale
        time_norm = cumulative_time / time_scale
        path_penalty = self.sequence_path_weight * path_norm
        time_penalty = self.sequence_time_weight * time_norm
        priority_completion_penalty = self.priority_completion_weight * priority_completion_sum
        abnormal_completion_penalty = self.abnormal_completion_weight * abnormal_completion_sum

        top_k = min(3, len(sequence_records))
        if top_k > 0:
            top_k_high_priority = sum(
                1
                for record in sequence_records[:top_k]
                if record["task"].priority >= 0.75
            )
            topk_priority_bonus_value = self.topk_priority_bonus * (top_k_high_priority / top_k)
        else:
            topk_priority_bonus_value = 0.0

        return (
            benefit
            + topk_priority_bonus_value
            - path_penalty
            - time_penalty
            - priority_completion_penalty
            - abnormal_completion_penalty
        )

    def _sequence_ids(self, sequence_records):
        return "->".join(record["task"].task_id for record in sequence_records)

    def beam_search_sequence_v2(self, current_pos, unfinished_tasks):
        beam = [
            {
                "pos": current_pos,
                "remaining_tasks": list(unfinished_tasks),
                "sequence_records": [],
                "score": 0.0,
            }
        ]

        for _ in range(self.horizon):
            new_beam = []
            for state in beam:
                if not state["remaining_tasks"]:
                    new_beam.append(state)
                    continue

                candidates = self.evaluate_single_step_candidates(
                    state["pos"], state["remaining_tasks"]
                )
                if not candidates:
                    new_beam.append(state)
                    continue

                candidate_pool = self.build_hybrid_candidate_pool(candidates)
                for candidate in candidate_pool:
                    task_id = candidate["task"].task_id
                    new_sequence_records = state["sequence_records"] + [candidate]
                    new_remaining_tasks = [
                        task for task in state["remaining_tasks"] if task.task_id != task_id
                    ]
                    new_score = self.sequence_score_v2(new_sequence_records)
                    new_beam.append(
                        {
                            "pos": candidate["task"].position,
                            "remaining_tasks": new_remaining_tasks,
                            "sequence_records": new_sequence_records,
                            "score": new_score,
                        }
                    )

            if not new_beam:
                break

            beam = sorted(
                new_beam,
                key=lambda state: (-state["score"], self._sequence_ids(state["sequence_records"])),
            )[: self.beam_width]

        if not beam:
            return []

        best_state = sorted(
            beam,
            key=lambda state: (-state["score"], self._sequence_ids(state["sequence_records"])),
        )[0]
        return best_state["sequence_records"]

    def select_next_task(self, current_pos):
        unfinished_tasks = self.get_unfinished_tasks()
        best_sequence_records = self.beam_search_sequence_v2(current_pos, unfinished_tasks)
        if not best_sequence_records:
            return None, None, None

        first_record = best_sequence_records[0]
        selected_task = first_record["task"]
        selected_path_info = first_record["path_info"]
        selection_record = {
            "task_id": selected_task.task_id,
            "rh_v2_sequence": self._sequence_ids(best_sequence_records),
            "rh_v2_sequence_score": self.sequence_score_v2(best_sequence_records),
            "base_score": first_record["base_score"],
            "path_length": first_record["path_length"],
            "distance_cost": first_record["distance_cost"],
            "complexity_cost": first_record["complexity_cost"],
            "energy_cost": first_record["energy_cost"],
            "priority": selected_task.priority,
            "risk": selected_task.risk,
            "abnormal_weight": selected_task.abnormal_weight,
            "turn_count": selected_path_info["turn_count"],
            "obstacle_nearby_count": selected_path_info["obstacle_nearby_count"],
        }
        return selected_task, selected_path_info, selection_record

    def compute_high_priority_avg_response_time(self):
        finish_times = [
            self.task_finish_times[task.task_id]
            for task in self.tasks
            if task.priority >= 0.75 and task.task_id in self.task_finish_times
        ]
        if not finish_times:
            return 0.0
        return sum(finish_times) / len(finish_times)

    def compute_priority_weighted_completion_time(self):
        weighted_sum = 0.0
        priority_sum = 0.0
        for task in self.tasks:
            if task.task_id not in self.task_finish_times:
                continue
            weighted_sum += task.priority * self.task_finish_times[task.task_id]
            priority_sum += task.priority
        if priority_sum == 0:
            return 0.0
        return weighted_sum / priority_sum

    def compute_high_priority_top5_rate(self):
        if not self.task_sequence:
            return 0.0
        task_lookup = {task.task_id: task for task in self.tasks}
        top5 = self.task_sequence[:5]
        high_priority_count = sum(
            1 for task_id in top5 if task_lookup[task_id].priority >= 0.75
        )
        return high_priority_count / min(5, len(top5)) * 100.0

    def run(self):
        while True:
            unfinished_tasks = self.get_unfinished_tasks()
            if not unfinished_tasks:
                break

            selected_task, path_info, record = self.select_next_task(self.current_pos)
            if selected_task is None:
                print("No reachable unfinished tasks. Stop RH-Proposed-v2.")
                break

            path_length = path_info["path_length"]
            travel_time = path_length / self.robot_speed
            finish_time = self.total_inspection_time + travel_time + self.inspection_time

            self.total_path_length += path_length
            self.total_inspection_time = finish_time
            selected_task.mark_completed()
            self.current_pos = selected_task.position
            self.task_sequence.append(selected_task.task_id)
            self.task_finish_times[selected_task.task_id] = finish_time

            record["finish_time"] = finish_time
            record["travel_time"] = travel_time
            self.selection_records.append(record)

        return {
            "task_sequence": self.task_sequence,
            "total_path_length": self.total_path_length,
            "total_inspection_time": self.total_inspection_time,
            "high_priority_avg_response_time": self.compute_high_priority_avg_response_time(),
            "priority_weighted_completion_time": self.compute_priority_weighted_completion_time(),
            "high_priority_top5_rate": self.compute_high_priority_top5_rate(),
            "completed_task_num": len(self.task_sequence),
            "selection_records": self.selection_records,
        }
