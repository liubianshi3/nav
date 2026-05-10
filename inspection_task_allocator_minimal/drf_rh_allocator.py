import math

from astar_planner import AStarPlanner
from deadline_model import (
    compute_deadline,
    compute_task_urgency,
    deadline_violation_penalty,
    estimate_reference_time,
    slack_score,
)
from dynamic_risk_field import DynamicRiskField


class DRFRHAllocator:
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
        discount=0.92,
        path_penalty_weight=0.25,
        time_penalty_weight=0.15,
        priority_completion_weight=0.30,
        abnormal_completion_weight=0.25,
        topk_bonus_weight=0.15,
        deadline_penalty_weight=0.35,
        risk_sigma=5.0,
        risk_decay_rate=0.01,
        use_hybrid_pool=True,
        use_deadline_penalty=True,
        use_dynamic_risk_field=True,
        use_priority_completion=True,
        use_abnormal_completion=True,
        use_topk_bonus=True,
        use_path_time_penalty=True,
        use_urgency_pool=True,
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
        self.path_penalty_weight = path_penalty_weight
        self.time_penalty_weight = time_penalty_weight
        self.priority_completion_weight = priority_completion_weight
        self.abnormal_completion_weight = abnormal_completion_weight
        self.topk_bonus_weight = topk_bonus_weight
        self.deadline_penalty_weight = deadline_penalty_weight
        self.use_hybrid_pool = use_hybrid_pool
        self.use_deadline_penalty = use_deadline_penalty
        self.use_dynamic_risk_field = use_dynamic_risk_field
        self.use_priority_completion = use_priority_completion
        self.use_abnormal_completion = use_abnormal_completion
        self.use_topk_bonus = use_topk_bonus
        self.use_path_time_penalty = use_path_time_penalty
        self.use_urgency_pool = use_urgency_pool

        self.planner = AStarPlanner(grid_map)
        self.risk_field = DynamicRiskField(sigma=risk_sigma, decay_rate=risk_decay_rate)
        self.grid_height = len(grid_map)
        self.grid_width = len(grid_map[0]) if self.grid_height else 0
        self.reference_time = estimate_reference_time(
            tasks,
            self.planner,
            start_pos,
            robot_speed,
            inspection_time,
        )
        self.path_cache = {}

        self.task_sequence = []
        self.selection_records = []
        self.total_path_length = 0.0
        self.total_inspection_time = 0.0
        self.task_finish_times = {}

    def get_unfinished_tasks(self):
        return [task for task in self.tasks if task.status == 0]

    def cached_plan(self, start, goal):
        key = (start, goal)
        if key not in self.path_cache:
            self.path_cache[key] = self.planner.plan(start, goal)
        return self.path_cache[key]

    def add_abnormal_event(self, event_id, position, current_time, intensity=1.0):
        self.risk_field.add_event(event_id, position, current_time, intensity)

    def update_dynamic_risk(self, current_time):
        if self.use_dynamic_risk_field:
            self.risk_field.update_tasks_abnormal_weight(self.tasks, current_time)

    def compute_complexity_cost(self, path_info):
        if not path_info.get("reachable", False):
            return math.inf
        path_length = path_info.get("path_length", 0.0)
        return (
            path_info.get("obstacle_nearby_count", 0)
            + self.mu * path_info.get("turn_count", 0)
        ) / (path_length + self.epsilon)

    def compute_energy_cost(self, distance_cost, complexity_cost):
        return 0.6 * distance_cost + 0.3 * complexity_cost

    def evaluate_candidates(self, current_pos, current_time, candidate_tasks=None):
        self.update_dynamic_risk(current_time)
        reachable = []
        tasks = candidate_tasks if candidate_tasks is not None else self.get_unfinished_tasks()
        for task in tasks:
            path_info = self.cached_plan(current_pos, task.position)
            if path_info.get("reachable", False):
                reachable.append((task, path_info))
        if not reachable:
            return []

        lengths = [path_info["path_length"] for _, path_info in reachable]
        l_min = min(lengths)
        l_max = max(lengths)
        denom = l_max - l_min + self.epsilon
        candidates = []
        for task, path_info in reachable:
            path_length = path_info["path_length"]
            distance_cost = (path_length - l_min) / denom
            complexity_cost = self.compute_complexity_cost(path_info)
            energy_cost = self.compute_energy_cost(distance_cost, complexity_cost)
            urgency = compute_task_urgency(task)
            arrival_time = path_length / self.robot_speed
            predicted_finish_time = current_time + arrival_time + self.inspection_time
            deadline = compute_deadline(task, self.reference_time)
            deadline_slack = slack_score(task, arrival_time, self.reference_time)
            base_score = (
                self.alpha * task.priority
                + self.beta * task.risk
                + self.lambda_abnormal * task.abnormal_weight
                - self.gamma * distance_cost
                - self.delta * complexity_cost
                - self.eta * energy_cost
            )
            candidates.append(
                {
                    "task": task,
                    "path_info": path_info,
                    "path_length": path_length,
                    "distance_cost": distance_cost,
                    "complexity_cost": complexity_cost,
                    "energy_cost": energy_cost,
                    "base_score": base_score,
                    "urgency": urgency,
                    "deadline": deadline,
                    "predicted_arrival_time": arrival_time,
                    "predicted_finish_time": predicted_finish_time,
                    "deadline_slack": deadline_slack,
                }
            )
        return candidates

    def _normalize(self, value, low, high):
        if high - low <= self.epsilon:
            return 0.0
        return (value - low) / (high - low)

    def build_mixed_candidate_pool(self, candidates):
        if not candidates:
            return []
        if not self.use_hybrid_pool:
            return sorted(candidates, key=lambda r: (-r["base_score"], r["task"].task_id))[
                : self.candidate_pool_size
            ]

        top_k = max(3, self.candidate_pool_size // 3)
        selected = {}

        sort_specs = [
            lambda r: (-r["base_score"], r["task"].task_id),
            lambda r: (r["path_length"], r["task"].task_id),
            lambda r: (-r["task"].priority, r["task"].task_id),
            lambda r: (-r["task"].abnormal_weight, r["task"].task_id),
            lambda r: (r["deadline_slack"], r["task"].task_id),
        ]
        if self.use_urgency_pool:
            sort_specs.append(
                lambda r: (-(r["urgency"] / (r["path_length"] + 1.0)), r["task"].task_id)
            )

        for sorter in sort_specs:
            for record in sorted(candidates, key=sorter)[:top_k]:
                selected[record["task"].task_id] = record

        base_values = [r["base_score"] for r in candidates]
        path_values = [r["path_length"] for r in candidates]
        slack_values = [r["deadline_slack"] for r in candidates]
        bmin, bmax = min(base_values), max(base_values)
        pmin, pmax = min(path_values), max(path_values)
        smin, smax = min(slack_values), max(slack_values)
        for record in candidates:
            normalized_base = self._normalize(record["base_score"], bmin, bmax)
            normalized_path = self._normalize(record["path_length"], pmin, pmax)
            normalized_slack = self._normalize(record["deadline_slack"], smin, smax)
            record["mixed_pool_score"] = (
                0.35 * normalized_base
                - 0.20 * normalized_path
                + 0.18 * record["task"].priority
                + 0.12 * record["task"].abnormal_weight
                + 0.10 * record["urgency"]
                - 0.05 * normalized_slack
            )

        for record in sorted(candidates, key=lambda r: (-r["mixed_pool_score"], r["task"].task_id)):
            selected.setdefault(record["task"].task_id, record)
            if len(selected) >= self.candidate_pool_size:
                break

        return sorted(
            selected.values(),
            key=lambda r: (-r.get("mixed_pool_score", r["base_score"]), r["task"].task_id),
        )[: self.candidate_pool_size]

    def sequence_score(self, sequence, current_pos=None, current_time=0.0):
        if not sequence:
            return 0.0
        cumulative_path = 0.0
        cumulative_time = 0.0
        gain = 0.0
        priority_completion = 0.0
        abnormal_completion = 0.0
        deadline_violation_sum = 0.0
        topk_high_priority_count = 0

        path_scale = max(1.0, self.horizon * (self.grid_width + self.grid_height) / 2.0)
        time_scale = max(1.0, self.horizon * self.reference_time)

        for k, record in enumerate(sequence):
            task = record["task"]
            path_length = record["path_length"]
            travel_time = path_length / self.robot_speed
            finish_offset = cumulative_time + travel_time + self.inspection_time
            cumulative_path += path_length
            cumulative_time = finish_offset
            urgency = compute_task_urgency(task)

            gain += (self.discount ** k) * (
                self.alpha * task.priority
                + self.beta * task.risk
                + self.lambda_abnormal * task.abnormal_weight
            )
            priority_completion += task.priority * (finish_offset / time_scale)
            abnormal_completion += task.abnormal_weight * (finish_offset / time_scale)
            deadline_violation_sum += deadline_violation_penalty(
                task,
                finish_offset,
                self.reference_time,
            )
            if k < 3 and task.priority >= 0.75:
                topk_high_priority_count += 1

        score = gain
        if self.use_path_time_penalty:
            score -= self.path_penalty_weight * (cumulative_path / path_scale)
            score -= self.time_penalty_weight * (cumulative_time / time_scale)
        if self.use_priority_completion:
            score -= self.priority_completion_weight * priority_completion
        if self.use_abnormal_completion:
            score -= self.abnormal_completion_weight * abnormal_completion
        if self.use_deadline_penalty:
            score -= self.deadline_penalty_weight * deadline_violation_sum
        if self.use_topk_bonus:
            score += self.topk_bonus_weight * topk_high_priority_count / max(1, min(3, len(sequence)))
        return score

    def _sequence_ids(self, sequence):
        return "->".join(record["task"].task_id for record in sequence)

    def beam_search_sequence(self, current_pos, current_time):
        beam = [
            {
                "pos": current_pos,
                "remaining_tasks": self.get_unfinished_tasks(),
                "sequence": [],
                "score": 0.0,
            }
        ]

        for _ in range(self.horizon):
            new_beam = []
            for state in beam:
                if not state["remaining_tasks"]:
                    new_beam.append(state)
                    continue
                candidates = self.evaluate_candidates(
                    state["pos"],
                    current_time,
                    state["remaining_tasks"],
                )
                if not candidates:
                    new_beam.append(state)
                    continue
                for candidate in self.build_mixed_candidate_pool(candidates):
                    task_id = candidate["task"].task_id
                    new_sequence = state["sequence"] + [candidate]
                    new_remaining = [
                        task for task in state["remaining_tasks"] if task.task_id != task_id
                    ]
                    score = self.sequence_score(new_sequence, current_pos, current_time)
                    new_beam.append(
                        {
                            "pos": candidate["task"].position,
                            "remaining_tasks": new_remaining,
                            "sequence": new_sequence,
                            "score": score,
                        }
                    )
            if not new_beam:
                break
            beam = sorted(
                new_beam,
                key=lambda state: (-state["score"], self._sequence_ids(state["sequence"])),
            )[: self.beam_width]

        if not beam:
            return []
        best = sorted(
            beam,
            key=lambda state: (-state["score"], self._sequence_ids(state["sequence"])),
        )[0]
        return best["sequence"]

    def select_next_task(self, current_pos, current_time=0.0):
        self.update_dynamic_risk(current_time)
        best_sequence = self.beam_search_sequence(current_pos, current_time)
        if not best_sequence:
            return None, None, None
        first = best_sequence[0]
        task = first["task"]
        path_info = first["path_info"]
        predicted_finish_offset = first["path_length"] / self.robot_speed + self.inspection_time
        deadline = compute_deadline(task, self.reference_time)
        deadline_violation = max(0.0, predicted_finish_offset - deadline)
        record = {
            "task_id": task.task_id,
            "selected_task_id": task.task_id,
            "drf_rh_sequence": self._sequence_ids(best_sequence),
            "drf_rh_sequence_score": self.sequence_score(best_sequence, current_pos, current_time),
            "base_score": first["base_score"],
            "urgency": first["urgency"],
            "deadline": deadline,
            "predicted_finish_time": current_time + predicted_finish_offset,
            "deadline_violation": deadline_violation,
            "path_length": first["path_length"],
            "distance_cost": first["distance_cost"],
            "complexity_cost": first["complexity_cost"],
            "energy_cost": first["energy_cost"],
            "priority": task.priority,
            "risk": task.risk,
            "abnormal_weight": task.abnormal_weight,
            "turn_count": path_info["turn_count"],
            "obstacle_nearby_count": path_info["obstacle_nearby_count"],
        }
        return task, path_info, record

    def compute_high_priority_avg_response_time(self):
        values = [
            self.task_finish_times[task.task_id]
            for task in self.tasks
            if task.priority >= 0.75 and task.task_id in self.task_finish_times
        ]
        return sum(values) / len(values) if values else 0.0

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

    def execute_task(self, task, path_info, record):
        path_length = path_info["path_length"]
        travel_time = path_length / self.robot_speed
        finish_time = self.total_inspection_time + travel_time + self.inspection_time
        self.total_path_length += path_length
        self.total_inspection_time = finish_time
        task.mark_completed()
        self.current_pos = task.position
        self.task_sequence.append(task.task_id)
        self.task_finish_times[task.task_id] = finish_time
        record["finish_time"] = finish_time
        record["travel_time"] = travel_time
        self.selection_records.append(record)

    def run(self):
        while self.get_unfinished_tasks():
            task, path_info, record = self.select_next_task(
                self.current_pos,
                self.total_inspection_time,
            )
            if task is None:
                print("No reachable unfinished tasks. Stop DRF-RH.")
                break
            self.execute_task(task, path_info, record)
        return {
            "task_sequence": self.task_sequence,
            "completed_task_num": len(self.task_sequence),
            "total_path_length": self.total_path_length,
            "total_inspection_time": self.total_inspection_time,
            "high_priority_avg_response_time": self.compute_high_priority_avg_response_time(),
            "priority_weighted_completion_time": self.compute_priority_weighted_completion_time(),
            "high_priority_top5_rate": self.compute_high_priority_top5_rate(),
            "selection_records": self.selection_records,
        }
