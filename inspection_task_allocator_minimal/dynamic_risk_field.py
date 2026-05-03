import math
from dataclasses import dataclass


@dataclass
class AbnormalEvent:
    event_id: str
    position: tuple[int, int]
    intensity: float
    trigger_time: float


class DynamicRiskField:
    def __init__(
        self,
        sigma: float = 5.0,
        decay_rate: float = 0.01,
        max_weight: float = 1.0,
        combine_mode: str = "sum_clip",
    ):
        if combine_mode not in {"sum_clip", "max"}:
            raise ValueError("combine_mode must be 'sum_clip' or 'max'.")
        self.sigma = sigma
        self.decay_rate = decay_rate
        self.max_weight = max_weight
        self.combine_mode = combine_mode
        self.events: list[AbnormalEvent] = []

    def add_event(self, event_id, position, current_time, intensity=1.0):
        self.events.append(
            AbnormalEvent(
                event_id=str(event_id),
                position=position,
                intensity=float(intensity),
                trigger_time=float(current_time),
            )
        )

    def compute_weight(self, task_position, current_time):
        if not self.events:
            return 0.0
        x, y = task_position
        contributions = []
        for event in self.events:
            ex, ey = event.position
            distance = abs(x - ex) + abs(y - ey)
            spatial_decay = math.exp(-distance / max(self.sigma, 1e-6))
            elapsed = max(0.0, current_time - event.trigger_time)
            temporal_decay = math.exp(-self.decay_rate * elapsed)
            contributions.append(event.intensity * spatial_decay * temporal_decay)

        if self.combine_mode == "max":
            value = max(contributions)
        else:
            value = sum(contributions)
        return min(self.max_weight, max(0.0, value))

    def update_tasks_abnormal_weight(self, tasks, current_time):
        for task in tasks:
            if task.status == 1:
                continue
            dynamic_weight = self.compute_weight(task.position, current_time)
            task.abnormal_weight = min(
                self.max_weight,
                max(task.abnormal_weight, dynamic_weight),
            )
