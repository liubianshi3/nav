from dataclasses import dataclass


@dataclass
class InspectionTask:
    task_id: str
    x: int
    y: int
    priority: float
    risk: float
    abnormal_weight: float = 0.0
    status: int = 0
