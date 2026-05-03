"""巡检任务数据结构。

字段含义：
- task_id：任务编号，例如 "P1"、"P2"
- x, y：任务点在栅格地图中的坐标，统一使用 (x, y)
- priority：任务优先级，范围 0 到 1，越大越应该优先巡检
- risk：区域风险等级，范围 0 到 1，越大表示风险越高
- abnormal_weight：异常反馈权重，范围 0 到 1，默认 0.0
- status：任务状态，0 表示未完成，1 表示已完成
"""

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

    @property
    def position(self):
        return (self.x, self.y)

    def mark_completed(self):
        self.status = 1

    def is_completed(self):
        return self.status == 1
