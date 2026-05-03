import math


def _clip(value, low=0.0, high=1.0):
    return min(high, max(low, value))


def compute_task_urgency(task):
    return _clip(
        0.50 * task.priority
        + 0.30 * task.risk
        + 0.20 * task.abnormal_weight,
        0.0,
        1.0,
    )


def estimate_reference_time(tasks, planner, start_pos, robot_speed, inspection_time):
    lengths = []
    for task in tasks:
        if task.status == 1:
            continue
        path_info = planner.plan(start_pos, task.position)
        if path_info.get("reachable", False):
            lengths.append(path_info["path_length"])
    if not lengths:
        return inspection_time
    mean_path_length = sum(lengths) / len(lengths)
    return mean_path_length / robot_speed + inspection_time


def compute_deadline(task, reference_time):
    urgency = compute_task_urgency(task)
    deadline = reference_time * (2.0 + 4.0 * (1.0 - urgency))
    deadline_min = 1.5 * reference_time
    deadline_max = 6.0 * reference_time
    return min(max(deadline, deadline_min), deadline_max)


def deadline_violation_penalty(task, predicted_finish_time, reference_time):
    deadline = compute_deadline(task, reference_time)
    violation = max(0.0, predicted_finish_time - deadline)
    return compute_task_urgency(task) * violation / (reference_time + 1e-6)


def slack_score(task, predicted_arrival_time, reference_time):
    return compute_deadline(task, reference_time) - predicted_arrival_time
