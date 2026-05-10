import csv
import math
from pathlib import Path


try:
    from scipy import stats
except Exception:  # pragma: no cover - scipy may be unavailable.
    stats = None


COMPARISONS = [
    ("main", "adaptive_rh_pads_main_results.csv", "A-RH-PADS", "RH-PADS"),
    ("main", "adaptive_rh_pads_main_results.csv", "A-RH-PADS-L", "RH-PADS-L"),
    ("main", "adaptive_rh_pads_main_results.csv", "A-RH-PADS", "AStarOnly"),
    ("main", "adaptive_rh_pads_main_results.csv", "A-RH-PADS", "TSP-2opt"),
    ("main", "adaptive_rh_pads_main_results.csv", "A-RH-PADS", "Priority-Greedy"),
    ("abnormal", "adaptive_rh_pads_abnormal_results.csv", "A-RH-PADS", "RH-PADS"),
    ("abnormal", "adaptive_rh_pads_abnormal_results.csv", "A-RH-PADS-L", "RH-PADS-L"),
    ("ablation", "adaptive_rh_pads_ablation_results.csv", "A-RH-PADS-Full", "A-RH-PADS-FixedLambda"),
    ("ablation", "adaptive_rh_pads_ablation_results.csv", "A-RH-PADS-Full", "A-RH-PADS-NoUrgencyPressure"),
    ("ablation", "adaptive_rh_pads_ablation_results.csv", "A-RH-PADS-Full", "A-RH-PADS-NoAbnormalPressure"),
    ("ablation", "adaptive_rh_pads_ablation_results.csv", "A-RH-PADS-Full", "A-RH-PADS-NoPathPressure"),
    ("ablation", "adaptive_rh_pads_ablation_results.csv", "A-RH-PADS-Full", "A-RH-PADS-ResponseOnly"),
    ("ablation", "adaptive_rh_pads_ablation_results.csv", "A-RH-PADS-Full", "A-RH-PADS-CostOnly"),
    ("ablation", "adaptive_rh_pads_ablation_results.csv", "A-RH-PADS-Full", "A-RH-PADS-NoFinishTimeResponse"),
    ("vehicle", "adaptive_vehicle_sim_results.csv", "A-RH-PADS-L", "RH-PADS-L"),
    ("vehicle", "adaptive_vehicle_sim_results.csv", "A-RH-PADS-L", "TSP-2opt"),
    ("vehicle", "adaptive_vehicle_sim_results.csv", "A-RH-PADS-L", "Priority-Greedy"),
]

METRICS = [
    "total_path_length",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "abnormal_avg_response_time",
    "abnormal_priority_rate",
    "vehicle_trajectory_length",
    "vehicle_execution_time",
]


def read_rows(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def pair_key(row):
    return (row.get("seed", ""),)


def paired_values(rows, method_a, method_b, metric):
    if not rows or metric not in rows[0]:
        return [], []
    by_key = {}
    for row in rows:
        if row.get("method") not in {method_a, method_b}:
            continue
        by_key.setdefault(pair_key(row), {})[row["method"]] = row
    a_values = []
    b_values = []
    for item in by_key.values():
        if method_a in item and method_b in item:
            a_values.append(float(item[method_a][metric]))
            b_values.append(float(item[method_b][metric]))
    return a_values, b_values


def mean(values):
    return sum(values) / len(values) if values else 0.0


def stdev(values):
    if len(values) < 2:
        return 0.0
    value_mean = mean(values)
    return math.sqrt(sum((value - value_mean) ** 2 for value in values) / (len(values) - 1))


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def analyze_pair(experiment, rows, method_a, method_b, metric):
    a_values, b_values = paired_values(rows, method_a, method_b, metric)
    n = len(a_values)
    if n == 0:
        return {
            "experiment": experiment,
            "comparison": f"{method_a} vs {method_b}",
            "metric": metric,
            "method_a": method_a,
            "method_b": method_b,
            "n": 0,
            "mean_a": "",
            "mean_b": "",
            "mean_diff": "",
            "std_diff": "",
            "t_stat": "",
            "p_value": "",
            "conclusion": "skipped",
        }

    diffs = [a - b for a, b in zip(a_values, b_values)]
    mean_diff = mean(diffs)
    std_diff = stdev(diffs)
    t_stat = 0.0
    p_value = ""
    conclusion = "t_stat_only"
    if stats is not None and n > 1:
        test = stats.ttest_rel(a_values, b_values)
        if finite(float(test.statistic)) and finite(float(test.pvalue)):
            t_stat = float(test.statistic)
            p_value = float(test.pvalue)
            conclusion = "significant" if p_value < 0.05 else "not_significant"
        elif std_diff > 0:
            t_stat = mean_diff / (std_diff / math.sqrt(n))
    elif n > 1 and std_diff > 0:
        t_stat = mean_diff / (std_diff / math.sqrt(n))

    return {
        "experiment": experiment,
        "comparison": f"{method_a} vs {method_b}",
        "metric": metric,
        "method_a": method_a,
        "method_b": method_b,
        "n": n,
        "mean_a": mean(a_values),
        "mean_b": mean(b_values),
        "mean_diff": mean_diff,
        "std_diff": std_diff,
        "t_stat": t_stat,
        "p_value": p_value,
        "conclusion": conclusion,
    }


def main():
    base_dir = Path(__file__).resolve().parent
    results_dir = base_dir / "results"
    output_path = results_dir / "adaptive_rh_pads_significance.csv"

    rows_by_file = {}
    output_rows = []
    for experiment, filename, method_a, method_b in COMPARISONS:
        rows_by_file.setdefault(filename, read_rows(results_dir / filename))
        for metric in METRICS:
            output_rows.append(
                analyze_pair(
                    experiment,
                    rows_by_file[filename],
                    method_a,
                    method_b,
                    metric,
                )
            )

    fieldnames = [
        "experiment",
        "comparison",
        "metric",
        "method_a",
        "method_b",
        "n",
        "mean_a",
        "mean_b",
        "mean_diff",
        "std_diff",
        "t_stat",
        "p_value",
        "conclusion",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(output_rows)

    print(f"Saved: {output_path}")
    print(f"Rows: {len(output_rows)}")
    print(f"scipy_available: {stats is not None}")


if __name__ == "__main__":
    main()
