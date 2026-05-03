import csv
import math
from pathlib import Path


try:
    from scipy import stats
except Exception:  # pragma: no cover - scipy may be unavailable.
    stats = None


COMPARISONS = [
    ("main", "drf_rh_main_results.csv", "DRF-RH-Full", "RH-Proposed-v2"),
    ("main", "drf_rh_main_results.csv", "DRF-RH-Full", "Priority-Greedy"),
    ("main", "drf_rh_main_results.csv", "DRF-RH-Full", "Deadline-Greedy"),
    ("main", "drf_rh_main_results.csv", "DRF-RH-Full", "TSP-2opt"),
    ("main", "drf_rh_main_results.csv", "DRF-RH-Light", "RH-Proposed-v2"),
    ("abnormal", "drf_rh_abnormal_results.csv", "DRF-RH-Full", "RH-v2-Full"),
    ("abnormal", "drf_rh_abnormal_results.csv", "DRF-RH-Full", "Proposed-Balanced"),
    ("abnormal", "drf_rh_abnormal_results.csv", "DRF-RH-Light", "RH-v2-Light"),
    ("ablation", "drf_rh_ablation_results.csv", "DRF-RH-Full", "DRF-RH-Horizon1"),
    ("ablation", "drf_rh_ablation_results.csv", "DRF-RH-Full", "DRF-RH-BaseScoreOnly"),
    ("ablation", "drf_rh_ablation_results.csv", "DRF-RH-Full", "DRF-RH-NoDeadlinePenalty"),
    ("ablation", "drf_rh_ablation_results.csv", "DRF-RH-Full", "DRF-RH-NoTopKBonus"),
    ("ablation", "drf_rh_ablation_results.csv", "DRF-RH-Full", "DRF-RH-NoPathTimePenalty"),
    ("ablation", "drf_rh_ablation_results.csv", "DRF-RH-Full", "DRF-RH-NoHybridPool"),
    ("structured", "drf_rh_structured_results.csv", "DRF-RH-Full", "RH-Proposed-v2"),
    ("structured", "drf_rh_structured_results.csv", "DRF-RH-Full", "TSP-2opt"),
]

METRICS = [
    "total_path_length",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "high_priority_top5_rate",
    "abnormal_avg_response_time",
    "abnormal_priority_rate",
]


def read_rows(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def pair_key(row):
    if "map_type" in row and row.get("map_type"):
        return (row["map_type"], row["seed"])
    return (row["seed"],)


def paired_values(rows, method_a, method_b, metric):
    if not rows or metric not in rows[0]:
        return [], []
    by_key = {}
    for row in rows:
        if row["method"] not in {method_a, method_b}:
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
    m = mean(values)
    return math.sqrt(sum((value - m) ** 2 for value in values) / (len(values) - 1))


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
    if stats is not None and n > 1:
        test = stats.ttest_rel(a_values, b_values)
        t_stat = float(test.statistic)
        p_value = float(test.pvalue)
        conclusion = "significant" if p_value < 0.05 else "not_significant"
    else:
        t_stat = mean_diff / (std_diff / math.sqrt(n)) if n > 1 and std_diff > 0 else 0.0
        p_value = ""
        conclusion = "t_stat_only"
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
    rows_by_file = {}
    out_rows = []
    for experiment, filename, method_a, method_b in COMPARISONS:
        path = results_dir / filename
        rows_by_file.setdefault(filename, read_rows(path))
        rows = rows_by_file[filename]
        for metric in METRICS:
            out_rows.append(analyze_pair(experiment, rows, method_a, method_b, metric))

    output_path = results_dir / "drf_rh_significance.csv"
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
        writer.writerows(out_rows)

    print(f"Saved: {output_path}")
    print(f"Rows: {len(out_rows)}")
    print(f"scipy_available: {stats is not None}")


if __name__ == "__main__":
    main()
