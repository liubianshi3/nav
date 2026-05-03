import csv
from pathlib import Path


def read_rows(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def fmt(value, digits=2):
    if value == "" or value is None:
        return ""
    try:
        return f"{float(value):.{digits}f}"
    except Exception:
        return str(value)


def mean_std(row, metric):
    return f"{fmt(row.get(metric + '_mean', ''))} +/- {fmt(row.get(metric + '_std', ''))}"


def markdown_table(headers, rows):
    lines = []
    lines.append("| " + " | ".join(headers) + " |")
    lines.append("| " + " | ".join(["---"] * len(headers)) + " |")
    for row in rows:
        lines.append("| " + " | ".join(str(item) for item in row) + " |")
    return "\n".join(lines)


def summary_table(title, rows, metrics):
    table_rows = []
    for row in rows:
        table_rows.append([row["method"]] + [mean_std(row, metric) for metric in metrics])
    return f"\n## {title}\n\n" + markdown_table(["Method"] + metrics, table_rows) + "\n"


def significance_table(rows):
    selected = [
        row
        for row in rows
        if row.get("n") not in {"", "0"}
        and row.get("metric")
        in {
            "total_path_length",
            "high_priority_avg_response_time",
            "priority_weighted_completion_time",
            "abnormal_avg_response_time",
            "vehicle_trajectory_length",
            "vehicle_execution_time",
        }
    ]
    table_rows = []
    for row in selected:
        table_rows.append(
            [
                row["experiment"],
                row["comparison"],
                row["metric"],
                row["n"],
                fmt(row["mean_diff"]),
                fmt(row["t_stat"], 3),
                fmt(row["p_value"], 4),
                row["conclusion"],
            ]
        )
    return "\n## Table E: Significance Test Summary\n\n" + markdown_table(
        ["Experiment", "Comparison", "Metric", "n", "Mean diff", "t", "p", "Conclusion"],
        table_rows,
    ) + "\n"


def lambda_table(summary_sets):
    table_rows = []
    for scenario, rows in summary_sets:
        for row in rows:
            method = row.get("method", "")
            has_lambda = any(
                key.startswith("lambda_mean")
                and str(value) not in {"", "0", "0.0"}
                for key, value in row.items()
            )
            if method.startswith("A-RH-PADS") and has_lambda:
                table_rows.append(
                    [
                        scenario,
                        method,
                        fmt(row.get("lambda_mean_mean", "")),
                        fmt(row.get("lambda_std_mean", "")),
                        fmt(row.get("lambda_min_mean", "")),
                        fmt(row.get("lambda_max_mean", "")),
                    ]
                )
    return "\n## Table F: lambda_t Statistics\n\n" + markdown_table(
        ["Scenario", "Method", "lambda mean", "lambda std", "lambda min", "lambda max"],
        table_rows,
    ) + "\n"


def main():
    base_dir = Path(__file__).resolve().parent
    results_dir = base_dir / "results"
    output_path = results_dir / "adaptive_rh_pads_paper_tables.md"

    main_summary = read_rows(results_dir / "adaptive_rh_pads_main_summary.csv")
    abnormal_summary = read_rows(results_dir / "adaptive_rh_pads_abnormal_summary.csv")
    ablation_summary = read_rows(results_dir / "adaptive_rh_pads_ablation_summary.csv")
    vehicle_summary = read_rows(results_dir / "adaptive_vehicle_sim_summary.csv")
    significance = read_rows(results_dir / "adaptive_rh_pads_significance.csv")

    parts = [
        "# Adaptive A-RH-PADS Paper Tables\n",
        "Notes:\n",
        "- A-RH-PADS is the adaptive enhanced method.\n",
        "- RH-PADS is the fixed-weight baseline.\n",
        "- A larger lambda_t means the scheduler is more response-priority oriented.\n",
        "- A-RH-PADS does not optimize for shortest path only; it dynamically balances response utility and motion cost.\n",
        "- The vehicle experiment is a kinematic simulation only, not Gazebo, Nav2, or a real robot experiment.\n",
        summary_table(
            "Table A: Adaptive Main Experiment Results",
            main_summary,
            [
                "completed_task_num",
                "total_path_length",
                "total_inspection_time",
                "high_priority_avg_response_time",
                "priority_weighted_completion_time",
                "high_priority_top5_rate",
            ],
        ),
        summary_table(
            "Table B: Adaptive Abnormal Feedback Results",
            abnormal_summary,
            [
                "abnormal_priority_rate",
                "abnormal_avg_response_time",
                "high_priority_avg_response_time",
                "priority_weighted_completion_time",
                "total_path_length",
                "lambda_change",
            ],
        ),
        summary_table(
            "Table C: Adaptive Structural Ablation Results",
            ablation_summary,
            [
                "total_path_length",
                "high_priority_avg_response_time",
                "priority_weighted_completion_time",
                "abnormal_avg_response_time",
                "lambda_mean",
            ],
        ),
        summary_table(
            "Table D: Adaptive Vehicle Kinematic Simulation Results",
            vehicle_summary,
            [
                "completed_task_num",
                "total_planned_path_length",
                "vehicle_trajectory_length",
                "vehicle_execution_time",
                "high_priority_avg_response_time",
                "goal_success_rate",
            ],
        ),
        significance_table(significance),
        lambda_table(
            [
                ("main", main_summary),
                ("abnormal", abnormal_summary),
                ("ablation", ablation_summary),
                ("vehicle", vehicle_summary),
            ]
        ),
    ]

    output_path.write_text("".join(parts), encoding="utf-8")
    print(f"Saved: {output_path}")


if __name__ == "__main__":
    main()
