import csv
from pathlib import Path

import matplotlib.pyplot as plt


plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC",
    "SimHei",
    "Microsoft YaHei",
    "WenQuanYi Zen Hei",
    "Arial Unicode MS",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False


def read_rows(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def f(row, key):
    try:
        return float(row.get(key, 0.0) or 0.0)
    except Exception:
        return 0.0


def by_method(rows):
    return {row["method"]: row for row in rows}


def save_bar(path, title, methods, series):
    if not methods:
        return
    x = list(range(len(methods)))
    width = 0.35 if len(series) <= 2 else 0.25
    fig, ax = plt.subplots(figsize=(10, 5))
    for i, (label, values) in enumerate(series):
        offset = (i - (len(series) - 1) / 2.0) * width
        ax.bar([item + offset for item in x], values, width=width, label=label)
    ax.set_title(title)
    ax.set_xticks(x)
    ax.set_xticklabels(methods, rotation=25, ha="right")
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=300)
    plt.close(fig)


def main_compare(summary, output_dir):
    methods = [
        "AStarOnly",
        "TSP-2opt",
        "Greedy-PADS",
        "Priority-Greedy",
        "RH-PADS",
        "RH-PADS-L",
        "A-RH-PADS",
        "A-RH-PADS-L",
    ]
    lookup = by_method(summary)
    methods = [method for method in methods if method in lookup]
    save_bar(
        output_dir / "fig_adaptive_main_compare.png",
        "Adaptive Main Experiment",
        methods,
        [
            ("Path length", [f(lookup[m], "total_path_length_mean") for m in methods]),
            ("HP response", [f(lookup[m], "high_priority_avg_response_time_mean") for m in methods]),
        ],
    )


def lambda_distribution(result_sets, output_dir):
    labels = []
    data = []
    for scenario, rows in result_sets:
        grouped = {}
        for row in rows:
            method = row.get("method", "")
            if method.startswith("A-RH-PADS"):
                grouped.setdefault(f"{scenario}-{method}", []).append(f(row, "lambda_mean"))
        for label, values in grouped.items():
            if values and any(value != 0.0 for value in values):
                labels.append(label)
                data.append(values)
    if not data:
        return
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.boxplot(data, labels=labels, showmeans=True)
    ax.set_title("lambda_t Distribution")
    ax.set_ylabel("lambda_t")
    ax.tick_params(axis="x", rotation=30)
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_dir / "fig_adaptive_lambda_distribution.png", dpi=300)
    plt.close(fig)


def abnormal_lambda_change(rows, output_dir):
    methods = ["A-RH-PADS", "A-RH-PADS-L"]
    data = {method: {"before": [], "after": []} for method in methods}
    for row in rows:
        method = row.get("method")
        if method in data:
            data[method]["before"].append(f(row, "lambda_before_abnormal"))
            data[method]["after"].append(f(row, "lambda_after_abnormal"))
    labels = []
    before = []
    after = []
    for method in methods:
        if data[method]["before"]:
            labels.append(method)
            before.append(sum(data[method]["before"]) / len(data[method]["before"]))
            after.append(sum(data[method]["after"]) / len(data[method]["after"]))
    save_bar(
        output_dir / "fig_adaptive_abnormal_lambda_change.png",
        "lambda_t Before and After Abnormal Trigger",
        labels,
        [("Before", before), ("After", after)],
    )


def ablation_compare(summary, output_dir):
    methods = [row["method"] for row in summary]
    lookup = by_method(summary)
    save_bar(
        output_dir / "fig_adaptive_ablation.png",
        "Adaptive Ablation",
        methods,
        [
            ("PWCT", [f(lookup[m], "priority_weighted_completion_time_mean") for m in methods]),
            ("Abnormal response", [f(lookup[m], "abnormal_avg_response_time_mean") for m in methods]),
        ],
    )


def vehicle_compare(summary, output_dir):
    methods = [row["method"] for row in summary]
    lookup = by_method(summary)
    save_bar(
        output_dir / "fig_adaptive_vehicle_sim_compare.png",
        "Vehicle Kinematic Simulation",
        methods,
        [
            ("Trajectory length", [f(lookup[m], "vehicle_trajectory_length_mean") for m in methods]),
            ("HP response", [f(lookup[m], "high_priority_avg_response_time_mean") for m in methods]),
        ],
    )


def main():
    base_dir = Path(__file__).resolve().parent
    results_dir = base_dir / "results"
    output_dir = results_dir / "figures" / "adaptive_rh_pads"
    output_dir.mkdir(parents=True, exist_ok=True)

    main_summary = read_rows(results_dir / "adaptive_rh_pads_main_summary.csv")
    abnormal_results = read_rows(results_dir / "adaptive_rh_pads_abnormal_results.csv")
    abnormal_summary = read_rows(results_dir / "adaptive_rh_pads_abnormal_summary.csv")
    ablation_results = read_rows(results_dir / "adaptive_rh_pads_ablation_results.csv")
    ablation_summary = read_rows(results_dir / "adaptive_rh_pads_ablation_summary.csv")
    vehicle_summary = read_rows(results_dir / "adaptive_vehicle_sim_summary.csv")
    main_results = read_rows(results_dir / "adaptive_rh_pads_main_results.csv")
    vehicle_results = read_rows(results_dir / "adaptive_vehicle_sim_results.csv")

    main_compare(main_summary, output_dir)
    lambda_distribution(
        [
            ("main", main_results),
            ("abnormal", abnormal_results),
            ("ablation", ablation_results),
            ("vehicle", vehicle_results),
        ],
        output_dir,
    )
    abnormal_lambda_change(abnormal_results, output_dir)
    ablation_compare(ablation_summary, output_dir)
    vehicle_compare(vehicle_summary, output_dir)

    for path in sorted(output_dir.glob("*.png")):
        print(f"Saved: {path}")


if __name__ == "__main__":
    main()
