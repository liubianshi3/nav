# Adaptive A-RH-PADS Paper Tables
Notes:
- A-RH-PADS is the adaptive enhanced method.
- RH-PADS is the fixed-weight baseline.
- A larger lambda_t means the scheduler is more response-priority oriented.
- A-RH-PADS does not optimize for shortest path only; it dynamically balances response utility and motion cost.
- The vehicle experiment is a kinematic simulation only, not Gazebo, Nav2, or a real robot experiment.

## Table A: Adaptive Main Experiment Results

| Method | completed_task_num | total_path_length | total_inspection_time | high_priority_avg_response_time | priority_weighted_completion_time | high_priority_top5_rate |
| --- | --- | --- | --- | --- | --- | --- |
| AStarOnly | 19.93 +/- 0.25 | 169.13 +/- 23.10 | 381.56 +/- 39.17 | 200.40 +/- 76.99 | 184.37 +/- 25.33 | 25.33 +/- 24.60 |
| TSP-2opt | 19.93 +/- 0.25 | 153.10 +/- 17.05 | 354.83 +/- 29.09 | 188.35 +/- 46.50 | 181.22 +/- 21.13 | 23.33 +/- 21.06 |
| Greedy-PADS | 19.93 +/- 0.25 | 236.63 +/- 29.84 | 494.06 +/- 50.20 | 170.82 +/- 52.09 | 208.61 +/- 27.24 | 44.67 +/- 24.46 |
| Priority-Greedy | 19.93 +/- 0.25 | 297.20 +/- 31.94 | 595.00 +/- 53.71 | 133.64 +/- 38.94 | 232.46 +/- 28.71 | 64.00 +/- 24.86 |
| RH-PADS | 19.93 +/- 0.25 | 220.27 +/- 22.30 | 466.78 +/- 37.64 | 130.12 +/- 40.49 | 183.10 +/- 18.64 | 53.33 +/- 27.46 |
| RH-PADS-L | 19.93 +/- 0.25 | 219.43 +/- 30.19 | 465.39 +/- 50.66 | 134.24 +/- 58.21 | 182.57 +/- 21.24 | 53.33 +/- 27.96 |
| A-RH-PADS | 19.93 +/- 0.25 | 207.07 +/- 21.53 | 444.78 +/- 36.27 | 143.00 +/- 54.37 | 178.72 +/- 20.70 | 48.67 +/- 27.63 |
| A-RH-PADS-L | 19.93 +/- 0.25 | 212.43 +/- 22.00 | 453.72 +/- 36.95 | 146.64 +/- 33.37 | 182.60 +/- 19.92 | 47.33 +/- 27.03 |

## Table B: Adaptive Abnormal Feedback Results

| Method | abnormal_priority_rate | abnormal_avg_response_time | high_priority_avg_response_time | priority_weighted_completion_time | total_path_length | lambda_change |
| --- | --- | --- | --- | --- | --- | --- |
| AStarOnly | 28.33 +/- 22.49 | 149.00 +/- 43.50 | 200.40 +/- 76.99 | 184.37 +/- 25.33 | 169.13 +/- 23.10 | 0.00 +/- 0.00 |
| Greedy-PADS | 55.00 +/- 20.13 | 137.92 +/- 40.94 | 176.60 +/- 63.66 | 220.13 +/- 36.61 | 250.53 +/- 33.97 | 0.00 +/- 0.00 |
| RH-PADS | 52.50 +/- 22.12 | 116.64 +/- 41.46 | 135.99 +/- 45.41 | 188.36 +/- 21.11 | 220.77 +/- 26.23 | 0.00 +/- 0.00 |
| RH-PADS-L | 50.00 +/- 27.07 | 121.83 +/- 43.26 | 134.89 +/- 42.64 | 187.99 +/- 25.00 | 219.73 +/- 23.57 | 0.00 +/- 0.00 |
| A-RH-PADS | 54.17 +/- 25.50 | 118.42 +/- 43.64 | 138.71 +/- 46.16 | 187.54 +/- 20.93 | 221.07 +/- 25.38 | 0.17 +/- 0.01 |
| A-RH-PADS-L | 47.50 +/- 27.35 | 126.96 +/- 45.17 | 147.22 +/- 49.18 | 188.30 +/- 21.13 | 219.60 +/- 24.16 | 0.17 +/- 0.01 |
| Priority-Greedy | 58.33 +/- 24.86 | 155.39 +/- 61.10 | 150.12 +/- 47.24 | 245.05 +/- 29.29 | 300.47 +/- 26.93 | 0.00 +/- 0.00 |

## Table C: Adaptive Structural Ablation Results

| Method | total_path_length | high_priority_avg_response_time | priority_weighted_completion_time | abnormal_avg_response_time | lambda_mean |
| --- | --- | --- | --- | --- | --- |
| A-RH-PADS-Full | 221.07 +/- 25.38 | 138.71 +/- 46.16 | 187.54 +/- 20.93 | 118.42 +/- 43.64 | 0.67 +/- 0.04 |
| A-RH-PADS-FixedLambda | 204.93 +/- 25.75 | 148.39 +/- 52.85 | 179.37 +/- 20.16 | 127.11 +/- 49.70 | 0.55 +/- 0.00 |
| A-RH-PADS-NoUrgencyPressure | 207.33 +/- 26.25 | 149.92 +/- 53.42 | 182.70 +/- 21.50 | 119.39 +/- 46.82 | 0.55 +/- 0.04 |
| A-RH-PADS-NoAbnormalPressure | 208.90 +/- 21.01 | 138.39 +/- 40.07 | 181.45 +/- 17.34 | 123.47 +/- 50.23 | 0.58 +/- 0.02 |
| A-RH-PADS-NoPathPressure | 221.90 +/- 24.92 | 136.98 +/- 47.34 | 189.55 +/- 21.77 | 122.25 +/- 46.11 | 0.73 +/- 0.03 |
| A-RH-PADS-ResponseOnly | 233.83 +/- 26.31 | 139.42 +/- 51.73 | 195.30 +/- 25.44 | 121.83 +/- 40.85 | 0.85 +/- 0.00 |
| A-RH-PADS-CostOnly | 189.60 +/- 25.67 | 158.14 +/- 42.44 | 175.07 +/- 18.50 | 149.79 +/- 53.31 | 0.25 +/- 0.00 |
| A-RH-PADS-NoFinishTimeResponse | 259.03 +/- 33.12 | 159.27 +/- 66.95 | 220.65 +/- 35.06 | 132.07 +/- 44.65 | 0.65 +/- 0.03 |

## Table D: Adaptive Vehicle Kinematic Simulation Results

| Method | completed_task_num | total_planned_path_length | vehicle_trajectory_length | vehicle_execution_time | high_priority_avg_response_time | goal_success_rate |
| --- | --- | --- | --- | --- | --- | --- |
| A-RH-PADS-L | 19.93 +/- 0.25 | 212.83 +/- 21.98 | 193.74 +/- 20.52 | 382.57 +/- 40.60 | 162.10 +/- 33.08 | 100.00 +/- 0.00 |
| RH-PADS-L | 19.93 +/- 0.25 | 232.93 +/- 25.05 | 211.95 +/- 23.61 | 420.87 +/- 45.18 | 125.70 +/- 34.82 | 100.00 +/- 0.00 |
| AStarOnly | 19.93 +/- 0.25 | 167.20 +/- 21.86 | 149.83 +/- 20.50 | 302.39 +/- 37.42 | 183.09 +/- 42.32 | 100.00 +/- 0.00 |
| TSP-2opt | 19.93 +/- 0.25 | 164.30 +/- 20.86 | 147.04 +/- 19.47 | 296.87 +/- 36.88 | 183.70 +/- 48.29 | 100.00 +/- 0.00 |
| Greedy-PADS | 19.93 +/- 0.25 | 203.30 +/- 23.58 | 183.61 +/- 21.50 | 367.26 +/- 40.88 | 172.78 +/- 58.97 | 100.00 +/- 0.00 |
| Priority-Greedy | 19.93 +/- 0.25 | 297.20 +/- 31.94 | 270.35 +/- 28.38 | 529.51 +/- 54.73 | 151.91 +/- 44.21 | 100.00 +/- 0.00 |

## Table E: Significance Test Summary

| Experiment | Comparison | Metric | n | Mean diff | t | p | Conclusion |
| --- | --- | --- | --- | --- | --- | --- | --- |
| main | A-RH-PADS vs RH-PADS | total_path_length | 30 | -13.20 | -3.886 | 0.0005 | significant |
| main | A-RH-PADS vs RH-PADS | high_priority_avg_response_time | 30 | 12.87 | 2.992 | 0.0056 | significant |
| main | A-RH-PADS vs RH-PADS | priority_weighted_completion_time | 30 | -4.38 | -3.300 | 0.0026 | significant |
| main | A-RH-PADS-L vs RH-PADS-L | total_path_length | 30 | -7.00 | -1.760 | 0.0890 | not_significant |
| main | A-RH-PADS-L vs RH-PADS-L | high_priority_avg_response_time | 30 | 12.40 | 1.265 | 0.2159 | not_significant |
| main | A-RH-PADS-L vs RH-PADS-L | priority_weighted_completion_time | 30 | 0.03 | 0.013 | 0.9896 | not_significant |
| main | A-RH-PADS vs AStarOnly | total_path_length | 30 | 37.93 | 10.327 | 0.0000 | significant |
| main | A-RH-PADS vs AStarOnly | high_priority_avg_response_time | 30 | -57.40 | -4.506 | 0.0001 | significant |
| main | A-RH-PADS vs AStarOnly | priority_weighted_completion_time | 30 | -5.65 | -1.457 | 0.1559 | not_significant |
| main | A-RH-PADS vs TSP-2opt | total_path_length | 30 | 53.97 | 15.368 | 0.0000 | significant |
| main | A-RH-PADS vs TSP-2opt | high_priority_avg_response_time | 30 | -45.35 | -4.524 | 0.0001 | significant |
| main | A-RH-PADS vs TSP-2opt | priority_weighted_completion_time | 30 | -2.50 | -0.565 | 0.5767 | not_significant |
| main | A-RH-PADS vs Priority-Greedy | total_path_length | 30 | -90.13 | -18.296 | 0.0000 | significant |
| main | A-RH-PADS vs Priority-Greedy | high_priority_avg_response_time | 30 | 9.36 | 0.925 | 0.3628 | not_significant |
| main | A-RH-PADS vs Priority-Greedy | priority_weighted_completion_time | 30 | -53.74 | -12.238 | 0.0000 | significant |
| abnormal | A-RH-PADS vs RH-PADS | total_path_length | 30 | 0.30 | 0.086 | 0.9322 | not_significant |
| abnormal | A-RH-PADS vs RH-PADS | high_priority_avg_response_time | 30 | 2.72 | 0.632 | 0.5322 | not_significant |
| abnormal | A-RH-PADS vs RH-PADS | priority_weighted_completion_time | 30 | -0.83 | -0.480 | 0.6350 | not_significant |
| abnormal | A-RH-PADS vs RH-PADS | abnormal_avg_response_time | 30 | 1.78 | 0.346 | 0.7322 | not_significant |
| abnormal | A-RH-PADS-L vs RH-PADS-L | total_path_length | 30 | -0.13 | -0.035 | 0.9726 | not_significant |
| abnormal | A-RH-PADS-L vs RH-PADS-L | high_priority_avg_response_time | 30 | 12.34 | 2.155 | 0.0396 | significant |
| abnormal | A-RH-PADS-L vs RH-PADS-L | priority_weighted_completion_time | 30 | 0.31 | 0.147 | 0.8844 | not_significant |
| abnormal | A-RH-PADS-L vs RH-PADS-L | abnormal_avg_response_time | 30 | 5.12 | 0.628 | 0.5348 | not_significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-FixedLambda | total_path_length | 30 | 16.13 | 3.654 | 0.0010 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-FixedLambda | high_priority_avg_response_time | 30 | -9.68 | -1.521 | 0.1390 | not_significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-FixedLambda | priority_weighted_completion_time | 30 | 8.16 | 3.265 | 0.0028 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-FixedLambda | abnormal_avg_response_time | 30 | -8.69 | -1.442 | 0.1600 | not_significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoUrgencyPressure | total_path_length | 30 | 13.73 | 3.712 | 0.0009 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoUrgencyPressure | high_priority_avg_response_time | 30 | -11.21 | -1.902 | 0.0672 | not_significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoUrgencyPressure | priority_weighted_completion_time | 30 | 4.84 | 2.165 | 0.0387 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoUrgencyPressure | abnormal_avg_response_time | 30 | -0.97 | -0.173 | 0.8641 | not_significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoAbnormalPressure | total_path_length | 30 | 12.17 | 3.164 | 0.0036 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoAbnormalPressure | high_priority_avg_response_time | 30 | 0.32 | 0.061 | 0.9519 | not_significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoAbnormalPressure | priority_weighted_completion_time | 30 | 6.09 | 2.496 | 0.0185 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoAbnormalPressure | abnormal_avg_response_time | 30 | -5.06 | -1.292 | 0.2064 | not_significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoPathPressure | total_path_length | 30 | -0.83 | -0.358 | 0.7226 | not_significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoPathPressure | high_priority_avg_response_time | 30 | 1.73 | 0.935 | 0.3577 | not_significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoPathPressure | priority_weighted_completion_time | 30 | -2.01 | -1.193 | 0.2424 | not_significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoPathPressure | abnormal_avg_response_time | 30 | -3.83 | -1.120 | 0.2718 | not_significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-ResponseOnly | total_path_length | 30 | -12.77 | -4.294 | 0.0002 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-ResponseOnly | high_priority_avg_response_time | 30 | -0.71 | -0.161 | 0.8732 | not_significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-ResponseOnly | priority_weighted_completion_time | 30 | -7.76 | -2.975 | 0.0059 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-ResponseOnly | abnormal_avg_response_time | 30 | -3.42 | -0.476 | 0.6374 | not_significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-CostOnly | total_path_length | 30 | 31.47 | 6.760 | 0.0000 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-CostOnly | high_priority_avg_response_time | 30 | -19.43 | -2.182 | 0.0373 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-CostOnly | priority_weighted_completion_time | 30 | 12.47 | 4.859 | 0.0000 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-CostOnly | abnormal_avg_response_time | 30 | -31.37 | -3.501 | 0.0015 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoFinishTimeResponse | total_path_length | 30 | -37.97 | -8.644 | 0.0000 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoFinishTimeResponse | high_priority_avg_response_time | 30 | -20.56 | -2.825 | 0.0085 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoFinishTimeResponse | priority_weighted_completion_time | 30 | -33.11 | -7.029 | 0.0000 | significant |
| ablation | A-RH-PADS-Full vs A-RH-PADS-NoFinishTimeResponse | abnormal_avg_response_time | 30 | -13.65 | -2.122 | 0.0425 | significant |
| vehicle | A-RH-PADS-L vs RH-PADS-L | total_path_length | 30 | -20.10 | -4.896 | 0.0000 | significant |
| vehicle | A-RH-PADS-L vs RH-PADS-L | high_priority_avg_response_time | 30 | 36.40 | 5.709 | 0.0000 | significant |
| vehicle | A-RH-PADS-L vs RH-PADS-L | priority_weighted_completion_time | 30 | 9.71 | 3.391 | 0.0020 | significant |
| vehicle | A-RH-PADS-L vs RH-PADS-L | vehicle_trajectory_length | 30 | -18.21 | -4.678 | 0.0001 | significant |
| vehicle | A-RH-PADS-L vs RH-PADS-L | vehicle_execution_time | 30 | -38.29 | -5.080 | 0.0000 | significant |
| vehicle | A-RH-PADS-L vs TSP-2opt | total_path_length | 30 | 48.53 | 14.797 | 0.0000 | significant |
| vehicle | A-RH-PADS-L vs TSP-2opt | high_priority_avg_response_time | 30 | -21.60 | -2.495 | 0.0185 | significant |
| vehicle | A-RH-PADS-L vs TSP-2opt | priority_weighted_completion_time | 30 | 2.95 | 0.780 | 0.4416 | not_significant |
| vehicle | A-RH-PADS-L vs TSP-2opt | vehicle_trajectory_length | 30 | 46.70 | 14.220 | 0.0000 | significant |
| vehicle | A-RH-PADS-L vs TSP-2opt | vehicle_execution_time | 30 | 85.70 | 13.695 | 0.0000 | significant |
| vehicle | A-RH-PADS-L vs Priority-Greedy | total_path_length | 30 | -84.37 | -16.941 | 0.0000 | significant |
| vehicle | A-RH-PADS-L vs Priority-Greedy | high_priority_avg_response_time | 30 | 10.20 | 1.209 | 0.2365 | not_significant |
| vehicle | A-RH-PADS-L vs Priority-Greedy | priority_weighted_completion_time | 30 | -51.49 | -12.057 | 0.0000 | significant |
| vehicle | A-RH-PADS-L vs Priority-Greedy | vehicle_trajectory_length | 30 | -76.61 | -16.592 | 0.0000 | significant |
| vehicle | A-RH-PADS-L vs Priority-Greedy | vehicle_execution_time | 30 | -146.94 | -16.730 | 0.0000 | significant |

## Table F: lambda_t Statistics

| Scenario | Method | lambda mean | lambda std | lambda min | lambda max |
| --- | --- | --- | --- | --- | --- |
| main | A-RH-PADS | 0.57 | 0.04 | 0.50 | 0.63 |
| main | A-RH-PADS-L | 0.57 | 0.04 | 0.50 | 0.63 |
| abnormal | A-RH-PADS | 0.67 | 0.10 | 0.51 | 0.79 |
| abnormal | A-RH-PADS-L | 0.67 | 0.10 | 0.51 | 0.79 |
| ablation | A-RH-PADS-Full | 0.67 | 0.10 | 0.51 | 0.79 |
| ablation | A-RH-PADS-FixedLambda | 0.55 | 0.00 | 0.55 | 0.55 |
| ablation | A-RH-PADS-NoUrgencyPressure | 0.55 | 0.10 | 0.42 | 0.67 |
| ablation | A-RH-PADS-NoAbnormalPressure | 0.58 | 0.04 | 0.50 | 0.65 |
| ablation | A-RH-PADS-NoPathPressure | 0.73 | 0.08 | 0.59 | 0.81 |
| ablation | A-RH-PADS-ResponseOnly | 0.85 | 0.00 | 0.85 | 0.85 |
| ablation | A-RH-PADS-CostOnly | 0.25 | 0.00 | 0.25 | 0.25 |
| ablation | A-RH-PADS-NoFinishTimeResponse | 0.65 | 0.11 | 0.49 | 0.78 |
| vehicle | A-RH-PADS-L | 0.57 | 0.04 | 0.50 | 0.63 |
