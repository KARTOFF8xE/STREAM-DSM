import os
import sys
import csv
import numpy as np
from scipy.stats import ttest_ind, mannwhitneyu, shapiro
import matplotlib.pyplot as plt
from tqdm import tqdm

def parse_file(filepath):
    with open(filepath, 'r') as file:
        lines = file.readlines()
    return {
        int(line.strip().split()[0]): int(line.strip().split()[1])
        for line in lines if line.strip()
    }

def collect_latencies(base_path):
    latencies = {}
    freqs = [f for f in os.listdir(base_path) if os.path.isdir(os.path.join(base_path, f))]
    for freq in tqdm(freqs, desc="Frequencies"):
        freq_path = os.path.join(base_path, freq)
        ns = [n for n in os.listdir(freq_path) if os.path.isdir(os.path.join(freq_path, n))]
        for n in tqdm(ns, desc=f"Pairs at freq {freq}", leave=False):
            n_path = os.path.join(freq_path, n)
            all_latencies = []
            runs = [run for run in os.listdir(n_path) if os.path.isdir(os.path.join(n_path, run))]
            for run in tqdm(runs, desc=f"Runs at freq {freq}, pair {n}", leave=False):
                run_path = os.path.join(n_path, run)
                received_file = os.path.join(run_path, "received.txt")
                send_file = os.path.join(run_path, "send.txt")
                if not os.path.exists(received_file) or not os.path.exists(send_file):
                    continue

                received_dict = parse_file(received_file)
                send_dict = parse_file(send_file)
                common_indices = received_dict.keys() & send_dict.keys()

                lat_run = [
                    (received_dict[j] - send_dict[j]) / 1000  # ns → µs
                    for j in common_indices
                    if (received_dict[j] - send_dict[j]) < 1_000_000
                ]
                all_latencies.extend(lat_run)

            if all_latencies:
                latencies[(freq, n)] = latencies.get((freq, n), []) + all_latencies

    return latencies

def try_float(value):
    try:
        return float(value)
    except ValueError:
        return value

def visualize_distribution(lat_trace, lat_base, freq, n):
    key = (freq, n)
    if key not in lat_trace or key not in lat_base:
        print(f"No data for visualization: freq={freq}, n={n}")
        return

    trace_vals = lat_trace[key]
    base_vals = lat_base[key]

    plt.figure(figsize=(10, 6))
    plt.hist(base_vals, bins=50, alpha=0.5, label='BASE', color='blue', density=True)
    plt.hist(trace_vals, bins=50, alpha=0.5, label='TRACE', color='orange', density=True)
    plt.title(f'Latency Distribution for freq={freq}, n={n}')
    plt.xlabel('Latency [µs]')
    plt.ylabel('Normalized frequency')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 significance_test.py /path/to/root")
        sys.exit(1)

    root_path = sys.argv[1]
    trace_path = os.path.join(root_path, "TRACE")
    base_path = os.path.join(root_path, "BASE")

    if not os.path.isdir(trace_path) or not os.path.isdir(base_path):
        print("Ordner TRACE oder BASE nicht gefunden.")
        sys.exit(1)

    print("Collecting TRACE latencies...")
    lat_trace = collect_latencies(trace_path)
    print("Collecting BASE latencies...")
    lat_base = collect_latencies(base_path)

    all_keys = set(lat_trace.keys()) | set(lat_base.keys())

    results_base = []
    results_trace = []

    print("Führe Shapiro-Wilk-Tests durch...")

    for key in tqdm(sorted(all_keys, key=lambda x: (try_float(x[0]), try_float(x[1]))), desc="Normalverteilungstest"):
        freq, n = key

        # BASE
        base_vals = lat_base.get(key, [])
        if len(base_vals) > 3:
            _, p_base = shapiro(base_vals)
            is_normal_base = int(p_base > 0.05)
            mean_base = np.mean(base_vals)
        else:
            p_base = ''
            is_normal_base = ''
            mean_base = ''

        results_base.append([
            freq, n, mean_base, p_base, is_normal_base
        ])

        # TRACE
        trace_vals = lat_trace.get(key, [])
        if len(trace_vals) > 3:
            _, p_trace = shapiro(trace_vals)
            is_normal_trace = int(p_trace > 0.05)
            mean_trace = np.mean(trace_vals)
        else:
            p_trace = ''
            is_normal_trace = ''
            mean_trace = ''

        results_trace.append([
            freq, n, mean_trace, p_trace, is_normal_trace
        ])

    # CSV schreiben
    base_file = os.path.join(root_path, "normaltest_base.csv")
    trace_file = os.path.join(root_path, "normaltest_trace.csv")

    with open(base_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['frequency', 'n', 'mean_latency', 'shapiro_p_value', 'is_normal'])
        writer.writerows(results_base)

    with open(trace_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['frequency', 'n', 'mean_latency', 'shapiro_p_value', 'is_normal'])
        writer.writerows(results_trace)

    print(f"BASE-Ergebnisse gespeichert in: {base_file}")
    print(f"TRACE-Ergebnisse gespeichert in: {trace_file}")



if __name__ == "__main__":
    main()
