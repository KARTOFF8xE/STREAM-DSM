import os
import sys
import csv
import numpy as np
from scipy.stats import ttest_ind, mannwhitneyu
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
                    if (received_dict[j] - send_dict[j]) < 1_000_000  # Filter: <1s
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

    print("Available freq,n keys TRACE:", sorted(lat_trace.keys()))
    print("Available freq,n keys BASE:", sorted(lat_base.keys()))

    results = []
    all_keys = set(lat_trace.keys()) | set(lat_base.keys())

    for key in all_keys:
        freq, n = key
        trace_vals = lat_trace.get(key, [])
        base_vals = lat_base.get(key, [])

        if len(trace_vals) > 1 and len(base_vals) > 1:
            # Neben t-Test auch Mann-Whitney U-Test für Robustheit
            t_stat, p_val = ttest_ind(trace_vals, base_vals, equal_var=False)
            mw_stat, mw_p_val = mannwhitneyu(trace_vals, base_vals, alternative='two-sided')
            mean_trace = np.mean(trace_vals)
            mean_base = np.mean(base_vals)
        else:
            t_stat, p_val, mw_stat, mw_p_val = None, None, None, None
            mean_trace = np.mean(trace_vals) if trace_vals else None
            mean_base = np.mean(base_vals) if base_vals else None

        results.append([freq, n, mean_base, mean_trace, t_stat, p_val, mw_stat, mw_p_val])

    # Sortieren numerisch (Frequenz und n)
    results.sort(key=lambda x: (try_float(x[0]), try_float(x[1])))

    output_file = os.path.join(root_path, "significance_per_config.csv")
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['frequency', 'n', 'mean_base', 'mean_trace', 't_stat', 't_p_value', 'mw_stat', 'mw_p_value'])
        writer.writerows(results)

    print(f"Signifikanz-Ergebnisse in {output_file} gespeichert.")

    # Beispiel-Visualisierungen — passe hier an, welche freq/n du sehen willst:
    # Nutze echte Strings der Ordner, z.B. "10.0", "3" usw.
    to_visualize = [
        ("10.0", "3"),
        ("20.0", "1"),
    ]
    for freq, n in to_visualize:
        visualize_distribution(lat_trace, lat_base, freq, n)


if __name__ == "__main__":
    main()
