import os
import sys
import csv
import numpy as np
from scipy.stats import ttest_ind

def parse_file(filepath):
    with open(filepath, 'r') as file:
        lines = file.readlines()
    return {
        int(line.strip().split()[0]): int(line.strip().split()[1])
        for line in lines if line.strip()
    }

def collect_latencies(base_path):
    latencies = {}
    for freq in os.listdir(base_path):
        freq_path = os.path.join(base_path, freq)
        if not os.path.isdir(freq_path):
            continue
        for n in os.listdir(freq_path):
            n_path = os.path.join(freq_path, n)
            if not os.path.isdir(n_path):
                continue

            all_latencies = []
            for run in os.listdir(n_path):
                run_path = os.path.join(n_path, run)
                if not os.path.isdir(run_path):
                    continue

                received_file = os.path.join(run_path, "received.txt")
                send_file = os.path.join(run_path, "send.txt")
                if not os.path.exists(received_file) or not os.path.exists(send_file):
                    continue

                received_dict = parse_file(received_file)
                send_dict = parse_file(send_file)
                common_indices = received_dict.keys() & send_dict.keys()

                lat_run = [
                    (received_dict[j] - send_dict[j]) / 1000
                    for j in common_indices
                    if (received_dict[j] - send_dict[j]) < 1_000_000
                ]
                all_latencies.extend(lat_run)

            if all_latencies:
                latencies[(freq, n)] = latencies.get((freq, n), []) + all_latencies

    return latencies

def try_int(value):
    try:
        return int(value)
    except ValueError:
        return value

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

    lat_trace = collect_latencies(trace_path)
    lat_base = collect_latencies(base_path)

    results = []
    all_keys = set(lat_trace.keys()) | set(lat_base.keys())

    for key in all_keys:
        freq, n = key
        trace_vals = lat_trace.get(key, [])
        base_vals = lat_base.get(key, [])

        if len(trace_vals) > 1 and len(base_vals) > 1:
            t_stat, p_val = ttest_ind(trace_vals, base_vals, equal_var=False)
            mean_trace = np.mean(trace_vals)
            mean_base = np.mean(base_vals)
        else:
            t_stat, p_val = None, None
            mean_trace = np.mean(trace_vals) if trace_vals else None
            mean_base = np.mean(base_vals) if base_vals else None

        results.append([freq, n, mean_base, mean_trace, t_stat, p_val])

    # Sortieren nach frequency und n numerisch wenn möglich
    results.sort(key=lambda x: (try_int(x[0]), try_int(x[1])))

    output_file = os.path.join(root_path, "significance_per_config.csv")
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['frequency', 'n', 'mean_base', 'mean_trace', 't_stat', 'p_value'])
        writer.writerows(results)

    print(f"Signifikanz-Ergebnisse in {output_file} gespeichert.")

if __name__ == "__main__":
    main()
