import csv
import argparse

def read_csv_to_dict(file_path):
    data = {}
    with open(file_path, 'r', newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            key = (row['frequency'], row['n'])
            data[key] = {
                'median': float(row['median']),
                'mean': float(row['mean']),
            }
    return data

def main(file1_path, file2_path, output_path):
    data1 = read_csv_to_dict(file1_path)  # TRACE
    data2 = read_csv_to_dict(file2_path)  # BASE

    all_keys = set(data1.keys()).union(set(data2.keys()))
    warnings = []

    rows = []

    for key in all_keys:
        freq, n = key
        if key not in data1:
            warnings.append(f"Missing in {file1_path}: frequency={freq}, n={n}")
            continue
        if key not in data2:
            warnings.append(f"Missing in {file2_path}: frequency={freq}, n={n}")
            continue

        base_median = data2[key]['median']
        base_mean = data2[key]['mean']
        trace_median = data1[key]['median']
        trace_mean = data1[key]['mean']

        # Differenz
        median_diff = trace_median - base_median
        mean_diff = trace_mean - base_mean

        # Overhead-Berechnung (TRACE/BASE - 1)
        try:
            median_dev = (trace_median / base_median) - 1
        except ZeroDivisionError:
            median_dev = float('inf')

        try:
            mean_dev = (trace_mean / base_mean) - 1
        except ZeroDivisionError:
            mean_dev = float('inf')

        rows.append({
            'frequency': freq,
            'n': n,
            'diff/median': median_diff,
            'dev/median': median_dev,
            'diff/mean': mean_diff,
            'dev/mean': mean_dev
        })

    # Sortiere nach frequency (float) und n (int)
    rows.sort(key=lambda row: (float(row['frequency']), int(row['n'])))

    with open(output_path, 'w', newline='') as csvfile:
        fieldnames = ['frequency', 'n', 'diff/median', 'dev/median', 'diff/mean', 'dev/mean']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    if warnings:
        print("Warnings:")
        for w in warnings:
            print("  -", w)
    else:
        print("All entries matched and output written to:", output_path)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Compute overhead (diff and dev-1) between TRACE and BASE CSV files.')
    parser.add_argument('file1', help='TRACE CSV file (numerator)')
    parser.add_argument('file2', help='BASE CSV file (denominator)')
    parser.add_argument('output', help='Output CSV file with overhead data')

    args = parser.parse_args()
    main(args.file1, args.file2, args.output)
