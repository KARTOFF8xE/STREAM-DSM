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
    data1 = read_csv_to_dict(file1_path)
    data2 = read_csv_to_dict(file2_path)

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

        median_diff = data1[key]['median'] - data2[key]['median']
        mean_diff = data1[key]['mean'] - data2[key]['mean']

        rows.append({
            'frequency': freq,
            'n': n,
            'median': median_diff,
            'mean': mean_diff
        })

    rows.sort(key=lambda row: (float(row['frequency']), int(row['n'])))

    with open(output_path, 'w', newline='') as csvfile:
        fieldnames = ['frequency', 'n', 'median', 'mean']
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
    parser = argparse.ArgumentParser(description='Compute differences between two CSV files and sort for TikZ matrix plot.')
    parser.add_argument('file1', help='First input CSV file')
    parser.add_argument('file2', help='Second input CSV file')
    parser.add_argument('output', help='Output CSV file with differences')

    args = parser.parse_args()
    main(args.file1, args.file2, args.output)
