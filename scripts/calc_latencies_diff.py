import csv
import sys

def compare_csv_files(file1_path, file2_path, output_path):
    with open(file1_path, newline='') as f1, open(file2_path, newline='') as f2:
        reader1 = csv.DictReader(f1)
        reader2 = csv.DictReader(f2)

        if reader1.fieldnames != reader2.fieldnames:
            raise ValueError("Csv-headers do not match.")

        rows1 = list(reader1)
        rows2 = list(reader2)

        if len(rows1) != len(rows2):
            raise ValueError("Can not compare files (are of different lengths).")

        with open(output_path, mode='w', newline='') as fout:
            fieldnames = ['frequency', 'n', 'diff_median', 'diff_mean']
            writer = csv.DictWriter(fout, fieldnames=fieldnames)
            writer.writeheader()

            for row1, row2 in zip(rows1, rows2):
                try:
                    median1 = float(row1['median'])
                    median2 = float(row2['median'])
                    mean1 = float(row1['mean'])
                    mean2 = float(row2['mean'])
                except ValueError as e:
                    raise ValueError(f"invalid numerical value in line: {e}")

                writer.writerow({
                    'frequency': row1['frequency'],
                    'n': row1['n'],
                    'diff_median': median1 - median2,
                    'diff_mean': mean1 - mean2
                })

    print(f"Finalized, stored in: {output_path}")

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage: python3 compare_csv.py <file1.csv> <file2.csv> <output.csv>")
        sys.exit(1)

    compare_csv_files(sys.argv[1], sys.argv[2], sys.argv[3])
