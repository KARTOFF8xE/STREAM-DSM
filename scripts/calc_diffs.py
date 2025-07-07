import sys
import os
import pandas as pd


def main():
    if len(sys.argv) != 4:
        print("Usage: python script.py <path/to/csv> <column1> <column2>")
        sys.exit(1)

    csv_path = sys.argv[1]
    col1 = sys.argv[2]
    col2 = sys.argv[3]

    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading the file: {e}")
        sys.exit(1)

    if col1 not in df.columns or col2 not in df.columns:
        print(f"One or both columns '{col1}' and '{col2}' not found in the CSV.")
        sys.exit(1)

    if 'frequency' not in df.columns or 'num_pairs' not in df.columns:
        print("Columns 'frequency' and 'num_pairs' must be present in the CSV.")
        sys.exit(1)

    rel_col_name = f"rel_{col1}_{col2}"
    rel_values = []

    for i, row in df.iterrows():
        numerator = row[col1]
        denominator = row[col2]
        if denominator == 0:
            print(f"Division by zero at row {i} (frequency={row['frequency']}, num_pairs={row['num_pairs']})")
            rel_values.append("")
        else:
            rel_values.append(numerator / denominator)

    df_out = df[['frequency', 'num_pairs']].copy()
    df_out[rel_col_name] = rel_values

    out_dir = os.path.dirname(csv_path)
    col1 = col1.replace("/", "")
    col2 = col2.replace("/", "")
    out_filename = f"rel_{col1}_{col2}.csv"
    out_path = os.path.join(out_dir, out_filename)

    try:
        df_out.to_csv(out_path, index=False)
        print(f"File successfully written: {out_path}")
    except Exception as e:
        print(f"Error writing the output file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
