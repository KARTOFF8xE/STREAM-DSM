import os
import sys
import csv
import numpy as np

def parse_file(filepath):
    """Parst eine Datei zu einem Dictionary: {index: timestamp}"""
    with open(filepath, 'r') as file:
        lines = file.readlines()
    return {
        int(line.strip().split()[0]): int(line.strip().split()[1])
        for line in lines if line.strip()
    }

def process_directory(root_path):
    results = []

    for f in os.listdir(root_path):
        f_path = os.path.join(root_path, f)
        if not os.path.isdir(f_path):
            continue

        for n in os.listdir(f_path):
            n_path = os.path.join(f_path, n)
            if not os.path.isdir(n_path):
                continue

            latenzen = []

            for i in os.listdir(n_path):
                i_path = os.path.join(n_path, i)
                if not os.path.isdir(i_path):
                    continue

                received_file = os.path.join(i_path, "received.txt")
                send_file = os.path.join(i_path, "send.txt")

                if not os.path.exists(received_file) or not os.path.exists(send_file):
                    continue

                received_dict = parse_file(received_file)
                send_dict = parse_file(send_file)

                common_indices = received_dict.keys() & send_dict.keys()

                latenzen.extend([
                    received_dict[j] - send_dict[j]
                    for j in common_indices
                ])

            if latenzen:
                median_latenz = round(np.median(latenzen) / 1000, 2)
                mean_latenz = round(np.mean(latenzen) / 1000, 2)
                results.append((f, n, median_latenz, mean_latenz))

    return results

def write_csv(results, output_file):
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['frequency', 'n', 'median', 'mean'])
        for row in results:
            writer.writerow(row)

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 calc_latencies_csv.py /path/to/root")
        sys.exit(1)

    root_path = sys.argv[1]
    if not os.path.isdir(root_path):
        print(f"Invalid path: {root_path}")
        sys.exit(1)

    results = process_directory(root_path)
    output_file = os.path.join(root_path, "latencies.csv")
    write_csv(results, output_file)
    print(f"Finalised. Stored result in {output_file}")

if __name__ == "__main__":
    main()
