import os
import csv
import argparse

def read_timestamps(file_path):
    timestamps = {}
    with open(file_path, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 2:
                continue
            x, timestamp = parts
            timestamps[x] = int(timestamp)
    return timestamps

def compute_latencies_us(send_path, receive_path):
    send = read_timestamps(send_path)
    receive = read_timestamps(receive_path)

    latencies_us = []
    for x in send:
        if x in receive:
            latency_ns = receive[x] - send[x]
            latency_us = latency_ns / 1000  # nanosekunden -> mikrosekunden
            latencies_us.append(latency_us)

    return latencies_us

def save_latencies(latencies_us, output_path):
    with open(output_path, "w") as f:
        for latency in latencies_us:
            f.write(f"{latency:.3f}\n")  # 3 Nachkommastellen

def main(root_path):
    output_csv_path = os.path.join(root_path, "latencies.csv")

    with open(output_csv_path, mode="w", newline="") as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["frequency", "n", "average_latency_us"])

        for freq_dir in sorted(os.listdir(root_path)):
            freq_path = os.path.join(root_path, freq_dir)
            if not os.path.isdir(freq_path):
                continue

            try:
                freq = float(freq_dir)
            except ValueError:
                continue

            for n_dir in sorted(os.listdir(freq_path)):
                n_path = os.path.join(freq_path, n_dir)
                if not os.path.isdir(n_path):
                    continue

                try:
                    n = int(n_dir)
                except ValueError:
                    continue

                total_latency = 0
                valid_runs = 0

                for i in range(n):
                    run_path = os.path.join(n_path, str(i))
                    send_path = os.path.join(run_path, "send.txt")
                    receive_path = os.path.join(run_path, "received.txt")

                    if not os.path.exists(send_path) or not os.path.exists(receive_path):
                        continue

                    latencies_us = compute_latencies_us(send_path, receive_path)

                    if not latencies_us:
                        continue

                    latencies_output_path = os.path.join(run_path, "latencies.txt")
                    save_latencies(latencies_us, latencies_output_path)

                    avg_latency = sum(latencies_us) / len(latencies_us)
                    total_latency += avg_latency
                    valid_runs += 1

                if valid_runs > 0:
                    overall_avg_latency = total_latency / valid_runs
                    csv_writer.writerow([freq, n, round(overall_avg_latency, 3)])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Berechne durchschnittliche Latenzen (in µs) aus einer Ordnerstruktur.")
    parser.add_argument("root_path", help="Pfad zum Root-Ordner mit der freq/n/i Struktur")
    args = parser.parse_args()

    main(args.root_path)
