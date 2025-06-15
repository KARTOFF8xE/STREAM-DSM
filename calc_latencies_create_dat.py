import numpy as np
import matplotlib.pyplot as plt

def read_timestamps(file_path):
    data = {}
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 2:
                idx, ts = int(parts[0]), int(parts[1])
                data[idx] = ts
    return data

send_times = read_timestamps("send.txt")
recv_times = read_timestamps("received.txt")

common_indices = sorted(set(send_times.keys()) & set(recv_times.keys()))
latencies_ns = [recv_times[i] - send_times[i] for i in common_indices]
latencies_us = [lat / 1000.0 for lat in latencies_ns]

threshold = np.percentile(latencies_us, 99)
latencies_filtered = [lat for lat in latencies_us if lat <= threshold]

with open("latencies.dat", "w") as f:
    for lat in latencies_filtered:
        f.write(f"{lat}\n")

lower_quartile = np.percentile(latencies_filtered, 25)
higher_quartile = np.percentile(latencies_filtered, 75)
median = np.median(latencies_filtered)

iqr = higher_quartile - lower_quartile
whisker_lower = lower_quartile - 1.5 * iqr
whisker_upper = higher_quartile + 1.5 * iqr

print(f"Gesamtzahl Nachrichten: {len(latencies_us)}")
print(f"Nach Filter (<= 99. Perzentil): {len(latencies_filtered)}")
print(f"Mittelwert: {np.mean(latencies_filtered):.2f} µs")
print(f"Max: {max(latencies_filtered):.2f} µs")
print(f"Obere Whisker-Grenze: {whisker_upper:.2f} µs")
print(f"Oberes Quartil (75%): {higher_quartile:.2f} µs")
print(f"Median: {median:.2f} µs")
print(f"Unteres Quartil (25%): {lower_quartile:.2f} µs")
print(f"Untere Whisker-Grenze: {whisker_lower:.2f} µs")
print(f"Min: {min(latencies_filtered):.2f} µs")

plt.figure(figsize=(6,5))
plt.boxplot(latencies_filtered, vert=True, patch_artist=True, showfliers=False)
plt.title("Message Latency (µs)")
plt.ylabel("Latenz in µs")
plt.grid(True)
plt.tight_layout()
plt.show()
