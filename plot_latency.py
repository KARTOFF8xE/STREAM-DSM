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

print(f"Gemessene Nachrichten: {len(latencies_us)}")
print(f"Min: {min(latencies_us):.2f} µs")
print(f"Max: {max(latencies_us):.2f} µs")
print(f"Mittelwert: {sum(latencies_us)/len(latencies_us):.2f} µs")

plt.figure(figsize=(6, 5))
plt.boxplot(latencies_us, vert=True, patch_artist=True, showfliers=False)
plt.title("Message Latency (µs)")
plt.ylabel("Latency in Microseconds")
plt.grid(True)
plt.tight_layout()
plt.show()
