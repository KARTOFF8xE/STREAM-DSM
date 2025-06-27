import pandas as pd
from datetime import timedelta
import sys

def shift_times(csv_path, hours_shift, output_path="times_shifted.csv"):
    try:
        df = pd.read_csv(csv_path, parse_dates=["start_time", "stop_time"])
        
        df["start_time"] += timedelta(hours=hours_shift)
        df["stop_time"] += timedelta(hours=hours_shift)

        df.to_csv(output_path, index=False)
        print(f"Successfully shifted by {hours_shift}h, stored in: {output_path}")
    except Exception as e:
        print(f"Error during shifting: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 shift_times.py <path/to/.csv> <hours> <*path/to/destination.csv>")
    else:
        shift_times(sys.argv[1], int(sys.argv[2]), (sys.argv[3]))
