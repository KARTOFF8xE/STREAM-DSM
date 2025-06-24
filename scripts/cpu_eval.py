import sys
import pandas as pd
import numpy as np

def parse_times(times_path):
    df = pd.read_csv(times_path, parse_dates=['start_time','stop_time'])
    return df

def load_cpu_data(cpu_path):
    df = pd.read_csv(cpu_path, parse_dates=['Time'])
    return df

def fill_none_with_zero(series):
    return series.fillna(0)

def compute_slopes(time_series, value_series):
    diffs = value_series.diff()
    timediffs = time_series.diff().dt.total_seconds()
    slopes = diffs / timediffs
    return slopes

def mean_with_zero_fill(values):
    filled = values.fillna(0)
    return filled.mean()

def median_no_fill(values):
    filtered = values.dropna()
    if filtered.empty:
        return 0
    return filtered.median()

def process_interval(cpu_df, start, stop):
    mask = (cpu_df['Time'] >= start) & (cpu_df['Time'] <= stop)
    interval_df = cpu_df.loc[mask].copy()
    return interval_df

def analyze_interval(interval_df):
    results = {}
    times = interval_df['Time']
    stream_keys = ['datamgmt/cpu', 'continuous/cpu', 'structural/cpu', 'neo4j/cpu', 'influxdb/cpu']
    ros2_keys = ['ros2talker/cpu', 'ros2listener/cpu']

    stream_mean_total = 0
    stream_median_total = 0
    ros2_mean_total = 0
    ros2_median_total = 0

    for col in interval_df.columns:
        if col == 'Time':
            continue
        series = interval_df[col]
        slopes = compute_slopes(times, series)
        mean_val = mean_with_zero_fill(slopes)
        median_val = median_no_fill(slopes)
        results[col + '_mean'] = mean_val
        results[col + '_median'] = median_val

        if col in stream_keys:
            stream_mean_total += mean_val
            stream_median_total += median_val
        elif col in ros2_keys:
            ros2_mean_total += mean_val
            ros2_median_total += median_val

    results['stream/cpu_mean'] = stream_mean_total
    results['stream/cpu_median'] = stream_median_total
    results['ros2/cpu_mean'] = ros2_mean_total
    results['ros2/cpu_median'] = ros2_median_total

    return results

def main(times_path, cpu_path, out_dir):
    times_df = parse_times(times_path)
    cpu_df = load_cpu_data(cpu_path)

    out_base = []
    out_trace = []

    for idx, row in times_df.iterrows():
        start = row['start_time']
        stop = row['stop_time']
        freq = row['frequency']
        npairs = row['num_pairs']
        ttype = row['type']

        interval_df = process_interval(cpu_df, start, stop)
        stats = analyze_interval(interval_df)

        entry = {'frequency': freq, 'num_pairs': npairs}
        entry.update(stats)

        if ttype == 'BASE':
            out_base.append(entry)
        else:
            out_trace.append(entry)

    base_df = pd.DataFrame(out_base).sort_values(['frequency','num_pairs'])
    trace_df = pd.DataFrame(out_trace).sort_values(['frequency','num_pairs'])

    base_df.to_csv(f"{out_dir}/cpu_usage_base.csv", index=False)
    trace_df.to_csv(f"{out_dir}/cpu_usage_trace.csv", index=False)

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])
