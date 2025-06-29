import sys
import pandas as pd
import numpy as np

def parse_times(times_path):
    df = pd.read_csv(times_path, parse_dates=['start_time', 'stop_time'])
    return df

def load_io_data(io_path):
    df = pd.read_csv(io_path, parse_dates=['Time'])
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

def process_interval(io_df, start, stop):
    mask = (io_df['Time'] >= start) & (io_df['Time'] <= stop)
    interval_df = io_df.loc[mask].copy()
    return interval_df

def analyze_interval(interval_df):
    results = {}
    times = interval_df['Time']

    write_keys = ['influxdb/write', 'neo4j/write']
    read_keys = ['influxdb/read', 'neo4j/read']

    write_mean_total = 0
    write_median_total = 0
    read_mean_total = 0
    read_median_total = 0

    for col in write_keys + read_keys:
        series = interval_df[col]
        slopes = compute_slopes(times, series)
        mean_val = mean_with_zero_fill(slopes)
        median_val = median_no_fill(slopes)
        results[col + '_mean'] = mean_val
        results[col + '_median'] = median_val

        if col in write_keys:
            write_mean_total += mean_val
            write_median_total += median_val
        elif col in read_keys:
            read_mean_total += mean_val
            read_median_total += median_val

    results['write_mean'] = write_mean_total
    results['write_median'] = write_median_total
    results['read_mean'] = read_mean_total
    results['read_median'] = read_median_total

    return results

def main(times_path, io_path, out_dir):
    times_df = parse_times(times_path)
    io_df = load_io_data(io_path)

    out_trace = []

    for idx, row in times_df.iterrows():
        if row['type'] != 'TRACE':
            continue

        start = row['start_time']
        stop = row['stop_time']
        freq = row['frequency']
        npairs = row['num_pairs']

        interval_df = process_interval(io_df, start, stop)
        if interval_df.empty:
            continue

        stats = analyze_interval(interval_df)

        entry = {'frequency': freq, 'num_pairs': npairs}
        entry.update(stats)
        out_trace.append(entry)

    trace_df = pd.DataFrame(out_trace)
    trace_df = trace_df.sort_values(['frequency', 'num_pairs'])

    trace_df.to_csv(f"{out_dir}/io_usage_trace.csv", index=False)

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])
