import sys
import pandas as pd
import numpy as np
import re

def parse_times(times_path):
    df = pd.read_csv(times_path, parse_dates=['start_time','stop_time'])
    return df

def parse_human_readable_bytes(value):
    if pd.isna(value):
        return np.nan
    if isinstance(value, (int, float)):
        return value
    value = str(value).strip()
    match = re.match(r'^([\d\.,]+)\s*([kKmMgGtT]?B)$', value)
    if not match:
        return np.nan
    num_str, unit = match.groups()
    num = float(num_str.replace(",", "."))
    unit = unit.upper()
    factor = {
        'B': 1,
        'KB': 1e3,
        'MB': 1e6,
        'GB': 1e9,
        'TB': 1e12
    }.get(unit, 1)
    return num * factor

def convert_all_units(df):
    for col in df.columns:
        if col == 'Time':
            continue
        df[col] = df[col].apply(parse_human_readable_bytes)
    return df

def load_network_data(net_path):
    df = pd.read_csv(net_path, parse_dates=['Time'])
    df = convert_all_units(df)
    return df

def compute_slopes(time_series, value_series):
    diffs = value_series.diff()
    timediffs = time_series.diff().dt.total_seconds()
    slopes = diffs / timediffs
    return slopes

def mean_no_fill(values):
    result = values.mean()
    return 0 if pd.isna(result) else result

def median_no_fill(values):
    filtered = values.dropna()
    if filtered.empty:
        return 0
    return filtered.median()

def process_interval(net_df, start, stop):
    mask = (net_df['Time'] >= start) & (net_df['Time'] <= stop)
    interval_df = net_df.loc[mask].copy()
    return interval_df

def analyze_interval(interval_df):
    results = {}
    times = interval_df['Time']

    received_total_mean = 0
    received_total_median = 0
    transmitted_total_mean = 0
    transmitted_total_median = 0

    for col in interval_df.columns:
        if col == 'Time':
            continue
        series = interval_df[col]
        slopes = compute_slopes(times, series)
        mean_val = mean_no_fill(slopes)
        median_val = median_no_fill(slopes)
        results[col + '_mean'] = mean_val
        results[col + '_median'] = median_val

        if 'receivedBytes' in col:
            received_total_mean += mean_val
            received_total_median += median_val
        elif 'transmittedBytes' in col:
            transmitted_total_mean += mean_val
            transmitted_total_median += median_val

    results['received/mean'] = received_total_mean
    results['received/median'] = received_total_median
    results['transmitted/mean'] = transmitted_total_mean
    results['transmitted/median'] = transmitted_total_median

    return results

def main(times_path, net_path, out_dir):
    times_df = parse_times(times_path)
    net_df = load_network_data(net_path)

    out_trace = []

    for idx, row in times_df.iterrows():
        if row['type'] != 'TRACE':
            continue

        start = row['start_time']
        stop = row['stop_time']
        freq = row['frequency']
        npairs = row['num_pairs']

        interval_df = process_interval(net_df, start, stop)
        stats = analyze_interval(interval_df)

        entry = {'frequency': freq, 'num_pairs': npairs}
        entry.update(stats)
        out_trace.append(entry)

    trace_df = pd.DataFrame(out_trace).sort_values(['frequency','num_pairs'])
    trace_df.to_csv(f"{out_dir}/network_usage_trace.csv", index=False)

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])
