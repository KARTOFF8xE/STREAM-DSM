import sys
import os
import pandas as pd
import numpy as np
import re

def convert_to_bytes(val):
    if pd.isna(val):
        return np.nan
    if isinstance(val, (int, float)):
        return val
    val = val.strip()
    match = re.match(r"^([\d.,]+)\s*([kKmMgG]?)B?$", val)
    if not match:
        try:
            return float(val)
        except:
            return np.nan
    number, unit = match.groups()
    number = float(number.replace(",","."))
    unit = unit.upper()
    factor = 1
    if unit == "K":
        factor = 1024
    elif unit == "M":
        factor = 1024**2
    elif unit == "G":
        factor = 1024**3
    return number * factor

def main(times_csv, input_csv, output_dir):
    times_df = pd.read_csv(times_csv, parse_dates=["start_time", "stop_time"])
    data_df = pd.read_csv(input_csv, parse_dates=["Time"])

    for col in data_df.columns:
        if col == "Time":
            continue
        data_df[col] = data_df[col].apply(convert_to_bytes)

    col_groups = {}
    col_props = {}
    for col in data_df.columns:
        if col == "Time":
            continue
        if col.startswith("datamgmt/") or col.startswith("continuous/") or col.startswith("structural/"):
            col_groups[col] = "stream"
        elif col.startswith("ros2talker/") or col.startswith("ros2listener/"):
            col_groups[col] = "ros2"
        elif col.startswith("neo4j/") or col.startswith("influxdb/"):
            col_groups[col] = "db"
        else:
            col_groups[col] = None
        prop = col.split("/",1)[1] if "/" in col else col
        col_props[col] = prop

    trace_rows = []
    base_rows = []

    for _, row in times_df.iterrows():
        freq = row.frequency
        npairs = row.num_pairs
        ttype = row.type
        start = row.start_time
        stop = row.stop_time

        sub_df = data_df[(data_df["Time"] >= start) & (data_df["Time"] <= stop)]

        means = sub_df.mean(skipna=True)
        medians = sub_df.median(skipna=True)
        medians = medians.fillna(0)

        group_mean = {"stream": {}, "ros2": {}, "db": {}}
        group_median = {"stream": {}, "ros2": {}, "db": {}}

        db_prop_vals_mean = {}
        db_prop_vals_median = {}

        stream_ros2_mean = {}
        stream_ros2_median = {}

        out = {"frequency": freq, "num_pairs": npairs}

        for col in data_df.columns:
            if col == "Time":
                continue
            group = col_groups.get(col)
            prop = col_props.get(col)
            if group is None:
                continue
            mean_val = means.get(col, 0)
            median_val = medians.get(col, 0)

            out[f"{col}_mean"] = mean_val if not pd.isna(mean_val) else 0
            out[f"{col}_median"] = median_val if median_val is not None else 0

            if group == "db":
                if prop not in db_prop_vals_mean:
                    db_prop_vals_mean[prop] = 0
                    db_prop_vals_median[prop] = 0
                if not pd.isna(mean_val):
                    db_prop_vals_mean[prop] += mean_val
                if median_val is not None and not pd.isna(median_val):
                    db_prop_vals_median[prop] += median_val
            else:
                key_mean = (group, prop)
                stream_ros2_mean[key_mean] = stream_ros2_mean.get(key_mean, 0) + (mean_val if not pd.isna(mean_val) else 0)
                stream_ros2_median[key_mean] = stream_ros2_median.get(key_mean, 0) + (median_val if median_val is not None else 0)

        for (group, prop), val in stream_ros2_mean.items():
            group_mean[group][prop] = val
        for (group, prop), val in stream_ros2_median.items():
            group_median[group][prop] = val
        for prop in db_prop_vals_mean:
            group_mean["db"][prop] = db_prop_vals_mean[prop]
        for prop in db_prop_vals_median:
            group_median["db"][prop] = db_prop_vals_median[prop]

        for group in ["stream", "ros2", "db"]:
            for prop in sorted(set(list(group_mean[group].keys()) + list(group_median[group].keys()))):
                mmean = group_mean[group].get(prop, 0)
                mmedian = group_median[group].get(prop, 0)
                out[f"{group}/{prop}_mean"] = mmean
                out[f"{group}/{prop}_median"] = mmedian

        if ttype == "TRACE":
            trace_rows.append(out)
        else:
            base_rows.append(out)

    trace_df = pd.DataFrame(trace_rows).sort_values(["frequency", "num_pairs"]).reset_index(drop=True)
    base_df = pd.DataFrame(base_rows).sort_values(["frequency", "num_pairs"]).reset_index(drop=True)

    os.makedirs(output_dir, exist_ok=True)
    trace_df.to_csv(os.path.join(output_dir, "ram_usage_trace.csv"), index=False)
    base_df.to_csv(os.path.join(output_dir, "ram_usage_base.csv"), index=False)

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python ram_analysis.py <times.csv> <input.csv> <output_dir>")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2], sys.argv[3])
