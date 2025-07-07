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
    number = float(number.replace(",", "."))
    unit = unit.upper()
    factor = {"": 1, "K": 1024, "M": 1024**2, "G": 1024**3}.get(unit, 1)
    return number * factor

def compute_mad(series):
    s = series.dropna()
    if s.empty:
        return 0
    median = s.median()
    mad = np.median(np.abs(s - median))
    return mad

def main(times_csv, input_csv, output_dir):
    times_df = pd.read_csv(times_csv, parse_dates=["start_time", "stop_time"])
    data_df = pd.read_csv(input_csv, parse_dates=["Time"])

    for col in data_df.columns:
        if col != "Time":
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
        prop = col.split("/", 1)[1] if "/" in col else col
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
        sub_df_clean = sub_df.drop(columns=["Time"], errors="ignore")

        means = sub_df_clean.mean(skipna=True)
        medians = sub_df_clean.median(skipna=True).astype(float).fillna(0)
        stds = sub_df_clean.std(skipna=True)
        mads = sub_df_clean.apply(compute_mad)

        group_mean, group_median = {"stream": {}, "ros2": {}, "db": {}}, {"stream": {}, "ros2": {}, "db": {}}
        group_std, group_mad = {"stream": {}, "ros2": {}, "db": {}}, {"stream": {}, "ros2": {}, "db": {}}

        db_agg = {"mean": {}, "median": {}, "std": {}, "mad": {}}
        sr_agg = {"mean": {}, "median": {}, "std": {}, "mad": {}}

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
            std_val = stds.get(col, 0)
            mad_val = mads.get(col, 0)

            out[f"{col}_mean"] = 0 if pd.isna(mean_val) else mean_val
            out[f"{col}_median"] = 0 if pd.isna(median_val) else median_val
            out[f"{col}_std"] = 0 if pd.isna(std_val) else std_val
            out[f"{col}_mad"] = 0 if pd.isna(mad_val) else mad_val

            if group == "db":
                for key, val in [("mean", mean_val), ("median", median_val), ("std", std_val), ("mad", mad_val)]:
                    if not pd.isna(val):
                        db_agg[key][prop] = db_agg[key].get(prop, 0) + val
            else:
                for key, val in [("mean", mean_val), ("median", median_val), ("std", std_val), ("mad", mad_val)]:
                    if not pd.isna(val):
                        sr_agg[key][(group, prop)] = sr_agg[key].get((group, prop), 0) + val

        # Gruppierte Werte eintragen
        for agg_dict, target in [(sr_agg["mean"], group_mean), (sr_agg["median"], group_median),
                                 (sr_agg["std"], group_std), (sr_agg["mad"], group_mad)]:
            for (group, prop), val in agg_dict.items():
                target[group][prop] = val

        for agg_dict, target in [(db_agg["mean"], group_mean), (db_agg["median"], group_median),
                                 (db_agg["std"], group_std), (db_agg["mad"], group_mad)]:
            for prop, val in agg_dict.items():
                target["db"][prop] = val

        for group in ["stream", "ros2", "db"]:
            props = set(group_mean[group]) | set(group_median[group]) | set(group_std[group]) | set(group_mad[group])
            for prop in sorted(props):
                out[f"{group}/{prop}_mean"] = group_mean[group].get(prop, 0)
                out[f"{group}/{prop}_median"] = group_median[group].get(prop, 0)
                out[f"{group}/{prop}_std"] = group_std[group].get(prop, 0)
                out[f"{group}/{prop}_mad"] = group_mad[group].get(prop, 0)

        if ttype == "TRACE":
            trace_rows.append(out)
        else:
            base_rows.append(out)

    os.makedirs(output_dir, exist_ok=True)
    pd.DataFrame(trace_rows).sort_values(["frequency", "num_pairs"]).to_csv(os.path.join(output_dir, "ram_usage_trace.csv"), index=False)
    pd.DataFrame(base_rows).sort_values(["frequency", "num_pairs"]).to_csv(os.path.join(output_dir, "ram_usage_base.csv"), index=False)

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python ram_analysis.py <times.csv> <input.csv> <output_dir>")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2], sys.argv[3])
