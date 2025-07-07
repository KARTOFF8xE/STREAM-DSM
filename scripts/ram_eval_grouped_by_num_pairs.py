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

    output_data = {"TRACE": [], "BASE": []}

    # Gruppiere nach type und num_pairs
    grouped = times_df.groupby(["type", "num_pairs"])

    for (ttype, npairs), group in grouped:
        combined_df = pd.DataFrame()
        for _, row in group.iterrows():
            start, stop = row.start_time, row.stop_time
            interval_df = data_df[(data_df["Time"] >= start) & (data_df["Time"] <= stop)]
            combined_df = pd.concat([combined_df, interval_df], ignore_index=True)

        sub_df_clean = combined_df.drop(columns=["Time"], errors="ignore")

        means = sub_df_clean.mean(skipna=True)
        medians = sub_df_clean.median(skipna=True).astype(float).fillna(0)
        stds = sub_df_clean.std(skipna=True)
        mads = sub_df_clean.apply(compute_mad)

        out = {
            "num_pairs": npairs,
            "interval_count": len(group)
        }

        for col in sub_df_clean.columns:
            out[f"{col}_mean"] = means.get(col, 0)
            out[f"{col}_median"] = medians.get(col, 0)
            out[f"{col}_std"] = stds.get(col, 0)
            out[f"{col}_mad"] = mads.get(col, 0)

        if ttype == "TRACE":
            output_data["TRACE"].append(out)
        else:
            output_data["BASE"].append(out)

    os.makedirs(output_dir, exist_ok=True)

    pd.DataFrame(output_data["TRACE"]).sort_values("num_pairs").to_csv(
        os.path.join(output_dir, "ram_usage_aggregated_trace.csv"), index=False
    )
    pd.DataFrame(output_data["BASE"]).sort_values("num_pairs").to_csv(
        os.path.join(output_dir, "ram_usage_aggregated_base.csv"), index=False
    )

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python ram_analysis_grouped_by_type.py <times.csv> <input.csv> <output_dir>")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2], sys.argv[3])
