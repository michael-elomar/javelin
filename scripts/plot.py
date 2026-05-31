import pandas as pd
import numpy as np
import struct
from datetime import datetime, timedelta
from collections import defaultdict


def read_log_file(file_path: str) -> dict:
    """
    Reads a binary telemetry log file and returns a dictionary where:
    - Keys: Field names (e.g., "altitude", "latitude")
    - Values: Dict with 'timestamps' and 'values' arrays.

    Binary format:
    - Timestamp (8B, uint64): Microseconds since epoch
    - Field name length (1B, uint8)
    - Field name (N B, ASCII)
    - Value (8B, double)

    Returns:
        {
            "altitude": {
                "timestamps": [datetime1, datetime2, ...],
                "values": [1200.5, 1201.0, ...]
            },
            "latitude": {
                "timestamps": [...],
                "values": [...]
            },
            ...
        }
    """

    d = {}

    with open(file_path, "rb") as f:
        while True:
            # Read timestamp (8B)
            timestamp_bytes = f.read(8)
            if not timestamp_bytes:
                break
            timestamp = struct.unpack("<d", timestamp_bytes)[0]  # Little-endian double

            # Read field name length (1B)
            field_len_bytes = f.read(1)
            if not field_len_bytes:
                break
            field_len = struct.unpack("<B", field_len_bytes)[0]

            # Read field name (N B)
            field_name_bytes = f.read(field_len)
            if len(field_name_bytes) != field_len:
                break
            field_name = field_name_bytes.decode("ascii")

            # Read value (8B)
            value_bytes = f.read(8)
            if not value_bytes:
                break
            value = struct.unpack("<d", value_bytes)[0]

            if field_name not in d:
                d[field_name] = [value]
            else:
                d[field_name].append(value)

    return d


def plot_field_arrays(df: dict):
    """
    Plots each field's values over time using matplotlib.
    """
    import matplotlib.pyplot as plt

    for field in df.keys():
        plt.figure(figsize=(10, 5))
        values = df[field]

        plt.plot(values, label=field, linestyle="-")

        plt.grid()
        # Plot paths
        # Add legend and axes labels
        plt.legend(loc=0)
        plt.xlabel(r"$t$")
        plt.grid(visible=True, which="major", color="k", linestyle="-", alpha=0.65)
        plt.grid(visible=True, which="minor", linestyle="--")
        plt.minorticks_on()
    plt.show()


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("Usage: python binary_telemetry_to_arrays.py <binary_file.bin>")
        sys.exit(1)

    binary_file = sys.argv[1]

    try:
        # Read binary file and group by field
        df = read_log_file(binary_file)
        plot_field_arrays(df)

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
