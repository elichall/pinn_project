import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import sys
import os
import subprocess
import tempfile
from datetime import datetime
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(
        description="Process and plot PINN Manipulator Telemetry"
    )
    parser.add_argument(
        "-f",
        "--file",
        type=str,
        nargs="?",
        const="USE_FILEMANAGER",
        default=None,
        help="Path to specific telemetry CSV file. Pass without args to use the terminal file chooser.",
    )
    args = parser.parse_args()

    # Resolve training_data relative to this script's location
    training_data_dir = Path(__file__).resolve().parent.parent.parent / "training_data"

    if args.file is None:
        if not training_data_dir.exists():
            print(f"Error: Directory {training_data_dir} does not exist.")
            sys.exit(1)

        csv_files = list(training_data_dir.glob("*_telemetry_log.csv"))
        if not csv_files:
            print(f"Error: No telemetry CSV files found in {training_data_dir}.")
            sys.exit(1)

        # The filename format YYYYMMDD_HHMM sorts chronologically naturally
        input_path = sorted(csv_files)[-1]
        print(f"Auto-selected most recent file: {input_path}")

    elif args.file == "USE_FILEMANAGER":
        if not training_data_dir.exists():
            print(f"Error: Directory {training_data_dir} does not exist.")
            sys.exit(1)

        filemanager = os.environ.get("FILEMANAGER", "yazi")
        with tempfile.NamedTemporaryFile() as tmp:
            try:
                subprocess.run(
                    [filemanager, str(training_data_dir), "--chooser-file", tmp.name],
                    check=True,
                )
            except FileNotFoundError:
                print(
                    f"Error: File manager '{filemanager}' not found. Please ensure it is installed and in your PATH."
                )
                sys.exit(1)
            except subprocess.CalledProcessError:
                # yazi might return non-zero if cancelled
                pass

            # Read the selected file path
            tmp.seek(0)
            chosen_path = tmp.read().decode("utf-8").strip()

        if not chosen_path:
            print("No file selected. Exiting.")
            sys.exit(0)

        input_path = Path(chosen_path)
        print(f"Selected file: {input_path}")

    else:
        input_path = Path(args.file)

    try:
        df = pd.read_csv(input_path)
    except FileNotFoundError:
        print(
            f"Error: Could not find {input_path}. Make sure you ran RoboticSim first and the path is correct."
        )
        sys.exit(1)

    # 1. Generate Timestamp and Base Output Path
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # input_path.parent gets the directory (e.g. 'training_data')
    # input_path.stem gets the filename without extension (e.g. '20260530_1006_telemetry_log')
    target_dir = input_path.parent.parent / "logs"
    target_dir.mkdir(parents=True, exist_ok=True)
    out_prefix = target_dir / f"{input_path.stem}"

    # 2. Open a text file for our logging output
    txt_output_path = f"{out_prefix}_report.txt"
    report_file = open(txt_output_path, "w")

    # Helper function to print exclusively to the text file to prevent console clutter
    def log_print(message=""):
        print(message, file=report_file)  # Prints ONLY to our opened txt file

    log_print(f"--- Analysis Report generated at {timestamp} ---\n")

    # Calculate dt for integrals
    dt = df["sysTime"].diff().fillna(0)

    # --- Plot Joint Positions (Desired vs Actual) ---
    plt.figure(figsize=(12, 10))
    for i in range(1, 4):
        plt.subplot(3, 1, i)
        plt.plot(df["sysTime"], df[f"dq{i}"], "--", label=f"Desired q{i}")
        plt.plot(df["sysTime"], df[f"q{i}"], label=f"Actual q{i}")
        plt.ylabel(f"Joint {i} Pos (rad)")
        plt.legend()
        plt.grid(True)
    plt.suptitle("Joint Positions: Desired vs Actual")
    plt.xlabel("Time (s)")
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_positions.png", dpi=300)

    # --- Control Effort (Integral of squared torque) ---
    total_effort = 0
    plt.figure(figsize=(12, 10))
    for i in range(1, 4):
        plt.subplot(3, 1, i)
        plt.plot(df["sysTime"], df[f"tau{i}"], label=f"Tau {i}", color="r")
        plt.ylabel(f"Joint {i} Effort (Nm)")
        plt.legend()
        plt.grid(True)
        # Calculate effort: integral of tau^2 dt
        effort_i = np.sum((df[f"tau{i}"] ** 2) * dt)
        total_effort += effort_i
        log_print(f"Joint {i} Control Effort (Integral of tau^2 dt): {effort_i:.4f}")

    log_print(f"Total Control Effort: {total_effort:.4f}")
    plt.suptitle("Control Effort (Torque/Force)")
    plt.xlabel("Time (s)")
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_torques.png", dpi=300)

    # --- RMS Tracking Error ---
    log_print("\n--- RMS Tracking Error ---")
    total_rms = 0
    for i in range(1, 4):
        error_sq = (df[f"q{i}"] - df[f"dq{i}"]) ** 2
        rms = np.sqrt(error_sq.mean())
        total_rms += rms
        log_print(f"Joint {i} RMS Error: {rms:.6f} rad")
    log_print(f"Total RMS Error: {total_rms:.6f} rad")

    # --- Real-Time Determinism Metrics ---
    log_print("\n--- Real-Time Determinism Metrics ---")

    # Convert from nanoseconds to microseconds for readable math
    wake_us = df["wakeJitter"] / 1000.0
    exec_us = df["executionTime"] / 1000.0

    # Calculate advanced statistics
    metrics = {"Wake Jitter": wake_us, "Execution Time": exec_us}

    for name, data in metrics.items():
        log_print(f"[{name}]")
        log_print(f"  Mean:       {data.mean():.2f} µs")
        log_print(f"  Max:        {data.max():.2f} µs")
        log_print(f"  Std Dev:    {data.std():.2f} µs")
        log_print(f"  99th Pctl:  {np.percentile(data, 99):.2f} µs")
        log_print(f"  99.9th Pctl:{np.percentile(data, 99.9):.2f} µs")

    # --- Time Keeping Histogram (Logarithmic) ---
    plt.figure(figsize=(12, 6))

    # Subplot 1: Wake Jitter
    plt.subplot(1, 2, 1)
    # rwidth=0.85 separates the pillars. edgecolor='black' defines their borders.
    # log=True forces the Y-axis to scale logarithmically so outlier tails are visible.
    plt.hist(
        wake_us,
        bins=50,
        color="#3498db",
        alpha=0.8,
        rwidth=0.85,
        edgecolor="black",
        log=True,
    )
    plt.title("Wake Jitter (Log Scale)")
    plt.xlabel("Jitter (microseconds)")
    plt.ylabel("Frequency (Log)")
    plt.grid(
        True, which="both", ls="--", alpha=0.3
    )  # Adds faint gridlines for log scale

    # Subplot 2: Execution Time
    plt.subplot(1, 2, 2)
    plt.hist(
        exec_us,
        bins=50,
        color="#2ecc71",
        alpha=0.8,
        rwidth=0.85,
        edgecolor="black",
        log=True,
    )
    plt.title("Execution Time (Log Scale)")
    plt.xlabel("Execution Time (microseconds)")
    plt.ylabel("Frequency (Log)")
    plt.grid(True, which="both", ls="--", alpha=0.3)

    plt.suptitle("Real-Time Loop Determinism")
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_timing.png", dpi=300)

    print(f"Artifacts successfully saved with prefix: {out_prefix}")
    log_print(f"Artifacts successfully saved with prefix: {out_prefix}")

    # Close the text file
    report_file.close()


if __name__ == "__main__":
    main()
