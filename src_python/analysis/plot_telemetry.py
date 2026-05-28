import pandas as pd
import matplotlib.pyplot as plt
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description="Plot PINN Manipulator Telemetry")
    parser.add_argument('--file', type=str, default='../../build/telemetry_log.csv', 
                        help='Path to telemetry CSV file. Adjust path depending on where you run this script.')
    args = parser.parse_args()

    try:
        df = pd.read_csv(args.file)
    except FileNotFoundError:
        print(f"Error: Could not find {args.file}. Make sure you ran RoboticSim first and the path is correct.")
        sys.exit(1)

    # Plot 1: Joint Positions (Actual vs Desired)
    plt.figure(figsize=(12, 8))
    for i in range(3):
        plt.subplot(3, 1, i+1)
        plt.plot(df['time'], df[f'q_des_{i}'], '--', label=f'Desired q{i}')
        plt.plot(df['time'], df[f'q_act_{i}'], label=f'Actual q{i}')
        plt.ylabel(f'Joint {i} Pos')
        plt.legend()
        plt.grid(True)
    plt.suptitle('Joint Positions: Desired vs Actual')
    plt.xlabel('Time (s)')
    plt.tight_layout()

    # Plot 2: Control Effort (Torque/Force)
    plt.figure(figsize=(12, 8))
    for i in range(3):
        plt.subplot(3, 1, i+1)
        plt.plot(df['time'], df[f'tau_{i}'], label=f'Tau {i}', color='r')
        plt.ylabel(f'Joint {i} Effort')
        plt.legend()
        plt.grid(True)
    plt.suptitle('Control Effort (Torque/Force)')
    plt.xlabel('Time (s)')
    plt.tight_layout()

    output_file = args.file.replace('.csv', '.png')
    plt.savefig(output_file, dpi=300)
    print(f"Plot successfully saved to {output_file}")

if __name__ == "__main__":
    main()
