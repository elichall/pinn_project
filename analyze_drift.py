import pandas as pd
import numpy as np

# load the data
df = pd.read_csv('build/telemetry_log.csv')
print(f"Loaded {len(df)} rows.")

# time col
t = df['sysTime']

# qDes cols
q3_des = df['qDes_2'] # Assuming 0-indexed, Joint 3 is index 2
q2_des = df['qDes_1']
q1_des = df['qDes_0']

print("Initial Desired State:")
print(f"q1: {q1_des.iloc[0]:.2f}, q2: {q2_des.iloc[0]:.2f}, q3: {q3_des.iloc[0]:.2f}")

print("Final Desired State:")
print(f"q1: {q1_des.iloc[-1]:.2f}, q2: {q2_des.iloc[-1]:.2f}, q3: {q3_des.iloc[-1]:.2f}")

