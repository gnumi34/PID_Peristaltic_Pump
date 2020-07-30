#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 26 18:47:21 2020

@author: gnumi34
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math

# Read file
try:
    file_name = sys.argv[1]
except IndexError:
    raise SystemExit(f"usage: {sys.argv[0]} csv_file")

data = pd.read_csv(file_name)

if not file_name[:3].isnumeric():
    flow_rate_sp = float(file_name[:2])
else:
    flow_rate_sp = float(file_name[:3])

# Number of samplepoints
N = len(data)
Time = data['Time (s)']
T = Time[N-1]
t_plt = np.linspace(0, T, N, endpoint=False)

# Calculate Average Flow Rate
sum = 0
for i in range(N):
    sum += data['Flow Rate (uL/min)'][i]

print(f'Average flow rate: {sum/N}')

# Calculate average deviation based on realtime flow rate
avg_min = 0
avg_max = 0
count_min = 0
count_max = 0

for i in range(N):
    if data['Flow Rate (uL/min)'][i] >= flow_rate_sp:
        avg_max += data['Flow Rate (uL/min)'][i]
        count_max += 1
    else:
        avg_min += data['Flow Rate (uL/min)'][i]
        count_min += 1

avg_max = avg_max / count_max
avg_min = avg_min / count_min

print(f'Average Max. Deviation: {avg_max} / {avg_max - flow_rate_sp}')
print(f'Average Min. Deviation: {avg_min} / {flow_rate_sp - avg_min}')
print(f'Average Deviation: +-{((avg_max - flow_rate_sp) + (flow_rate_sp - avg_min)) / 2}')

# Calculate average deviation based on moving average flow rate
avg_min = 0
avg_max = 0
count_min = 0
count_max = 0

for i in range(N):
    if data['Moving Average Flow Rate (uL/min)'][i] >= flow_rate_sp:
        avg_max += data['Moving Average Flow Rate (uL/min)'][i]
        count_max += 1
    else:
        avg_min += data['Moving Average Flow Rate (uL/min)'][i]
        count_min += 1

avg_max = avg_max / count_max
avg_min = avg_min / count_min

print(f'Average Max. Deviation (from Moving Average): {avg_max} / {avg_max - flow_rate_sp}')
print(f'Average Min. Deviation (from Moving Average): {avg_min} / {flow_rate_sp - avg_min}')
print(f'Average Deviation (from Moving Average): +-{((avg_max - flow_rate_sp) + (flow_rate_sp - avg_min)) / 2}')

# Calculate rise time based on average flow rate
max_time = 0
min_time = 0

for i in range(N):
    if (data['Time (s)'][i] >= 0.2) and (data['Average Flow Rate (uL/min)'][i] >= (0.9 * flow_rate_sp)):
        max_time = data['Time (s)'][i]
        print(f'max_time = {max_time}')
        break

for i in range(N):
    if (data['Time (s)'][i] >= 0.2) and (data['Average Flow Rate (uL/min)'][i] >= (0.1 * flow_rate_sp)):
        min_time = data['Time (s)'][i]
        print(f'max_time = {min_time}')
        break

print(f'Rise time = {max_time - min_time}')

# Plot data
fig, ax = plt.subplots(2,1)
ax[0].set_title("Flow Rate Measurement")
ax[0].plot(t_plt, data['Flow Rate (uL/min)'], 'r-', label='Measured Flow')

try:
    ax[0].plot(t_plt, data['Moving Average Flow Rate (uL/min)'], 'b-', label='Moving Average Flow')
except KeyError:
    pass

try:
    ax[0].plot(t_plt, data['Average Flow Rate (uL/min)'], 'g-', label='Instantenous Average Flow Rate')
except KeyError:
    pass

ax[0].set_xlabel('Time [sec]')
ax[0].set_ylabel('Flow Rate [uL/min]')
ax[0].grid(True)
ax[0].legend()

ax[1].set_title("Control Measurement")
ax[1].plot(t_plt, data['Error'], 'g-', label='Error')
ax[1].plot(t_plt, data['Frequency'], 'b-', label='Motor Step Frequency')
ax[1].set_xlabel('Time [sec]')
ax[1].set_ylabel('Frequency [Hz]/Flow Rate')
ax[1].grid(True)
ax[1].legend()
fig.tight_layout()

plt.show()