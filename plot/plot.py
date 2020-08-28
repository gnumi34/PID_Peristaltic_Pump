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

print(f"File name: {file_name}")
print(f"Set-point = {flow_rate_sp} uL/min")
print(f"Time elapsed = {T} s")
print("Processing data.....\n")



# Calculate Average Flow Rate
print("Calculating average flow rate based on realtime flow rate measurement..")
sum = 0
for i in range(N):
    sum += data['Flow Rate (uL/min)'][i]

average_fr = sum/N

print(f'Average flow rate: {average_fr} uL/m\n')

# Calculate average deviation based on realtime flow rate
print("Calculating average flow rate deviation based on realtime flow rate..")
avg_min = 0
avg_max = 0
count_min = 0
count_max = 0

for i in range(N):
    if data['Flow Rate (uL/min)'][i] > flow_rate_sp:
        avg_max += data['Flow Rate (uL/min)'][i]
        count_max += 1
    elif data['Flow Rate (uL/min)'][i] < flow_rate_sp:
        avg_min += data['Flow Rate (uL/min)'][i]
        count_min += 1

avg_max = avg_max / count_max
avg_min = avg_min / count_min

print(f'Average Max. Deviation: {avg_max} uL/m / {avg_max - flow_rate_sp} uL/m')
print(f'Average Min. Deviation: {avg_min} uL/m / {flow_rate_sp - avg_min} uL/m')
print(f'Average Deviation: +-{((avg_max - flow_rate_sp) + (flow_rate_sp - avg_min)) / 2} uL/m\n')

# Calculate average deviation based on moving average flow rate
print("Calculating average flow rate deviation based on moving average flow rate..")
avg_min = 0
avg_max = 0
count_min = 0
count_max = 0

if 'Moving Average Flow Rate (uL/min)' in data:
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
    print(f'Average Deviation (from Moving Average): +-{((avg_max - flow_rate_sp) + (flow_rate_sp - avg_min)) / 2}\n')
else:
    print("Moving average flow rate data is not found. Skipping..\n")

# Calculate rise time based on average flow rate
print("Calculating rise time of system (10% of set-point to 90% of set-point)..")
max_time = 0
min_time = 0

if 'Average Flow Rate (uL/min)' in data:
    for i in range(N):
        if (data['Time (s)'][i] >= 0.2) and (data['Average Flow Rate (uL/min)'][i] >= (0.9 * flow_rate_sp)):
            max_time = data['Time (s)'][i]
            print(f'max_time = {max_time} s')
            break

    for i in range(N):
        if (data['Time (s)'][i] >= 0.2) and (data['Average Flow Rate (uL/min)'][i] >= (0.1 * flow_rate_sp)):
            min_time = data['Time (s)'][i]
            print(f'min_time = {min_time} s')
            break
    if (max_time):
        print(f'Rise time = {max_time - min_time} s\n')
    else:
        print("Average flow rate doesn't rise into 90% set-point, cannot calculate rise time.\n")
else:
    print("Realtime Average Flow Rate data is not found. Skipping..\n")

# Calculate dispensed volume using real-time flow rate measurement
print("Calculating dispensed volume using realtime flow rate measurement..")
totalizer = 0.0
last_flow = 0.0
last_time = 0.0
vol_dispensed = []
for i in range(N):
    totalizer += (last_flow + data['Flow Rate (uL/min)'][i]) / 2.0 * (data['Time (s)'][i] - last_time) / 60.0
    vol_dispensed.append(totalizer)
    last_flow = data['Flow Rate (uL/min)'][i]
    last_time = data['Time (s)'][i]

print(f"Dispensed volume in real-time = {totalizer:.5f} uL\n")

# Calculate dispensed volume using average flow rate
print("Calculating dispensed volume using realtime average flow rate measurement..")
totalizer_avg = 0.0
last_time = 0.0
for i in range(N):
    totalizer_avg += average_fr / 60.0 * (data['Time (s)'][i] - last_time)
    last_time = data['Time (s)'][i]

print(f"Dispensed volume in average = {totalizer_avg:.5f} uL\n")

# Calculate dispensed volume using realtime average flow rate
print("Calculating dispensed volume using realtime average flow rate measurement..")
totalizer_avg = 0.0
last_time = 0.0
if 'Average Flow Rate (uL/min)' in data:
    for i in range(N):
        totalizer_avg += data['Average Flow Rate (uL/min)'][i] / 60.0 * (data['Time (s)'][i] - last_time)
        last_time = data['Time (s)'][i]
else:
    print("Realtime average flow rate data is not found.")

print(f"Dispensed volume in real time average = {totalizer_avg:.5f} uL\n")

# Plot data
print("Plotting data..")
if int(sys.argv[2]) == 1:
    fig, ax = plt.subplots(2,1)
    ax[0].set_title("Flow Rate Measurement")
    ax[0].plot(t_plt, data['Flow Rate (uL/min)'], 'r-', label='Measured Flow Rate')

    if 'Moving Average Flow Rate (uL/min)' in data:
        ax[0].plot(t_plt, data['Moving Average Flow Rate (uL/min)'], 'b-', label='Moving Average Flow Rate')

    if 'Average Flow Rate (uL/min)' in data:
        ax[0].plot(t_plt, data['Average Flow Rate (uL/min)'], 'g-', label='Instantenous Average Flow Rate')

    ax[0].set_xlabel('Time [sec]')
    ax[0].set_ylabel('Flow Rate [uL/min]')
    ax[0].grid(True)
    ax[0].legend()

    ax[1].set_title("Dispensed Volume")
    if "Volume Dispensed (uL)" in data:
        ax[1].plot(t_plt, data["Volume Dispensed (uL)"], 'b-', label='Volume Dispensed')
    else:
        ax[1].plot(t_plt, vol_dispensed, 'b-', label='Volume Dispensed')

    if "test_vol" in data:
        ax[1].plot(t_plt, data["test_vol"], 'r-', label='Volume Dispensed (from average)')

    ax[1].set_xlabel('Time [sec]')
    ax[1].set_ylabel('Volume [uL]')
    ax[1].grid(True)
    ax[1].legend()

    fig.tight_layout()

    plt.show()
elif int(sys.argv[2]) == 2:
    fig, ax = plt.subplots(1,1)
    ax[0].set_title("Flow Rate Measurement")
    ax[0].plot(t_plt, data['Flow Rate (uL/min)'], 'r-', label='Measured Flow Rate')

    if 'Moving Average Flow Rate (uL/min)' in data:
        ax[0].plot(t_plt, data['Moving Average Flow Rate (uL/min)'], 'b-', label='Moving Average Flow Rate')

    if 'Average Flow Rate (uL/min)' in data:
        ax[0].plot(t_plt, data['Average Flow Rate (uL/min)'], 'g-', label='Instantenous Average Flow Rate')

    ax[0].set_xlabel('Time [sec]')
    ax[0].set_ylabel('Flow Rate [uL/min]')
    ax[0].grid(True)
    ax[0].legend()

    fig.tight_layout()

    plt.show()
elif int(sys.argv[2]) == 3:
    fig, ax = plt.subplots(3,1)
    ax[0].set_title("Flow Rate Measurement")
    ax[0].plot(t_plt, data['Flow Rate (uL/min)'], 'r-', label='Realtime Flow Rate')

    if 'Moving Average Flow Rate (uL/min)' in data:
        ax[0].plot(t_plt, data['Moving Average Flow Rate (uL/min)'], 'b-', label='Moving Average Flow Rate')

    if 'Average Flow Rate (uL/min)' in data:
        ax[0].plot(t_plt, data['Average Flow Rate (uL/min)'], 'g-', label='Instantenous Average Flow Rate')

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

    ax[2].set_title("Dispensed Volume")
    if "Volume Dispensed (uL)" in data:
        ax[2].plot(t_plt, data["Volume Dispensed (uL)"], 'b-', label='Volume Dispensed')
    else:
        ax[2].plot(t_plt, vol_dispensed, 'b-', label='Volume Dispensed')

    if "test_vol" in data:
        ax[2].plot(t_plt, data["test_vol"], 'r-', label='Volume Dispensed (from average)')

    ax[2].set_xlabel('Time [sec]')
    ax[2].set_ylabel('Volume [uL]')
    ax[2].grid(True)
    ax[2].legend()

    fig.tight_layout()

    plt.show()
print("Data processing is completed. Exiting..")