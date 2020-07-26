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

try:
    file_name = sys.argv[1]
except IndexError:
    raise SystemExit(f"usage: {sys.argv[0]} csv_file")

data = pd.read_csv(file_name)

# Number of samplepoints
N = len(data)
Time = data['Time (s)']
T = Time[N-1]
t_plt = np.linspace(0, T, N, endpoint=False)

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

# plt.subplots_adjust(hspace=0.35)
plt.show()

sum = 0
for i in range(N):
    sum += data['Flow Rate (uL/min)'][i]

print(f'mean flow rate: {sum/N}')