##############################################################################################################
#KX134

import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
df = pd.read_csv('c:/Users/dines/Desktop/AAX_SensorData_20250419/ax_12800Hz_2s_20250419_020054.csv')

# Combine date and time into a single datetime column
df['datetime'] = pd.to_datetime(df['date'] + ' ' + df['time'])

# Set datetime as the index (optional but useful for plotting)
df.set_index('datetime', inplace=True)

# Plot accelerometer data
plt.figure(figsize=(12, 6))
plt.plot(df.index, df['ax'], label='ax')
plt.plot(df.index, df['ay'], label='ay')
plt.plot(df.index, df['az'], label='az')

plt.xlabel('Time')
plt.ylabel('Acceleration')
plt.title('Accelerometer Data Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

##############################################################################################################

##############################################################################################################
# IMU

# import pandas as pd
# import matplotlib.pyplot as plt

# # Load the CSV file
# df = pd.read_csv('c:/Users/dines/Desktop/AAX_SensorData_20250419/ax_2000Hz_6s_20250419_015303.csv')

# # Combine date and time into a datetime column
# df['datetime'] = pd.to_datetime(df['date'] + ' ' + df['time'])

# # Set datetime as the index
# df.set_index('datetime', inplace=True)

# # Create subplots for accelerometer and gyroscope
# fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# # Plot accelerometer data
# axs[0].plot(df.index, df['ax'], label='ax')
# axs[0].plot(df.index, df['ay'], label='ay')
# axs[0].plot(df.index, df['az'], label='az')
# axs[0].set_ylabel('Acceleration')
# axs[0].set_title('Accelerometer Data')
# axs[0].legend()
# axs[0].grid(True)

# # Plot gyroscope data
# axs[1].plot(df.index, df['gx'], label='gx')
# axs[1].plot(df.index, df['gy'], label='gy')
# axs[1].plot(df.index, df['gz'], label='gz')
# axs[1].set_ylabel('Angular Velocity')
# axs[1].set_title('Gyroscope Data')
# axs[1].legend()
# axs[1].grid(True)

# plt.xlabel('Time')
# plt.tight_layout()
# plt.show()

##############################################################################################################

##############################################################################################################
# Audio

# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# # Load the CSV file
# df = pd.read_csv('c:/Users/dines/Desktop/AAX_SensorData_20250419/audio_16000Hz_6s_20250419_014742.csv')

# # Extract the audio samples
# audio = df['audio'].values

# # Sampling rate in Hz
# sample_rate = 16000

# # Create time axis
# time = np.arange(len(audio)) / sample_rate

# # Plot the waveform
# plt.figure(figsize=(12, 4))
# plt.plot(time, audio, linewidth=0.8)
# plt.xlabel('Time (s)')
# plt.ylabel('Amplitude')
# plt.title('PDM Audio Waveform (16 kHz)')
# plt.grid(True)
# plt.tight_layout()
# plt.show()

##############################################################################################################
