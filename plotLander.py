import pandas as pd
import matplotlib.pyplot as plt

# Load the data
file_path = 'pid_lander_log_2026-03-24-17-24-42.txt'
df = pd.read_csv(file_path, sep='\t')

# Clean column names
df.columns = df.columns.str.replace('%', '')

plt.figure(figsize=(10, 6))
plt.plot(df['time(s)'], df['height(m)'], label='Lander Height', color='blue')

# --- SET Y-AXIS MINIMUM TO 0 ---
# The first argument is the bottom (min), the second is the top (max)
# Using None for the top allows matplotlib to auto-scale the maximum height
plt.ylim(bottom=0) 

plt.title('Lander Flight Profile: Time vs Height')
plt.xlabel('Time (s)')
plt.ylabel('Height (m)')
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend()

plt.show()