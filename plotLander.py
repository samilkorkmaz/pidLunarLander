import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# List of your log files
file_list = [
    'pid_lander_log_2026-03-26-07-59-26.txt',
    #'pid_lander_log_2026-03-25-12-38-34.txt',
    #'pid_lander_log_2026-03-25-12-42-13.txt'
]

def plot_lander_telemetry(files):
    # 3 subplots: Height, Velocity, and Thrust Command
    # figsize (8, 6) at 100 DPI = 800x600 pixels
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6), dpi=100, sharex=True)
    
    colors = plt.cm.viridis(np.linspace(0, 0.8, len(files)))

    for i, file_path in enumerate(files):
        try:
            df = pd.read_csv(file_path, sep='\t')
            df.columns = df.columns.str.replace('%', '').str.strip() # Clean column names, remove % and whitespace
            
            run_mode = df['mode'].iloc[0] if 'mode' in df.columns else file_path
            
            # 1. Height Plot
            ax1.plot(df['time(s)'], df['height(m)'], 
                     label=f"Mode: {run_mode}", color=colors[i], linewidth=1.5)
            
            # 2. Velocity Plot
            ax2.plot(df['time(s)'], df['velocity(m/s)'], 
                     color=colors[i], linewidth=1.5)
            
            # 3. Thrust Plot
            ax3.plot(df['time(s)'], df['thrust_commanded()'], # Note that since we removed % from column names, we use 'thrust_command()' instead of 'thrust_command(%)'
                     color=colors[i], linewidth=1.5, alpha=0.8, label="Commanded")
            ax3.plot(df['time(s)'], df['thrust_realized()'], 
             color=colors[i], linewidth=1.2, linestyle='--', alpha=0.9, label="Realized")
            
        except Exception as e:
            print(f"Error processing {file_path}: {e}")

    # Formatting Top Plot (Height)
    ax1.set_ylabel('Height (m)')
    ax1.set_title('Lander Telemetry: Height, Velocity, & Thrust')
    ax1.set_ylim(bottom=0)
    ax1.grid(True, linestyle=':', alpha=0.6)
    # 4m landing height
    ax1.axhline(4, color='red', linestyle='--', linewidth=1.2, label='Landing height (4m)')    
    ax1.legend(loc='upper right', fontsize='x-small')

    # Formatting Middle Plot (Velocity)
    ax2.set_ylabel('Velocity +up (m/s)')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.axhline(0, color='black', linewidth=0.8, alpha=0.3)
    # 5m/s limit for safe landing velocity
    ax2.axhline(-5, color='red', linestyle='--', linewidth=1.2, label='Safe Speed Limit (5m/s)')
    ax2.legend(loc='center right', fontsize='x-small')

    # Formatting Bottom Plot (Thrust)
    ax3.set_ylabel('Thrust (%)')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylim(-5, 105) # Thrust is usually 0-100%
    ax3.grid(True, linestyle=':', alpha=0.6)
    ax3.legend(loc='upper right', fontsize='x-small')

    plt.tight_layout()
    plt.show()

plot_lander_telemetry(file_list)
