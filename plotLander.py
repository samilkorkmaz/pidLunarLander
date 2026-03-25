import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# List of your log files
file_list = [
    'pid_lander_log_2026-03-25-09-19-02.txt',
    'pid_lander_log_2026-03-25-09-20-44.txt'
    # You can add more files here
]

def plot_lander_telemetry(files):
    # Create a figure with 2 subplots (2 rows, 1 column)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    
    # Generate distinct colors
    colors = plt.cm.plasma(np.linspace(0, 0.8, len(files)))

    for i, file_path in enumerate(files):
        try:
            # Load and clean data
            df = pd.read_csv(file_path, sep='\t')
            df.columns = df.columns.str.replace('%', '').str.strip()
            
            # Extract the mode for the legend
            run_mode = df['mode'].iloc[0] if 'mode' in df.columns else file_path
            
            # --- Subplot 1: Height ---
            ax1.plot(df['time(s)'], df['height(m)'], 
                     label=f"Mode: {run_mode}", color=colors[i], linewidth=2)
            
            # --- Subplot 2: Speed ---
            ax2.plot(df['time(s)'], df['speed(m/s)'], 
                     color=colors[i], linewidth=2, linestyle='--')
            
        except Exception as e:
            print(f"Error processing {file_path}: {e}")

    # Styling Top Plot (Height)
    ax1.set_ylabel('Height (m)')
    ax1.set_title('Lander Flight Profile: Height & Speed Comparison')
    ax1.set_ylim(bottom=0)
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='upper right')

    # Styling Bottom Plot (Speed)
    ax2.set_ylabel('Speed (m/s)')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True, linestyle='--', alpha=0.5)
    
    # Optional: Add a horizontal line at 0 speed to see hover/touchdown clearly
    ax2.axhline(0, color='black', linewidth=1, alpha=0.3)

    plt.tight_layout()
    plt.show()

# Run the updated plotting function
plot_lander_telemetry(file_list)
