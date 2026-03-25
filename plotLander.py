import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# List of your log files
file_list = [
    'pid_lander_log_2026-03-25-09-19-02.txt',
    'pid_lander_log_2026-03-25-09-20-44.txt'
    # Add more log files here
]

def plot_lander_by_mode(files):
    plt.figure(figsize=(12, 7))
    
    # Generate distinct colors for n files
    colors = plt.cm.plasma(np.linspace(0, 0.8, len(files)))

    for i, file_path in enumerate(files):
        try:
            # Read file (tab separated)
            df = pd.read_csv(file_path, sep='\t')
            
            # Clean column names (remove '%' and whitespace)
            df.columns = df.columns.str.replace('%', '').str.strip()
            
            # 1. Extract the mode from the data
            # We take the first value in the 'mode' column to use as the label
            run_mode = df['mode'].iloc[0] if 'mode' in df.columns else f"Unknown ({file_path})"
            
            # 2. Plot using the mode as the label
            plt.plot(
                df['time(s)'], 
                df['height(m)'], 
                label=f"{run_mode}", 
                color=colors[i], 
                linewidth=2,
                alpha=0.8
            )
            
        except FileNotFoundError:
            print(f"File {file_path} not found.")
        except Exception as e:
            print(f"Error processing {file_path}: {e}")

    # Plot styling
    plt.ylim(bottom=0)
    plt.title('Lander Flight Profile: Height Comparison by Control Mode')
    plt.xlabel('Time (s)')
    plt.ylabel('Height (m)')
    plt.grid(True, linestyle='--', alpha=0.5)
    
    # Show legend with the Modes
    plt.legend(loc='best')
    plt.tight_layout()
    
    # Save or show the plot
    plt.savefig('lander_mode_comparison.png')
    plt.show()

# Run the plotting function
plot_lander_by_mode(file_list)
