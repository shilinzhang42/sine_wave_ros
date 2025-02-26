import pandas as pd
import os
import matplotlib.pyplot as plt

# path to the csv file
file_path = "/home/ws/src/sine_wave_ros/sine_wave_logs/sine_wave_data.csv"

# read the csv file
try:
    df = pd.read_csv(file_path)

    # plot the sine wave
    plt.figure(figsize=(10, 5))
    plt.plot(df["Time (s)"], df["Sine Value"], label="Sine Wave", color="b")
    plt.xlabel("Time (s)")
    plt.ylabel("Sine Value")
    plt.title("Sine Wave")
    plt.grid(True)
    plt.legend()
    # create directory and save the plot
    output_file_path = "/home/ws/src/sine_wave_ros/plot/sine_wave_plot.png"
    if not os.path.exists(os.path.dirname(output_file_path)):
        os.makedirs(os.path.dirname(output_file_path))

    plt.draw()
    plt.savefig(output_file_path)
    plt.show()
    plt.close()

except FileNotFoundError:
    print(f"Error: File not found at {file_path}")
