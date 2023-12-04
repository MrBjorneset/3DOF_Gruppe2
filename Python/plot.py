import matplotlib.pyplot as plt
import pandas as pd

def plot_data(line, farge):
    df = pd.read_excel('pid_data.xlsx')
    plt.plot(df['Time'], df[line], label='Ball_pos', color = farge)
    plt.title('PID Output Over Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Output')
    plt.legend()
    plt.grid(True)
    plt.show()

plot_data('Output_X', 'blue')
plot_data('Output_Y', 'green')