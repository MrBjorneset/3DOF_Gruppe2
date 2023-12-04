import matplotlib.pyplot as plt
import pandas as pd

def plot_data(line, farge):
    df = pd.read_csv('Gen_Data/saved_data.csv')
    plt.plot(df['num'], df[line], label='Ball_pos', color = farge)
    plt.title('PID Output Over Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Output')
    plt.legend()
    plt.grid(True)
    plt.show()

plot_data('PID_X', 'blue')
plot_data('PID_Y', 'green')
