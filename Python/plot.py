import matplotlib.pyplot as plt
import pandas as pd
def plot_data():
    df = pd.read_excel('pid_data.xlsx')
    plt.plot(df['Time'], df['Output_X'], label='Ball_pos_x')
    plt.plot(df['Time'], df['Output_Y'], label='Ball_pos_y')
    plt.title('PID Output Over Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Output')
    plt.legend()
    plt.grid(True)
    plt.show()

plot_data()