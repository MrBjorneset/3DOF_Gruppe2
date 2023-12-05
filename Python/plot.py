import matplotlib.pyplot as plt
import pandas as pd

def plot_data(line, farge, label):
    df = pd.read_csv('Gen_Data/saved_data.csv')
    plt.plot(df['num'], df[line], label= label, color = farge)
    plt.title('Ball Position Over Time')
    plt.xlabel('Counter')
    plt.ylabel('Output')
    plt.legend()
    plt.grid(True)
    plt.show()

plot_data('x', 'blue', 'Ball X Position')
plot_data('y', 'green', 'Ball Y Position')
