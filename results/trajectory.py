import os
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot(measured_filename, truth_filename, drone_filename):

    # Ler CSV
    measured_df = pd.read_csv(measured_filename)
    truth_df = pd.read_csv(truth_filename)
    drone_df = pd.read_csv(drone_filename)

    stddev = drone_df.replace("DRONE__","").replace(".csv","")

    # Filter data for the current param value

    # 3D Plot using DRONE and BASE_TRUTH
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(drone_df['X'], drone_df['Y'], abs(drone_df['Z']), c='r', label='DRONE')
    ax.scatter(truth_df['X'], truth_df['Y'], truth_df['Z'], c='b', label='BASE_TRUTH')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'3D Plot for stddev = {stddev}')
    ax.legend()

    # 2D Plot using only columns X and Y of BASE_TRUTH and BASE_MEASURED
    plt.figure()
    plt.scatter(truth_df['X'], truth_df['Y'], c='b', label='BASE_TRUTH')
    plt.scatter(measured_df['X'], measured_df['Y'], c='g', label='BASE_MEASURED')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(f'2D Plot for stddev = {stddev}')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    # Replace the directory path with the location of your CSV files
    directory = 'results/data/landing/'


    # Plot for each value of "param"
    #for param_value in param_values:
    #    plot_3d_and_2d(drone_df, base_truth_df, base_measured_df, param_value)


    for measured, truth, drone in zip(sorted(os.listdir(directory+'measured')),
                                      sorted(os.listdir(directory+'truth')),
                                      sorted(os.listdir(directory+'drone'))):
        measured_file = os.path.join(directory+'measured',measured)
        truth_file = os.path.join(directory+'truth',truth)
        drone_file = os.path.join(directory+'drone',drone)
        plot(measured_file,truth_file,drone_file)
