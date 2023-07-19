import os
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot(measured_filename, truth_filename, drone_filename):

    # Ler CSV
    measured_df = pd.read_csv(measured_filename)
    truth_df = pd.read_csv(truth_filename)
    drone_df = pd.read_csv(drone_filename)

    _,stddev= drone_filename.split("DRONE__")
    stddev = stddev.replace(".csv","")

    file1 = f'results/plot/landing/stddev_{stddev}_trajectory.png'
    file2 = f'results/plot/landing/stddev_{stddev}_base.png'

    if os.path.exists(file1) and os.path.exists(file2):
        return 


    print(stddev)

    # 3D Plot using DRONE and BASE_TRUTH
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(drone_df['X'], drone_df['Y'], abs(drone_df['Z']), c='r', label='DRONE', s=5)
    ax.scatter(truth_df['X'], truth_df['Y'], truth_df['Z'], c='b', label='BASE_TRUTH', s=5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Trajectories for stddev = {stddev}')
    ax.legend()
    plt.savefig(file1)
    plt.close()

    # 2D Plot using only columns X and Y of BASE_TRUTH and BASE_MEASURED
    plt.figure()
    plt.plot(measured_df['X'], measured_df['Y'], 'go', label='BASE_MEASURED', markersize=3)
    plt.plot(truth_df['X'], truth_df['Y'], 'bo', label='BASE_TRUTH', markersize=3)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(f'Base position for stddev = {stddev}')
    plt.legend()
    plt.grid(True)
    plt.savefig(file2)
    plt.close()

if __name__ == '__main__':
    # Replace the directory path with the location of your CSV files
    directory = 'results/data/landing/'

    for measured, truth, drone in zip(sorted(os.listdir(directory+'measured')),
                                      sorted(os.listdir(directory+'truth')),
                                      sorted(os.listdir(directory+'drone'))):
        measured_file = os.path.join(directory+'measured',measured)
        truth_file = os.path.join(directory+'truth',truth)
        drone_file = os.path.join(directory+'drone',drone)
        plot(measured_file,truth_file,drone_file)
