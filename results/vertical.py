import os
import pandas as pd
import matplotlib.pyplot as plt

def plot_curve_from_csv(csv_file):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(csv_file)

    PID = csv_file.replace(".csv", "").replace("data","plot")

    plot_filename = f'{PID}.png'
    if os.path.exists(plot_filename):
        return


    # Extract columns "Time" and "Output_Z"
    time = df['Time'][2:]
    output_z = df['Output_Z'][2:]

    # Plot the curve
    plt.plot(time, output_z)
    plt.xlabel('Time')
    plt.ylabel('Output_Z')
    plt.title(f'Curve for {PID}')
    plt.grid(True)
    plt.savefig(plot_filename)  # Save the plot as an image
    plt.show()  # Show the plot on the screen (optional)

if __name__ == '__main__':
    # Replace the directory path with the location of your CSV files
    directory = 'results/data/'
    
    # Iterate through all files in the directory
    for filename in os.listdir(directory):
        if filename.startswith('VERT_') and filename.endswith('.csv'):
            filepath = os.path.join(directory, filename)
            plot_curve_from_csv(filepath)
