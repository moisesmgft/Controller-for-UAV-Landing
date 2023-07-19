import pandas as pd
import matplotlib.pyplot as plt
import os

def tuning_plot(filepaths, params, fig_name, label, header, title):
    # Create a new figure
    plt.figure()

    # Iterate through each filepath and param, and plot the curve for each
    for filepath, param in zip(filepaths, params):
        # Read the CSV file into a DataFrame
        df = pd.read_csv(filepath)

        # Extract columns "Time" and "Output_X"
        time = df['Time'][2:]
        output_x = abs(df[header][2:])

        # Plot the curve with a label 'Kp={param}'
        plt.plot(time, output_x, label=label+param)

    # Set labels, title, legend, and grid for the plot
    plt.xlabel('Tempo (s)')
    plt.ylabel('Comando')
    plt.title(title,wrap=True, fontsize=12)
    plt.legend()
    plt.grid(True)
    plt.savefig(fig_name)
    plt.close()

def measured_analysis(directory, headerX, headerY, labels, title, fig_name):
    # Create a new figure
    plt.figure()

    # Iterate through all files in the directory
    for filename,label in zip(sorted(os.listdir(directory),reverse=True),labels):
        if filename.endswith('.csv'):
            filepath = os.path.join(directory, filename)
            # Read the CSV file into a DataFrame
            df = pd.read_csv(filepath)

            # Extract columns "X" and "Y"
            x_values = df[headerX]
            y_values = df[headerY]

            # Plot X vs Y with the filename as the label
            plt.scatter(x_values, y_values, label=label, s=2)

    # Set labels, title, legend, and grid for the plot
    plt.xlabel(headerX)
    plt.ylabel(headerY)
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.savefig(fig_name)
    plt.close()

def horizontal():
    horizontal_data_path = 'results/data/horizontal/'

    filepaths = [
        horizontal_data_path + "HORZ_0.200000__0.000000__0.000000.csv",
        horizontal_data_path + "HORZ_0.300000__0.000000__0.000000.csv",
        horizontal_data_path + "HORZ_0.400000__0.000000__0.000000.csv",
        horizontal_data_path + "HORZ_0.500000__0.000000__0.000000.csv"
    ]
    params = ["0.2", "0.3", "0.4", "0.5"] 
    horizontal_plot = 'results/plot/report/horizontal_Kp_tuning.png'
    tuning_plot(filepaths, params, 
                fig_name=horizontal_plot, 
                label='Kp = ', 
                header='Output_X', 
                title='Resposta à entrada degrau unitária para diferentes valores de Kp')
    
    filepaths = [
        horizontal_data_path + "HORZ_0.400000__0.250000__0.000000.csv",
        horizontal_data_path + "HORZ_0.400000__0.500000__0.000000.csv",
        horizontal_data_path + "HORZ_0.400000__0.800000__0.000000.csv",
        horizontal_data_path + "HORZ_0.400000__1.000000__0.000000.csv"
    ]
    params = ["0.25", "0.5", "0.8", "1.0"] 
    horizontal_plot = 'results/plot/report/horizontal_Ki_tuning.png'
    tuning_plot(filepaths, params, 
                fig_name=horizontal_plot, 
                label='Ki = ', 
                header='Output_X', 
                title='Resposta à entrada degrau unitária para diferentes valores de Ki\n(Kp = 0.4)')
    
    filepaths = [
        horizontal_data_path + "HORZ_0.400000__0.800000__0.050000.csv",
        horizontal_data_path + "HORZ_0.400000__0.800000__0.100000.csv",
        horizontal_data_path + "HORZ_0.400000__0.800000__0.150000.csv",
        horizontal_data_path + "HORZ_0.400000__0.800000__0.200000.csv"
    ]
    params = ["0.05", "0.1", "0.15", "0.2"] 
    horizontal_plot = 'results/plot/report/horizontal_Kd_tuning.png'
    tuning_plot(filepaths, params, 
                fig_name=horizontal_plot, 
                label='Kd = ', 
                header='Output_X', 
                title='Resposta à entrada degrau unitária para diferentes valores de Kd\n(Kp = 0.4, Ki = 0.8)')

def vertical():
    # Vertical tuning
    vertical_data_path = 'results/data/vertical/'

    filepaths = [
        vertical_data_path + "VERT_0.750000__0.000000__0.000000.csv",
        vertical_data_path + "VERT_1.000000__0.000000__0.000000.csv",
        vertical_data_path + "VERT_1.250000__0.000000__0.000000.csv",
        vertical_data_path + "VERT_1.500000__0.000000__0.000000.csv"
    ]
    params = ["0.75", "1.0", "1.25", "1.5"] 
    vertical_plot = 'results/plot/report/vertical_Kp_tuning.png'
    tuning_plot(filepaths, params, 
                fig_name=vertical_plot, 
                label='Kp = ', 
                header='Output_Z', 
                title='Resposta à entrada degrau unitária para diferentes valores de Kp')
    
    filepaths = [
        vertical_data_path + "VERT_1.250000__0.600000__0.000000.csv",
        vertical_data_path + "VERT_1.250000__0.750000__0.000000.csv",
        vertical_data_path + "VERT_1.250000__0.900000__0.000000.csv",
        vertical_data_path + "VERT_1.250000__1.050000__0.000000.csv"
    ]
    params = ["0.6", "0.75", "0.9", "1.05"] 
    vertical_plot = 'results/plot/report/vertical_Ki_tuning.png'
    tuning_plot(filepaths, params, 
                fig_name=vertical_plot, 
                label='Ki = ', 
                header='Output_Z', 
                title='Resposta à entrada degrau unitária para diferentes valores de Ki\n(Kp = 1.25)')
    
    filepaths = [
        vertical_data_path + "VERT_1.250000__0.900000__0.050000.csv",
        vertical_data_path + "VERT_1.250000__0.900000__0.100000.csv",
        vertical_data_path + "VERT_1.250000__0.900000__0.150000.csv",
        vertical_data_path + "VERT_1.250000__0.900000__0.200000.csv"
    ]
    params = ["0.05", "0.1", "0.15", "0.2"] 
    vertical_plot = 'results/plot/report/vertical_Kd_tuning.png'
    tuning_plot(filepaths, params, 
                fig_name=vertical_plot, 
                label='Kd = ', 
                header='Output_Z', 
                title='Resposta à entrada degrau unitária para diferentes valores de Kd\n(Kp = 1.25, Ki = 0.9)')

if __name__ == '__main__':

    #horizontal()
    #vertical()
    
    directory = 'results/data/landing/measured'
    headerX = 'X'
    headerY = 'Y'
    sig = r'$\sigma$ = '
    labels = [sig+r'0.20 $m$',
              sig+r'0.15 $m$',
              sig+r'0.10 $m$',
              sig+r'0.05 $m$',
              sig+r'0.02 $m$',
              sig+r'0.01 $m$',
              sig+r'0.00 $m$']
    title = 'Posição da base medida pelo UAV'
    fig_name = 'results/plot/report/trajectoryXY.png'

    measured_analysis(directory, headerX, headerY, labels, title, fig_name)

    directory = 'results/data/landing/drone'
    headerX = 'Time'
    headerY = 'X'
    sig = r'$\sigma$ = '
    labels = [sig+r'0.20 $m$',
              sig+r'0.15 $m$',
              sig+r'0.10 $m$',
              sig+r'0.05 $m$',
              sig+r'0.02 $m$',
              sig+r'0.01 $m$',
              sig+r'0.00 $m$']
    title = 'Posição X do UAV em função do tempo'
    fig_name = 'results/plot/report/trajectoryXTime.png'

    measured_analysis(directory, headerX, headerY, labels, title, fig_name)

    directory = 'results/data/landing/drone'
    headerX = 'Time'
    headerY = 'Y'
    sig = r'$\sigma$ = '
    labels = [sig+r'0.20 $m$',
              sig+r'0.15 $m$',
              sig+r'0.10 $m$',
              sig+r'0.05 $m$',
              sig+r'0.02 $m$',
              sig+r'0.01 $m$',
              sig+r'0.00 $m$']
    title = 'Posição Y do UAV em função do tempo'
    fig_name = 'results/plot/report/trajectoryYTime.png'

    measured_analysis(directory, headerX, headerY, labels, title, fig_name)


