import pandas as pd
import matplotlib.pyplot as plt

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

if __name__ == '__main__':

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