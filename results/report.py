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
        output_x = df[header][2:]

        # Plot the curve with a label 'Kp={param}'
        plt.plot(time, output_x, label=label+param)

    # Set labels, title, legend, and grid for the plot
    plt.xlabel('Tempo (s)')
    plt.ylabel('Comando')
    plt.title(title)
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
                title='Resposta à entrada degrau unitária para diferentes valores de Ki (Kp = 0.4)')
    
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
                title='Resposta à entrada degrau unitária para diferentes valores de Ki (Kp = 0.4, Kd = 0.8)')