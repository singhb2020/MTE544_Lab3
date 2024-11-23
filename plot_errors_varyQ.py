import matplotlib.pyplot as plt
import matplotlib as mpl
from utilities import FileReader

mpl.rcParams['xtick.labelsize'] = 8  # Font size for x-axis ticks
mpl.rcParams['ytick.labelsize'] = 8  # Font size for y-axis ticks

def plot_errors(filenames):
    
    all_headers = []
    all_values = []

    for filename in filenames:
        headers, values=FileReader(filename).read_file()
        all_headers.append(headers)
        all_values.append(values)
    
    fig, axes = plt.subplots(2,3, figsize=(14,5))

    plt0_odom, = axes[0][0].plot([lin[len(all_headers[0]) - 3] for lin in all_values[0]], [lin[len(all_headers[0]) - 2] for lin in all_values[0]], label='Odometry (Measured)', color='orange')
    plt0_ekf, = axes[0][0].plot([lin[len(all_headers[0]) - 5] for lin in all_values[0]], [lin[len(all_headers[0]) - 4] for lin in all_values[0]], label='EKF (Estimated)', color='blue')
    axes[0][0].set_title("Original (R=0.5 | Q=0.5)")
    axes[0][0].set_xlabel("X Position (m)", labelpad=0, fontsize=8)
    axes[0][0].set_ylabel("Y Position (m)", labelpad=0, fontsize=8)
    
    axes[0][0].grid()

    values = all_values[0]
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    plt0_odomx, = axes[1][0].plot(time_list, [lin[len(all_headers[0]) - 5] for lin in values], label= all_headers[0][6], color='red')
    plt0_odomy, = axes[1][0].plot(time_list, [lin[len(all_headers[0]) - 4] for lin in values], label= all_headers[0][7], color='green')
    plt0_kfx, = axes[1][0].plot(time_list, [lin[len(all_headers[0]) - 3] for lin in values], label= all_headers[0][8], color='purple')
    plt0_kfy, = axes[1][0].plot(time_list, [lin[len(all_headers[0]) - 2] for lin in values], label= all_headers[0][9], color='black')
    axes[1][0].set_xlabel("Time (ns)", labelpad=0, fontsize=8)
    axes[1][0].set_ylabel("Measurements/Filter Values (m)", labelpad=0, fontsize=8)
    
    axes[1][0].grid()

    axes[0][1].plot([lin[len(all_headers[1]) - 3] for lin in all_values[1]], [lin[len(all_headers[1]) - 2] for lin in all_values[1]], label='Odometry (Measured)', color='orange')
    axes[0][1].plot([lin[len(all_headers[1]) - 5] for lin in all_values[1]], [lin[len(all_headers[1]) - 4] for lin in all_values[1]], label='EKF (Estimated)', color='blue')
    axes[0][1].set_title("Large Q (R=0.5 | Q=1.0)")
    axes[0][1].set_xlabel("X Position (m)", labelpad=0, fontsize=8)
    axes[0][1].set_ylabel("Y Position (m)", labelpad=0, fontsize=8)
    
    axes[0][1].grid()

    values = all_values[1]
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    plt0_odomx, = axes[1][1].plot(time_list, [lin[len(all_headers[1]) - 5] for lin in values], label= all_headers[1][6], color='red')
    plt0_odomy, = axes[1][1].plot(time_list, [lin[len(all_headers[1]) - 4] for lin in values], label= all_headers[1][7], color='green')
    plt0_kfx, = axes[1][1].plot(time_list, [lin[len(all_headers[1]) - 3] for lin in values], label= all_headers[1][8], color='purple')
    plt0_kfy, = axes[1][1].plot(time_list, [lin[len(all_headers[1]) - 2] for lin in values], label= all_headers[1][9], color='black')
    axes[1][1].set_xlabel("Time (ns)", labelpad=0, fontsize=8)
    axes[1][1].set_ylabel("Measurements/Filter Values (m)", labelpad=0, fontsize=8)
    
    axes[1][1].grid()

    axes[0][2].plot([lin[len(all_headers[2]) - 3] for lin in all_values[2]], [lin[len(all_headers[2]) - 2] for lin in all_values[2]], label='Odometry (Measured)', color='orange')
    axes[0][2].plot([lin[len(all_headers[2]) - 5] for lin in all_values[2]], [lin[len(all_headers[2]) - 4] for lin in all_values[2]], label='EKF (Estimated)', color='blue')
    axes[0][2].set_title("Small Q (R=0.5 | Q=0.1)")
    axes[0][2].set_xlabel("X Position (m)", labelpad=0, fontsize=8)
    axes[0][2].set_ylabel("Y Position (m)", labelpad=0, fontsize=8)
    
    axes[0][2].grid()

    values = all_values[2]
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    plt0_odomx, = axes[1][2].plot(time_list, [lin[len(all_headers[2]) - 5] for lin in values], label= all_headers[2][6], color='red')
    plt0_odomy, = axes[1][2].plot(time_list, [lin[len(all_headers[2]) - 4] for lin in values], label= all_headers[2][7], color='green')
    plt0_kfx, = axes[1][2].plot(time_list, [lin[len(all_headers[2]) - 3] for lin in values], label= all_headers[2][8], color='purple')
    plt0_kfy, = axes[1][2].plot(time_list, [lin[len(all_headers[2]) - 2] for lin in values], label= all_headers[2][9], color='black')
    axes[1][2].set_xlabel("Time (ns)", labelpad=0, fontsize=8)
    axes[1][2].set_ylabel("Measurements/Filter Values (m)", labelpad=0, fontsize=8)
    
    axes[1][2].grid()


    fig.suptitle("Spiral Trajectory (Top) and Time Data (Bottom) with Varying Q", fontsize=16, fontweight='bold')

    #fig.text(0.5, 0.04, 'X Position (m)', ha='center')

    #fig.text(0.09, 0.5, 'Y Position (m)', va='center', rotation='vertical')
    
    fig.legend(handles=[plt0_odom, plt0_ekf, plt0_odomx, plt0_odomy, plt0_kfx, plt0_kfy], 
               labels=['Odometry (Measured)', 'EKF (Estimated)', 'EKF x (m)', 'EKF y (m)', 'Odom x (m)', 'Odom y (m)'], loc='upper right', fontsize=7)

    plt.show()

import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    plot_errors(filenames)


