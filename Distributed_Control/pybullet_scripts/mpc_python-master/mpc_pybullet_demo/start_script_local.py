import numpy as np
import time
import datetime
import ast
import os
import random
import pandas as pd

# import mpc_local as one
import pid_local as two

directory = "/Users/pragyasharma/Documents/GitHub/Research-PragyaSharma/pybullet_scripts_personal_copy/mpc_python-master/mpc_pybullet_demo"
path_file = directory+"/trajectories.txt"
save_file_mpc = directory+"/latency_metrics_mpc.csv"
save_file_pid = directory+"/latency_metrics_pid.csv"
mpc_control_file = directory+"/mpc_local.py"
pid_control_file = directory+"/pid_local.py"
exec_files = [two]

path_list = []
df_columns = ['Path_Number', 'Num_Iteration', 'Total_Path_Latency(s)', 'Accuracy',
              'Num_Unique_Msgs', 'Command_Latency_Metrics',]

# Create DataFrame
global df_metrics
df_metrics = pd.DataFrame(columns=df_columns)
 
# Read paths from path_file
def get_paths():
    ret_path_list = []
    with open(path_file) as file:
        path_list = [line.rstrip() for line in file]
    # print(path_list)
    for path in path_list:
        path = ast.literal_eval(path)
        # print(type(path))
        # print(path)
        ret_path_list.append(path)
    return ret_path_list

# Main function
def run_script():
    global df_metrics
    # Run local, cloud_mqtt, local+cloud_mqtt control
    for file in exec_files:
        print(file)
        path_list = get_paths()
        exec_file_lat = []
        # Run all paths for each file
        for p_idx, path in enumerate(path_list):
            path_iter_lat = []
            iterations = 1
            # Run 5 iterations of each cloud command timeout
            while iterations>0:
                # print(iterations)
                iter_start_time = time.time()
                accuracy, lat_metrics, rxvd_lat, state_sq_no = file.reset_sim(str(path))
                # print(accuracy, lat_metrics, rxvd_lat, state_sq_no)
                lat_data = {'Path_Number': p_idx+1, 
                            'Num_Iteration': iterations, 
                            'Total_Path_Latency(s)': rxvd_lat, 
                            'Accuracy': accuracy,
                            'Num_Unique_Msgs': state_sq_no, 
                            'Command_Latency_Metrics': lat_metrics}
                pd.concat([df_metrics, pd.DataFrame([lat_data])], ignore_index=True)
                # if file == one:
                #     df_metrics.to_csv(save_file_mpc, encoding='utf-8', index=False)
                if file == two:
                    df_metrics.to_csv(save_file_pid, encoding='utf-8', index=False)
                iter_end_time = time.time()
                iter_time = iter_end_time - iter_start_time
                # print(iter_time)
                iterations = iterations-1
                path_iter_lat.append(iter_time)
            # Process path latency
            path_lat = np.average(path_iter_lat)
            exec_file_lat.append(path_lat)
        # Process exec_file latency
        total_lat = np.average(exec_file_lat)
        print(total_lat)


if __name__ == "__main__":
    run_script()