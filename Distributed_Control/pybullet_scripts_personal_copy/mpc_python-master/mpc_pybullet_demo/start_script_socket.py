import numpy as np
import time
import datetime
import ast
import os
import random
import pandas as pd

# import mpc_socket_lc_robot as one
# import pid_socket_lc_robot_static_deadline as one
import mpc_socket_lc_robot_static_deadline as one
# import transfer_socket_lc_robot_static_deadline as one
# import pid_socket_lc_robot_dynamic_deadline as one
# import mpc_socket_lc_robot_dynamic_deadline as one
# import transfer_socket_lc_robot_dynamic_deadline as one
# import mpc_socket_lc_robot_stat_obs as one

directory = "/Users/pragyasharma/Documents/GitHub/Research-PragyaSharma/pybullet_scripts_personal_copy/mpc_python-master/mpc_pybullet_demo/latest"
path_file = "/Users/pragyasharma/Documents/GitHub/Research-PragyaSharma/pybullet_scripts_personal_copy/mpc_python-master/mpc_pybullet_demo/trajectories.txt"
save_file_local_pid_asap = directory+"/socket/latency_metrics_local_pid_asap.csv"
save_file_cloud_pid_asap = directory+"/socket/latency_metrics_cloud_pid_asap.csv"
save_file_local_mpc_asap = directory+"/socket/latency_metrics_local_mpc_asap.csv"
save_file_cloud_mpc_asap = directory+"/socket/latency_metrics_cloud_mpc_asap.csv"
save_file_stat_pid_local = directory+"/socket/latency_metrics_static_pid_local.csv"
save_file_stat_pid_cloud = directory+"/socket/latency_metrics_static_pid_cloud.csv"
save_file_stat_mpc_local = directory+"/socket/latency_metrics_static_mpc_local.csv"
save_file_stat_mpc_cloud = directory+"/socket/latency_metrics_static_mpc_cloud.csv"
save_file_stat_transfer = directory+"/socket/latency_metrics_static_transfer.csv"
save_file_dyn_pid_low = directory+"/socket/latency_metrics_dynamic_pid_low.csv"
save_file_dyn_pid_mod = directory+"/socket/latency_metrics_dynamic_pid_mod.csv"
save_file_dyn_pid_high = directory+"/socket/latency_metrics_dynamic_pid_high.csv"
save_file_dyn_mpc_low_local = directory+"/socket/latency_metrics_dynamic_mpc_low_local.csv"
save_file_dyn_mpc_mod_local = directory+"/socket/latency_metrics_dynamic_mpc_mod_local.csv"
save_file_dyn_mpc_high_local = directory+"/socket/latency_metrics_dynamic_mpc_high_local.csv"
save_file_dyn_mpc_low_cloud = directory+"/socket/latency_metrics_dynamic_mpc_low_cloud.csv"
save_file_dyn_mpc_mod_cloud = directory+"/socket/latency_metrics_dynamic_mpc_mod_cloud.csv"
save_file_dyn_mpc_high_cloud = directory+"/socket/latency_metrics_dynamic_mpc_high_cloud.csv"
save_file_dyn_mpc = directory+"/socket/latency_metrics_dynamic_mpc.csv"
save_file_dyn_transfer_low = directory+"/socket/latency_metrics_dynamic_transfer_low.csv"
save_file_dyn_transfer_mod = directory+"/socket/latency_metrics_dynamic_transfer_mod.csv"
save_file_dyn_transfer_high = directory+"/socket/latency_metrics_dynamic_transfer_high.csv"
save_file_asap_transfer = directory+"/socket/latency_metrics_asap_transfer.csv"
save_file_lc = directory+"/socket/latency_metrics_mpc_local_cloud.csv"
local_control_file = directory+"/mpc_local.py"
cloud_control_file = directory+"/mpc_lc_robot_test.py"
lc_control_file = directory+"/mpc_lc_robot_transfer.py"
# exec_files = [local_control_file, cloud_control_file, lc_control_file]
exec_files = [one]

path_list = []
df_columns = ['Path_Number', 'Command_Timeout(s)', 'Num_Iteration', 'Total_Path_Latency(s)', 
              'Accuracy', 'Num_Unique_Msgs', 'Num_Timeouts', 'Command_Latency_Metrics',]
df_columns_lc = ['Path_Number', 'Command_Timeout(s)', 'Num_Iteration', 'Total_Path_Latency(s)', 
              'Accuracy', 'Num_Local_Cmds', 'Num_Cloud_Cmds', 'Num_Timeouts', 'Command_Latency_Metrics',]

# Create DataFrame
global df_cloud, df_lc
df_cloud = pd.DataFrame(columns=df_columns)
df_lc = pd.DataFrame(columns=df_columns_lc)
 
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

# Robot's starting x,y pos from the specified path
def get_start_pos(path):
    # print(path)
    return [path[0][0], path[1][0]]

# Randomize cloud command timeout
def get_cmd_timeout():
    timeout = random.randint(10,30)
    # return timeout/100
    return 0.3

# Main function
def run_script():
    global df_cloud, df_lc
    # Run local, cloud_mqtt, local+cloud_mqtt control
    for file in exec_files:
        print(file)
        path_list = get_paths()
        tout_count = 1
        exec_file_lat = []
        # Run all paths for each file
        for p_idx, path in enumerate(path_list):
            start_pos = get_start_pos(path)
            path_iter_lat = []
            # Run with randomized cloud command timeout
            for tout_idx in range(tout_count):
                cmd_timeout = get_cmd_timeout()
                # print(path)
                # print(start_pos)
                print(cmd_timeout)
                cmd_iter_lat = []
                iterations = 1
                # Run 5 iterations of each cloud command timeout
                while iterations>0:
                    # print(iterations)
                    iter_start_time = time.time()

                    if file == one:
                        acc, lat_metrics, rxvd_lat, timeout_cnt, state_sq_no = file.reset_sim(str(path), cmd_timeout)
                        print(acc, lat_metrics, rxvd_lat, timeout_cnt, state_sq_no)
                        lat_data = {'Path_Number': p_idx+1, 
                                    'Command_Timeout(s)': cmd_timeout, 
                                    'Num_Iteration': iterations, 
                                    'Total_Path_Latency(s)': rxvd_lat,
                                    'Accuracy': acc,  
                                    'Num_Unique_Msgs': state_sq_no, 
                                    'Num_Timeouts': timeout_cnt, 
                                    'Command_Latency_Metrics': lat_metrics}
                        df_cloud = pd.concat([df_cloud, pd.DataFrame([lat_data])], ignore_index=True)
                        df_cloud.to_csv(save_file_stat_mpc_local, encoding='utf-8', index=False)
                    
                    # elif file == two:
                    #     acc, lat_metrics, rxvd_lat, timeout_cnt, state_sq_no , local_cmd_cnt, cloud_cmd_cnt = file.reset_sim(str(path), cmd_timeout)
                    #     print(acc, lat_metrics, rxvd_lat, timeout_cnt, state_sq_no)
                    #     lat_data = {'Path_Number': p_idx+1, 
                    #                 'Command_Timeout(s)': cmd_timeout, 
                    #                 'Num_Iteration': iterations, 
                    #                 'Total_Path_Latency(s)': rxvd_lat, 
                    #                 'Accuracy': acc, 
                    #                 'Num_Local_Cmds': local_cmd_cnt, 
                    #                 'Num_Cloud_Cmds': cloud_cmd_cnt, 
                    #                 'Num_Timeouts': timeout_cnt, 
                    #                 'Command_Latency_Metrics': lat_metrics}
                    #     pd.concat([df_lc, pd.DataFrame([lat_data])], ignore_index=True)
                    #     df_lc.to_csv(save_file_lc, encoding='utf-8', index=False)
                    
                    iter_end_time = time.time()
                    iter_time = iter_end_time - iter_start_time
                    # print(iter_time)
                    cmd_iter_lat.append(iter_time)
                    iterations = iterations-1
                print(cmd_iter_lat)
                # Process simulation latency for each timeout
                cmd_lat = np.average(cmd_iter_lat)
                path_iter_lat.append(cmd_lat)
            # Process path latency
            path_lat = np.average(path_iter_lat)
            exec_file_lat.append(path_lat)
        # Process exec_file latency
        total_lat = np.average(exec_file_lat)
        print(total_lat)
    # df.to_csv(save_file, encoding='utf-8', index=False)


if __name__ == "__main__":
    run_script()