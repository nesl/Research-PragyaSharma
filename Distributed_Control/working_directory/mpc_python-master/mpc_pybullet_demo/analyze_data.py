import pandas as pd
import numpy as np
import time
from matplotlib import pyplot as plt
import ast
from scipy.signal import savgol_filter

directory = "/home/pragya/Desktop/pybullet_scripts_personal_copy/mpc_python-master/mpc_pybullet_demo/"
files = ["latency_metrics_mpc.csv", "latency_metrics_pid.csv"]

def plot_mean_std(file, df):
    df.boxplot(by="Path_Number", column="Total_Path_Latency(s)")
    df.boxplot(by="Path_Number", column="Command_Latency_Metrics")
    plt.show()

def parse_data(df):
    lat_list = list(df.Command_Latency_Metrics)
    cmd_lat_list = []
    for i in lat_list[0]:
        cmd_lat_list.append(i)
    # print(cmd_lat_list)
    return cmd_lat_list

def smoothed_data(list):
    if "mpc" in file:
        return savgol_filter(list, 50, 3)
    elif "pid" in file:
        return savgol_filter(list, 100, 3)

def plot_iter_lat(file, lat_list, iter_list):
    max_len = len(max(lat_list, key = lambda i: len(i)))
    print(max_len)
    x_list = list(range(1,max_len+1))
    print(x_list)
    for i in lat_list:
        i += [None] * (max_len-len(i))
        i = smoothed_data(i)
        # print(i)
        plt.plot(x_list,i)
    if "mpc" in file:
        plt.title("Control Latency - MPC")
    elif "pid" in file:
        plt.title("Control Latency - PID")
    # plt.axis("equal")
    plt.legend(['Iteration 1', 'Iteration 2', 'Iteration 3'])
    plt.show()

def plot_other_figs():
    timeout_s = [0.33, 0.39, 0.48, 0.51, 0.66, 0.8]
    timeout_m = [0.45, 0.56, 0.59, 0.72, 0.81, 0.94]
    acc_s = [65.25, 66, 77, 78.8, 82, 86.2]
    acc_m = [66, 69.1, 70.1, 71.9, 73, 89.34]
    plt.plot(timeout_s, acc_s, "ro-")
    plt.plot(timeout_m, acc_m, "bo-")
    plt.title("Accuracy comparison - MPC Cloud")
    plt.legend(["Socket", "MQTT"])
    plt.ylabel("Accuracy (%)")
    plt.xlabel("Timeout (s)")
    plt.show()
    e2e_lat_s = [62.16, 62, 61.9, 61.5, 61.5, 61.2]
    e2e_lat_m = [101.43, 98.7, 99, 93.2, 92.7, 92.03]
    plt.plot(timeout_s, e2e_lat_s, "ro-")
    plt.plot(timeout_m, e2e_lat_m, "bo-")
    plt.title("E2E Latency comparison - MPC Cloud")
    plt.legend(["Socket", "MQTT"])
    plt.ylabel("E2E Latency (s)")
    plt.xlabel("Timeout (s)")
    plt.show()

for file in files:
    df = pd.read_csv(directory+file, converters={"Command_Latency_Metrics":pd.eval})
    # plot_mean_std(file, df)
    iter_lat = df.groupby("Path_Number")
    # print(iter_lat)
    collected_lat_data = []
    # iter_list = []
    for key, item in iter_lat:
        # print(key, item)
        temp_df = iter_lat.get_group(key)
        # print(temp_df)
        lat_list = temp_df.Command_Latency_Metrics.values.tolist()
        # print(lat_list)
        iter_list = temp_df.Num_Iteration.values.tolist()
        # print(iter_list)
        # plot_iter_lat(file, lat_list, iter_list)
    
plot_other_figs()
