import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
from matplotlib import animation

from mpcpy.utils import compute_path_from_wp
import mpcpy

global P
P = mpcpy.Params()

import sys
import time
import paho.mqtt.client as mqtt
import datetime
import pybullet as p

import socket 
import threading

from rrt_path_planning import rrt_path_planner as rrt
from rrt_path_planning import PathSmoothing as ps
from rrt_path_planning import utils as ut

global rrt_params
rrt_params = ut.RRT_Params()

HEADER = 64
PORT = 5061
SERVER = "127.0.0.1"
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(ADDR)

ctrl_cmd = [0]
old_cmd = [0]
global cmd_sq_no
cmd_sq_no = 1

# starting guess
global action
action = np.zeros(P.M)
action[0] = P.MAX_ACC / 2  # a
action[1] = 0.0  # delta
# print("Initial action: ", action)

# Cost Matrices
Q = np.diag([20, 20, 10, 20])  # state error cost
# Qf = np.diag([30, 30, 30, 30])  # state final error cost
R = np.diag([10, 10])  # input cost
# R_ = np.diag([10, 10])  # input rate of change cost

mpc = mpcpy.MPC(P.N, P.M, Q, R)

def get_path_from_num(path_num):
    if path_num == 1:
        path = compute_path_from_wp(
            [0, 4, 6, 8],
            [0, 0, -2, 2],
            P.path_tick,)
    elif path_num == 2:
        path = compute_path_from_wp(
            [0, 2, 4, 8, 12, 20],
            [0, 2.5, -2.5, 2.5, -1.8, 3.8],
            P.path_tick,)
    elif path_num == 3:
        path = compute_path_from_wp(
            [0, 1, -3, -5, -3, 2, 2, 4, 6, 6, 8, 8, 4, 4, 0, -2, -4, -6, -6],
            [0, 3, -2, -6, -7, -7, -2, -2, 0, 3, 5, 10, 10, 7, 7, 9, 9, 5, 2],
            P.path_tick,)
    elif path_num == 4:
        path = compute_path_from_wp(
            [0, 3, 4, 6, 10, 12, 10],
            [0, 0, 2, 4, 3, 0, -3],
            P.path_tick,)
    elif path_num == 5:
        path = compute_path_from_wp(
            [0, 2, 3, 5, 6, 8, 8, 10, 8, 8, 6, 5, 3, 2, 0],
            [0, -2, -2, -4, -4, -2, -1, 0, 1, 2, 4, 4, 2, 2, 4],
            P.path_tick,)
    return path

def process_message(msg):

    msg_decode = msg #str(msg.payload.decode("utf-8"))
    now = datetime.datetime.now()
    # f.write(str(now.time()))
    print(" Message received: "+msg_decode+'\n')

    try:
        msg_decode = msg_decode.split("[ ")[1].split("]")[0]
    except:
        msg_decode = msg_decode.split("[")[1].split("]")[0]
    temp_state = list(msg_decode.split(" "))
    rx_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # print(temp_state)
    temp_state = list(filter(lambda a: a!= '', temp_state))
    # print(temp_state)
    rx_state[0] = float(temp_state[0])
    rx_state[1] = float(temp_state[1])
    rx_state[2] = float(temp_state[2])
    rx_state[3] = float(temp_state[3])
    rx_state[4] = float(temp_state[4])
    rx_state[5] = float(temp_state[5])
    state = np.asarray(rx_state[0:4])
    print("State info received at cloud: ", state)
    # print("Rxvd State Seq No: ", str(rx_state[4]))
    state_sq_no = rx_state[4]
    path_num = int(rx_state[5])
    print("Rxvd Path No: ", str(path_num))

    path = get_path_from_num(path_num)
    # time.sleep(0.5)

    # # w = 1 # bounding box
    # # loc1, loc2 = get_obs_location()
    # # obs = [
    # #     np.array([[loc1[0]-w, -w], [loc1[0]+w, -w],  [loc1[1]-w, w], [loc1[1]+w, w]]), 
    # #     np.array([[loc2[0]-w, -w], [loc2[0]+w, -w],  [loc2[1]-w, w], [loc2[1]+w, w]])]

    # # if get_distance(loc1[0], loc1[1], rx_state[0], rx_state[1])<0.5 or get_distance(loc2[0], loc2[1], rx_state[0], rx_state[1])<0.5:
        
    # #     # Create new waypoints, invoke RRT

    # #     xy_start = np.array([rx_state[0], rx_state[1]])
    # #     xy_goal =  np.array(find_nearest_goal(trans_path, rx_state[0], rx_state[1])) # Get next to next waypoint

    # #     rrt_P = rrt.rrt_path(obs, xy_start, xy_goal, rrt_params)
    # #     print("start: ", xy_start)
    # #     print("goal: ", xy_goal)
    # #     print("rrt_P: ", rrt_P)
    # #     P_smooth = ps.SmoothPath(rrt_P, obs, smoothiters=100)

    # #     traj = np.array([P_smooth[0]])
    # #     for i in range(len(P_smooth)-1):
    # #         A = P_smooth[i]
    # #         B = P_smooth[i+1]
    # #         traj = np.vstack([traj, A])
            
    # #         n = (B-A) / norm(B-A)
    # #         delta = n * P.DT
    # #         N = int( norm(B-A) / norm(delta))
    # #         sp = A
    # #         for i in range(N):
    # #             sp += delta
    # #             traj = np.vstack([traj, sp])
    # #         traj = np.vstack([traj, B])

    # for MPC car ref frame is used
    new_state = np.copy(state)
    new_state[0:2] = 0.0
    new_state[3] = 0.0
    # new_state = np.ndarray(shape=(1,4), dtype=float)
    # new_state[0,0]=0.0
    # new_state[0,1] = 0.0
    # new_state[0,2] = state[0,2]
    # new_state[0,3] = 0.0
    # state[0:2] = 0.0
    # state[3] = 0.0

    # add 1 timestep delay to input
    print(new_state)
    global action
    new_state[0] = new_state[0] + new_state[2] * np.cos(new_state[3]) * P.DT
    new_state[1] = new_state[1] + new_state[2] * np.sin(new_state[3]) * P.DT
    new_state[2] = new_state[2] + action[0] * P.DT
    new_state[3] = new_state[3] + action[0] * np.tan(action[1]) / P.L * P.DT


    # State Matrices
    A, B, C = mpcpy.get_linear_model_matrices(new_state, action)

    # Get Reference_traj -> inputs are in worldframe
    # try:
    #     target, _ = mpcpy.get_ref_trajectory(state, traj, 1.0)
    # except:
    target, _ = mpcpy.get_ref_trajectory(state, path, 1.0)

    x_mpc, u_mpc = mpc.optimize_linearized_model(
        A, B, C, new_state, target, time_horizon=P.T, verbose=False
    )

    # action = np.vstack((np.array(u_mpc.value[0,:]).flatten(),
    #              (np.array(u_mpc.value[1,:]).flatten())))

    action[:] = [u_mpc.value[0, 1], u_mpc.value[1, 1]]


    global ctrl_cmd
    global old_cmd
    old_cmd = ctrl_cmd
    ctrl_cmd = [new_state[2], action[0], action[1], state_sq_no]
    # ctrl_cmd = [1, 1, 1, 1]
    print("Control command generated: "+str(ctrl_cmd))


def handle_client(conn, addr):
    print(f"[NEW CONNECTION] {addr} connected.")

    connected = True
    while connected:
        msg_length = conn.recv(HEADER).decode(FORMAT)
        if msg_length:
            msg_length = int(msg_length)
            msg = conn.recv(msg_length).decode(FORMAT)
            if msg == DISCONNECT_MESSAGE:
                connected = False

            # print(msg)
            process_message(msg)

            conn.send(str(ctrl_cmd).encode(FORMAT))

    conn.close()

def start():
    server.listen()
    print(f"[LISTENING] Server is listening on {SERVER}")
    while True:
        conn, addr = server.accept()
        thread = threading.Thread(target=handle_client, args=(conn, addr))
        thread.start()
        print(f"[ACTIVE CONNECTIONS] {threading.activeCount() - 1}")


print("[STARTING] server is starting...")
start()