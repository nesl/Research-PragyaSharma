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

global prev_error_angle, prev_error_position, prev_waypoint_idx, prev_body_to_goal
global kp_linear, kd_linear, kp_angular, kd_angular
prev_error_position = 0
prev_error_angle = 0
prev_body_to_goal = 0
prev_waypoint_idx = -1
kp_linear = 2.6
kd_linear = 0.1
ki_linear = 0
kp_angular = 100
kd_angular = 25
ki_angular = 0
global current_idx_pid
current_idx_pid = 0
global prev_path_num
prev_path_num = 0

def transform_path(path):
    idx = 0
    trans_path = []
    while idx < len(path[0]):
        curr_coord = []
        curr_coord.append(path[0][idx])
        curr_coord.append(path[1][idx])
        trans_path.append(curr_coord)
        idx = idx+1
    # print(trans_path)
    return trans_path

def get_path_from_num(path_num):
    if path_num == 1:
        path = compute_path_from_wp(
            [0, 6, 28, 28.5],
            [0, 7.5, -5.5, 0.8],
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
    trans_path = transform_path(path)
    return path, trans_path

def get_distance(x1, y1, x2, y2):
	return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def get_angle(x1, y1, x2, y2):
	# return np.arctan2(y2 - y1, x2 - x1)
	return np.arctan2(y2 - y1, x2 - x1)

#TODO: Update to handle past waypoints
def find_nearest_goal(trans_path, x_pos, y_pos):
    waypoint_lst = [tuple(x) for x in trans_path]
    waypoint_lst = np.array(waypoint_lst)
    target = np.array((x_pos, y_pos))
    dist = np.linalg.norm(waypoint_lst-target, axis=1)
    min_idx = np.argmin(dist)
    goal_pt = waypoint_lst[min_idx]
    # print(type(goal_pt))
    return goal_pt

def get_pid_control_inputs(state, goal_x, waypoint_idx):
    print("In PID control")
    global prev_error_angle, prev_error_position, prev_waypoint_idx, prev_body_to_goal
    global kp_linear, kd_linear, kp_angular, kd_angular

    error_position = get_distance(state[0], state[1], goal_x[0], goal_x[1])
    
    body_to_goal = get_angle(state[0], state[1], goal_x[0], goal_x[1])

    # if self.prev_waypoint_idx == waypoint_idx and 350<(abs(self.prev_body_to_goal - body_to_goal)*180/np.pi):
    # 	print("HERE")
    # 	body_to_goal = self.prev_body_to_goal
    error_angle = (body_to_goal) - state[3]

    linear_velocity_control = kp_linear*error_position + kd_linear*(error_position - prev_error_position)
    angular_velocity_control = kp_angular*error_angle + kd_angular*(error_angle - prev_error_angle)

    prev_error_angle = error_angle
    prev_error_position = error_position

    prev_waypoint_idx = waypoint_idx
    prev_body_to_goal = body_to_goal

    if linear_velocity_control>0.5:
        linear_velocity_control = 0.5

    return linear_velocity_control, angular_velocity_control


def process_message(msg):
    global prev_path_num
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

    path, trans_path = get_path_from_num(path_num)
    global current_idx_pid
    if prev_path_num != path_num:
        current_idx_pid = 0
        prev_path_num = path_num
    # time.sleep(0.5)

    if len(trans_path)>0 and current_idx_pid != len(trans_path):
            goal_pt = trans_path[current_idx_pid] #target waypoint
            print(goal_pt)
            linear_v = 0
            angular_v = 0
            linear_v, angular_v = get_pid_control_inputs(state, goal_pt, current_idx_pid)
            # set_ctrl(car, state[2], linear_v, angular_v)
            dist = get_distance(state[0], state[1], goal_pt[0], goal_pt[1])
            print("dist: ", dist)
            if dist<0.5:
                current_idx_pid+= 1

    global ctrl_cmd
    global old_cmd
    old_cmd = ctrl_cmd
    ctrl_cmd = [state[2], linear_v, angular_v, state_sq_no]
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
    # time.sleep(20)
    server.listen()
    print(f"[LISTENING] Server is listening on {SERVER}")
    while True:
        conn, addr = server.accept()
        thread = threading.Thread(target=handle_client, args=(conn, addr))
        thread.start()
        print(f"[ACTIVE CONNECTIONS] {threading.activeCount() - 1}")


print("[STARTING] server is starting...")
start()