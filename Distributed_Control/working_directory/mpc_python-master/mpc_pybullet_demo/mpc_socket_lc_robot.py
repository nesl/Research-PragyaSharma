import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

from mpcpy.utils import compute_path_from_wp
import mpcpy

P = mpcpy.Params()

import sys
import time
import ast

import pybullet as p
import sys
import paho.mqtt.client as mqtt
import datetime
import socket

state_rxvd = "last_cmd_rxvd"
state_txd = "latest_state_txd"
state_timeout = "timeout"

global curr_state
curr_state = state_rxvd
global state_sq_no
state_sq_no = 0
global path_num
path_num=1

global last_state_time
last_state_time = 0
global counter
counter = 0
global x_history, y_history
x_history = []
y_history = []
global cld_cmd_lat
cld_cmd_lat = []
global trans_path
trans_path = []

HEADER = 64
PORT = 5060
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
SERVER = "54.183.0.212"
ADDR = (SERVER, PORT)

def get_state(robotId):
    """ """
    robPos, robOrn = p.getBasePositionAndOrientation(robotId)
    linVel, angVel = p.getBaseVelocity(robotId)

    return np.array(
        [
            robPos[0],
            robPos[1],
            np.sqrt(linVel[0] ** 2 + linVel[1] ** 2),
            p.getEulerFromQuaternion(robOrn)[2],
            state_sq_no,
            path_num,
        ]
    )

def set_ctrl(robotId, currVel, acceleration, steeringAngle, cmd_sq_no):

    # print("In set_ctrl function")
    # print(robotId, currVel, acceleration, steeringAngle)

    gearRatio = 1.0 / 21
    steering = [0, 2]
    wheels = [8, 15]
    maxForce = 50

    targetVelocity = (currVel + acceleration * P.DT)
    # targetVelocity=lastVel
    # print(targetVelocity)

    for wheel in wheels:
        p.setJointMotorControl2(
            robotId,
            wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=targetVelocity/ gearRatio,
            force=100,
        )

    for steer in steering:
        p.setJointMotorControl2(
            robotId, steer, p.POSITION_CONTROL, targetPosition=steeringAngle
        )
    
    if cmd_sq_no!=9999:
        #Don't switch state for halting
        global curr_state
        curr_state = state_rxvd

    if cmd_sq_no==9999:
        return

    print("Rx cmd sq no: ", str(cmd_sq_no))

def plot_results(path, x_history, y_history, timeout):
    """ """
    plt.style.use("ggplot")
    plt.figure()
    plt.title("MPC Tracking Results - Socket Cloud")

    plt.plot(
        path[0, :], path[1, :], c="tab:orange", marker=".", label="reference track"
    )
    plt.plot(
        x_history,
        y_history,
        c="tab:blue",
        marker=".",
        alpha=0.5,
        label="vehicle trajectory",
    )
    plt.axis("equal")
    plt.legend()
    # plt.show()
    plt.savefig(f"Socket_Cloud_PathNum_{path_num}_Timeout_{timeout}.png")

def connect():
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(ADDR)
    return client

def send(client, msg):
    message = msg.encode(FORMAT)
    msg_length = len(message)
    send_length = str(msg_length).encode(FORMAT)
    send_length += b' ' * (HEADER - len(send_length))
    client.send(send_length)
    client.send(message)

def recv(client):
    msg_decode = client.recv(2048).decode(FORMAT)
    global last_cmd_time
    last_cmd_time = time.time()
    print("Message received: "+msg_decode+'\n')
    if curr_state == "end_sim":
        return

    msg_decode = msg_decode.split("[")[1].split("]")[0]
    temp_state = list(msg_decode.split(","))
    temp_state[0]=float(temp_state[0])
    temp_state[1] = float(temp_state[1])
    temp_state[2] = float(temp_state[2])
    temp_state[3] = float(temp_state[3])
    state = np.asarray(temp_state)

    global cld_cmd_lat, last_state_time
    latency = last_cmd_time-last_state_time
    print("Message received latency: ", latency)
    if latency>10000:
        return
    cld_cmd_lat.append(latency)

    set_ctrl(car, state[0], state[1], state[2], state[3])

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

def check_path_num(path_var):
    global path_num
    if path_var[0] == [0, 6, 28, 28.5, 25, 26]:
        path_num = 1
    elif path_var[0] == [0, 2, 4, 8, 12, 20]:
        path_num = 2
    elif path_var[0] == [0, 1, -3, -5, -3, 2, 2, 4, 6, 6, 8, 8, 4, 4, 0, -2, -4, -6, -6]:
        path_num = 3
    elif path_var[0] == [0, 3, 4, 6, 10, 11, 12, 6, 1, 0]:
        path_num = 4
    elif path_var[0] == [0, 2, 3, 5, 6, 8, 8, 10, 8, 8, 6, 5, 3, 2, 0]:
        path_num = 5
        
def env_setup(path_var):
    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.0,
        cameraYaw=-90,
        cameraPitch=-45,
        cameraTargetPosition=[-0.1, -0.0, 0.65],
    )

    p.resetSimulation()

    p.setGravity(0, 0, -10)
    useRealTimeSim = 1

    p.setTimeStep(1.0 / 120.0)
    p.setRealTimeSimulation(useRealTimeSim)  # either this

    plane = p.loadURDF("racecar/plane.urdf")
    # track = p.loadSDF("racecar/f10_racecar/meshes/barca_track.sdf", globalScaling=1)

    global car
    car = p.loadURDF("racecar/f10_racecar/racecar_differential.urdf", [0, 0, 0.3])
    print("car:", car)
    for wheel in range(p.getNumJoints(car)):
        # print("joint[",wheel,"]=", p.getJointInfo(car,wheel))
        p.setJointMotorControl2(
            car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0
        )
        p.getJointInfo(car, wheel)

    c = p.createConstraint(
        car,
        9,
        car,
        11,
        jointType=p.JOINT_GEAR,
        jointAxis=[0, 1, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0],
    )
    p.changeConstraint(c, gearRatio=1, maxForce=10000)

    c = p.createConstraint(
        car,
        10,
        car,
        13,
        jointType=p.JOINT_GEAR,
        jointAxis=[0, 1, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0],
    )
    p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    c = p.createConstraint(
        car,
        9,
        car,
        13,
        jointType=p.JOINT_GEAR,
        jointAxis=[0, 1, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0],
    )
    p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    c = p.createConstraint(
        car,
        16,
        car,
        18,
        jointType=p.JOINT_GEAR,
        jointAxis=[0, 1, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0],
    )
    p.changeConstraint(c, gearRatio=1, maxForce=10000)

    c = p.createConstraint(
        car,
        16,
        car,
        19,
        jointType=p.JOINT_GEAR,
        jointAxis=[0, 1, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0],
    )
    p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    c = p.createConstraint(
        car,
        17,
        car,
        19,
        jointType=p.JOINT_GEAR,
        jointAxis=[0, 1, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0],
    )
    p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    c = p.createConstraint(
        car,
        1,
        car,
        18,
        jointType=p.JOINT_GEAR,
        jointAxis=[0, 1, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0],
    )
    p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
    c = p.createConstraint(
        car,
        3,
        car,
        19,
        jointType=p.JOINT_GEAR,
        jointAxis=[0, 1, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0],
    )
    p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

    # Update path num
    check_path_num(path_var)
    
    # Interpolated Path to follow given waypoints
    path = compute_path_from_wp(
        path_var[0],
        path_var[1],
        P.path_tick,
    )

    global trans_path
    trans_path = transform_path(path)
    print(trans_path)

    for x_, y_ in zip(path[0, :], path[1, :]):
        p.addUserDebugLine([x_, y_, 0], [x_, y_, 0.33], [0, 0, 1])

    return trans_path, path

def timer_expired (last_state_time, timeout):
    now = time.time()
    print("Timer status: ", now-last_state_time)
    if now-last_state_time >= timeout:
        print("Timer Expired!!")
        return True
    else:
        return False

def get_distance(x1, y1, x2, y2):
	return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def find_nearest_goal(trans_path, x_pos, y_pos):
    waypoint_lst = [tuple(x) for x in trans_path]
    waypoint_lst = np.array(waypoint_lst)
    target = np.array((x_pos, y_pos))
    dist = np.linalg.norm(waypoint_lst-target, axis=1)
    min_idx = np.argmin(dist)
    goal_pt = waypoint_lst[min_idx]
    # print(type(goal_pt))
    return goal_pt

def calc_accuracy(path,  x_history, y_history):
    idx=0
    err_bound = 1
    err_array = np.zeros(len(x_history))
    trans_path = transform_path(path)
    print(len(trans_path))
    print(len(x_history))
    # print(trans_path)
    
    while idx!=len(x_history)-1:
        print("idx:", idx)
        target_waypt = find_nearest_goal(trans_path, x_history[idx], y_history[idx])
        err = get_distance(target_waypt[0], target_waypt[1], x_history[idx], y_history[idx])
        print(target_waypt[0], target_waypt[1], x_history[idx], y_history[idx], err)
        if err>err_bound:
            err = err_bound
        err_array[idx] = ((err_bound - err)/err_bound)*100
        print(err_array[idx])
        print("average = ", np.average(err_array))
        idx = idx+1
    return np.average(err_array)

def have_reached(state, path, timeout):
    global sim_start_time

    if np.sqrt((state[0] - path[0, -1]) ** 2 + (state[1] - path[1, -1]) ** 2) < 0.2:
        print("Success! Goal Reached")
        set_ctrl(car, 0, 0, 0, 9999)
        global curr_state
        curr_state = "end_sim"
        
        # Get simulation start time
        sim_end_time = time.time()
        time_taken = sim_end_time-sim_start_time
        print("Time at finish: ", sim_end_time)
        print("Total time: ", time_taken)

        plot_results(path, x_history, y_history, timeout)
        accuracy = calc_accuracy(path, x_history, y_history)
        print("Accuracy: ", accuracy)
        p.disconnect()
        return accuracy, time_taken
    else:
        return None, None

global client
client = connect()

def reset_sim(path_var, timeout):
    global last_state_time, counter, sim_start_time, cld_cmd_lat
    global x_history, y_history, curr_state, state_sq_no, trans_path

    last_state_time = 0
    counter = 0
    x_history = []
    y_history = []
    cld_cmd_lat = []
    trans_path = []
    curr_state = state_rxvd
    state_sq_no = 1
    # Get simulation start time
    sim_start_time = time.time()

    acc, rxvd_lat, timeout_cnt, state_sq_no = start_sim(path_var, timeout)
    return acc, cld_cmd_lat, rxvd_lat, timeout_cnt, state_sq_no

def start_sim(path_var, timeout):

    global last_state_time, curr_state, state_sq_no, sim_start_time
    global x_history, y_history

    # Process path variable from str to list
    path_var = ast.literal_eval(path_var)
    tarns_path, path = env_setup(path_var)

    timeout_cnt = 0
    state = get_state(car)

    sim_start_time = time.time()

    while True:
        # f.flush()
        if curr_state == state_txd and timer_expired(last_state_time, timeout):
            print(f"Timeout: {timeout}; halting car")
            set_ctrl(car, 0, 0, 0, 9999) # Halt car
            # last_state_time = time.time()
            timeout_cnt = timeout_cnt+1
            curr_state = state_timeout
        elif curr_state == state_timeout:
            print("State machine at: ", curr_state)
            # Resend new state after 100ms
            # time.sleep(0.08)
            curr_state = state_rxvd
        elif curr_state == state_rxvd:
            # time.sleep(1)
            print("State machine at: ", curr_state)
            # send new state
            state = get_state(car)
            print("Sent new state",str(state))
            send(client, str(state))
            last_state_time = time.time()
            # print(last_state_time)
            state_sq_no = state_sq_no + 1
            if state_sq_no >= 2000:
                print("Failed! Car in bad state")
                set_ctrl(car, 0, 0, 0, 9999)
                curr_state = "end_sim"
                plot_results(path, x_history, y_history, timeout)
                p.disconnect()
                time_taken = None
                acc = None
                return acc, time_taken, timeout_cnt, state_sq_no
            curr_state = state_txd
            print("State machine at: ", curr_state)
            print('\n')
        elif curr_state == state_txd:
            recv(client)
            global counter
            counter = counter+1
            if counter % 10 == 0:
                print("State machine at: ", curr_state)
            # wait for new cmd to arrive
            # time.sleep(3.5)

        # time.sleep(3)

        # track path in bullet
        p.addUserDebugLine(
            [state[0], state[1], 0], [state[0], state[1], 0.5], [1, 0, 0]
        )

        x_history.append(state[0])
        y_history.append(state[1])

        acc, time_taken = have_reached(state, path, timeout)
        if time_taken!=None and acc!=None:
            return acc, time_taken, timeout_cnt, state_sq_no
        
if __name__ == "__main__":
    path = str(sys.argv[1])
    timeout = float(sys.argv[2])
    start_sim(path, timeout)