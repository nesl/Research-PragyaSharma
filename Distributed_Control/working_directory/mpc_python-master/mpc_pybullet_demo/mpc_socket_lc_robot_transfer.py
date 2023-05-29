import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

from mpcpy.utils import compute_path_from_wp
import mpcpy

P = mpcpy.Params()

import sys
import time

import pybullet as p
import sys
import paho.mqtt.client as mqtt
import datetime

import socket
import ast

HEADER = 64
PORT = 5056
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
SERVER = "127.0.0.1"#13.52.247.117"
ADDR = (SERVER, PORT)

state_rxvd = "last_cmd_rxvd"
state_txd = "latest_state_txd"
state_timeout = "timeout"
state_msg_proc = "cloud_cmd_processing"

global curr_state
curr_state = state_rxvd
global state_sq_no
state_sq_no = 0
global cloud_cmd_sq_no, local_cmd_sq_no
cloud_cmd_sq_no = 9999
local_cmd_sq_no = 9999
global local_ctrl_cmd
global path_num
path_num=1
global pid_path
pid_path = []
global current_idx_pid
current_idx_pid = 0

global last_state_time
last_state_time = 0
global counter
counter = 0
global x_history, y_history
x_history = []
y_history = []
global cld_cmd_lat
cld_cmd_lat = []
global local_iter
local_iter = 0
global num_state_sent
num_state_sent = 0
global last_state_sq_no
last_state_sq_no = 1

global local_cmd_cnt, cloud_cmd_cnt
local_cmd_cnt = 0
cloud_cmd_cnt = 0

global prev_error_angle, prev_error_position, prev_waypoint_idx, prev_body_to_goal
global kp_linear, kd_linear, kp_angular, kd_angular
prev_error_position = 0
prev_error_angle = 0
prev_body_to_goal = 0
prev_waypoint_idx = -1
kp_linear = 0.5
kd_linear = 0.1
ki_linear = 0
kp_angular = 20
kd_angular = 2
ki_angular = 0

# starting guess
action = np.zeros(P.M)
action[0] = P.MAX_ACC / 2  # a
action[1] = 0.0  # delta

# Cost Matrices
Q = np.diag([20, 20, 10, 20])  # state error cost
Qf = np.diag([30, 30, 30, 30])  # state final error cost
R = np.diag([10, 10])  # input cost
R_ = np.diag([10, 10])  # input rate of change cost

mpc = mpcpy.MPC(P.N, P.M, Q, R)

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
            path_num
        ]
    )

def set_ctrl(robotId, currVel, acceleration, steeringAngle, cmd_sq_no, tag):

    # print("In set_ctrl function")
    # print(robotId, currVel, acceleration, steeringAngle)

    gearRatio = 1.0 / 21
    steering = [0, 2]
    wheels = [8, 15]
    maxForce = 50

    if tag == 'local':
        targetVelocity = acceleration
    elif tag == 'cloud':
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
    
    if cmd_sq_no==9999:
        return

    print("Rx cmd sq no: ", str(cmd_sq_no))

def plot_results(path, x_history, y_history, timeout):
    """ """
    plt.style.use("ggplot")
    plt.figure()
    plt.title("MPC Tracking Results - Socket Local+Cloud")

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
    plt.savefig(f"MQTT_Transfer_PathNum_{path_num}_Timeout_{timeout}.png")

def transform_path(path):
    idx = 0
    pid_path = []
    while idx < len(path[0]):
        curr_coord = []
        curr_coord.append(path[0][idx])
        curr_coord.append(path[1][idx])
        pid_path.append(curr_coord)
        idx = idx+1
    # print(pid_path)
    return pid_path

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

    global pid_path
    pid_path = transform_path(path)
    # print(pid_path)

    for x_, y_ in zip(path[0, :], path[1, :]):
        p.addUserDebugLine([x_, y_, 0], [x_, y_, 0.33], [0, 0, 1])

    return pid_path, path

def timer_expired (last_state_time, timeout):
    now = time.time()
    if now-last_state_time >= timeout:
        print(f"Timer({timeout}) Expired!!")
        return True
    else:
        return False

def get_distance(x1, y1, x2, y2):
	return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def get_angle(x1, y1, x2, y2):
	# return np.arctan2(y2 - y1, x2 - x1)
	return np.arctan2(y2 - y1, x2 - x1)

def find_nearest_goal(pid_path, x_pos, y_pos):
    waypoint_lst = [tuple(x) for x in pid_path]
    waypoint_lst = np.array(waypoint_lst)
    target = np.array((x_pos, y_pos))
    dist = np.linalg.norm(waypoint_lst-target, axis=1)
    min_idx = np.argmin(dist)
    goal_pt = waypoint_lst[min_idx]
    # print(type(goal_pt))
    return goal_pt

def calc_accuracy(pid_path,  x_history, y_history):
    idx=0
    err_array = np.zeros(len(x_history))
    print("length", len(x_history))
    while idx!=len(x_history):
        # print("idx", idx)
        target_waypt = find_nearest_goal(pid_path, x_history[idx], y_history[idx])
        err = get_distance(target_waypt[0], target_waypt[1], x_history[idx], y_history[idx])
        if err==0:
            err_array[idx] = 1
        elif err>0 or err<0.1:
            err_array[idx] = 0.9
        elif err>=0.1 or err<0.2:
            err_array[idx] = 0.8
        elif err>=0.2 or err<0.3:
            err_array[idx] = 0.7
        elif err>=0.3 or err<0.4:
            err_array[idx] = 0.6
        elif err>=0.4 or err<0.5:
            err_array[idx] = 0.5
        elif err>=0.5 or err<0.6:
            err_array[idx] = 0.4
        elif err>=0.6 or err<0.7:
            err_array[idx] = 0.3
        elif err>=0.7 or err<0.8:
            err_array[idx] = 0.2
        elif err>=0.8 or err<0.9:
            err_array[idx] = 0.1
        elif err>=0.9:
            err_array[idx] = 0
        idx = idx+1
    return np.average(err_array)*100

def get_pid_control_inputs(rx_state, goal_x, waypoint_idx):
    global prev_error_angle, prev_error_position, prev_waypoint_idx, prev_body_to_goal
    global kp_linear, kd_linear, kp_angular, kd_angular

    error_position = get_distance(rx_state[0], rx_state[1], goal_x[0], goal_x[1])
    
    body_to_goal = get_angle(rx_state[0], rx_state[1], goal_x[0], goal_x[1])

    # if self.prev_waypoint_idx == waypoint_idx and 350<(abs(self.prev_body_to_goal - body_to_goal)*180/np.pi):
    # 	print("HERE")
    # 	body_to_goal = self.prev_body_to_goal
    error_angle = (body_to_goal) - rx_state[3]

    linear_velocity_control = kp_linear*error_position + kd_linear*(error_position - prev_error_position)
    angular_velocity_control = kp_angular*error_angle + kd_angular*(error_angle - prev_error_angle)

    prev_error_angle = error_angle
    prev_error_position = error_position

    prev_waypoint_idx = waypoint_idx
    prev_body_to_goal = body_to_goal

    # if linear_velocity_control>0.5:
    #     linear_velocity_control = 0.5

    return linear_velocity_control, angular_velocity_control

def invoke_local_control(path, rx_state, state_sq_no):

    # print("In local control")

    local_ctrl_start = time.time()
    global pid_path, current_idx_pid, local_iter, curr_state
    local_iter = 0
    while local_iter<5:
        if len(pid_path)>0:
            goal_pt = find_nearest_goal(pid_path, rx_state[0], rx_state[1]) #pid_path[current_idx_pid] #find_nearest_goal(pid_path, rx_state) #target waypoint
            linear_v, angular_v = get_pid_control_inputs(rx_state, goal_pt, current_idx_pid)
            print("Current index", current_idx_pid)
            print("Current x,y", rx_state[0], rx_state[1])
            print("Target x,y", goal_pt[0], goal_pt[1])
            print("Linear and angular velocity", linear_v, angular_v)
            dist = get_distance(rx_state[0], rx_state[1], goal_pt[0], goal_pt[1])
            if dist<1:
                current_idx_pid+= 1
                local_iter = local_iter+1
                rx_state = get_state(car)
            elif time.time()-local_ctrl_start>10:
                print("-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n--\n-\n-\n-\n-\n-\n-\n-\n-\n-\n--")
                local_iter = 10

        global local_cmd_sq_no, local_ctrl_cmd
        local_cmd_sq_no = state_sq_no
        local_ctrl_cmd = [rx_state[2], 0.5, angular_v, local_cmd_sq_no]
        where_to_ctrl(car)
    if local_iter == 5:
        curr_state = state_rxvd
    # print("Local control command generated: "+str(local_ctrl_cmd))

def where_to_ctrl(car):

    global local_cmd_sq_no, cloud_cmd_sq_no
    global local_ctrl_cmd, cloud_ctrl_cmd
    global curr_state, last_state_time
    global local_cmd_cnt, cloud_cmd_cnt
    print("In Where to Control")
    now = time.time()

    # Latest cloud command received
    if curr_state == state_msg_proc and cloud_cmd_sq_no == state_sq_no-1:
        # Apply cloud command
        print("Applying cloud command")
        cloud_cmd_cnt = cloud_cmd_cnt + 1
        set_ctrl(car, cloud_ctrl_cmd[0], cloud_ctrl_cmd[1], cloud_ctrl_cmd[2], cloud_ctrl_cmd[3], tag='cloud')
        curr_state = state_rxvd
    # Latest local command ready
    elif curr_state == state_timeout and local_cmd_sq_no == state_sq_no-1:
        # Apply local command
        print("Applying local command")
        local_cmd_cnt = local_cmd_cnt + 1
        set_ctrl(car, local_ctrl_cmd[0], local_ctrl_cmd[1], local_ctrl_cmd[2], local_ctrl_cmd[3], tag='local')
    # If no control command is ready
    elif now>last_state_time+1:
        # Halt robot        
        print("No command ready, halting robot")
        set_ctrl(car, 0, 0, 0, 9999, tag='cloud')
        last_state_time = time.time()
        curr_state = state_rxvd

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
    global last_cmd_time, curr_state
    last_cmd_time = time.time()
    print("Message received: "+msg_decode+'\n')
    if curr_state == "end_sim":
        return
    elif curr_state == state_txd:
        curr_state = state_msg_proc
    
    global cloud_cmd_sq_no, cloud_ctrl_cmd, state_sq_no

    msg_decode = msg_decode.split("[")[1].split("]")[0]
    temp_state = list(msg_decode.split(","))
    temp_state[0]=float(temp_state[0])
    temp_state[1] = float(temp_state[1])
    temp_state[2] = float(temp_state[2])
    temp_state[3] = float(temp_state[3])
    cloud_cmd_sq_no = temp_state[3]
    state = np.asarray(temp_state)

    global cld_cmd_lat, last_state_time
    if state_sq_no-1 == cloud_cmd_sq_no:
        latency = last_cmd_time-last_state_time
        print("On message latency: ", latency)
        if latency>10000:
            return
        cld_cmd_lat.append(latency)

    cloud_ctrl_cmd = [state[0], state[1], state[2], cloud_cmd_sq_no]
    where_to_ctrl(car)
    
global client
client = connect()

def have_reached(state, path, timeout):
    global sim_start_time, pid_path
    x_history.append(state[0])
    y_history.append(state[1])

    if np.sqrt((state[0] - path[0, -1]) ** 2 + (state[1] - path[1, -1]) ** 2) < 0.2:
        print("Success! Goal Reached")
        set_ctrl(car, 0, 0, 0, 9999, tag='cloud')
        global curr_state
        curr_state = "end_sim"
        
        # Get simulation start time
        sim_end_time = time.time()
        time_taken = sim_end_time-sim_start_time
        print("Time at finish: ", sim_end_time)
        print("Total time: ", time_taken)

        plot_results(path, x_history, y_history, timeout)
        accuracy = calc_accuracy(pid_path, x_history, y_history)
        print("Accuracy: ", accuracy)
        p.disconnect()
        return accuracy, time_taken
    else:
        return None, None

def reset_sim(path_var, timeout):
    global last_state_time, counter, sim_start_time, cld_cmd_lat
    global x_history, y_history, curr_state, state_sq_no, pid_path
    global num_state_sent, local_cmd_cnt, cloud_cmd_cnt, timeout_cnt

    last_state_time = 0
    counter = 0
    x_history = []
    y_history = []
    cld_cmd_lat = []
    pid_path = []
    curr_state = state_rxvd
    state_sq_no = 1
    num_state_sent = 0
    local_cmd_cnt = 0
    cloud_cmd_cnt = 0
    timeout_cnt = 0
    # Get simulation start time
    sim_start_time = time.time()

    acc, rxvd_lat, timeout_cnt, state_sq_no = start_sim(path_var, timeout)
    return acc, cld_cmd_lat, rxvd_lat, timeout_cnt, state_sq_no

def start_sim(path_var, timeout):

    global last_state_time, curr_state, state_sq_no, sim_start_time
    global x_history, y_history, num_state_sent

    # Process path variable from str to list
    path_var = ast.literal_eval(path_var)
    pid_path, path = env_setup(path_var)

    timeout_cnt = 0
    state = get_state(car)

    sim_start_time = time.time()

    while True:
        # f.flush()
        if curr_state == state_txd and timer_expired(last_state_time, timeout):
            # print(f"Timeout: {timeout}; halting car")
            # set_ctrl(car, 0, 0, 0, 9999) # Halt car
            # last_state_time = time.time()
            timeout_cnt = timeout_cnt+1
            curr_state = state_timeout
        elif curr_state == state_timeout:
            print("State machine at: ", curr_state)
            invoke_local_control(path, state, state_sq_no-1)
            # Resend new state after 100ms
            # time.sleep(0.08)
            # curr_state = state_rxvd
        elif curr_state == state_rxvd:
            print("State machine at: ", curr_state)
            # send new state
            state = get_state(car)
            print("Sent new state",str(state))
            send(client, str(state))
            last_state_time = time.time()
            # print(last_state_time)
            state_sq_no = state_sq_no + 1
            num_state_sent = num_state_sent + 1
            if state_sq_no >= 2000:
                print("Failed! Car in bad state")
                set_ctrl(car, 0, 0, 0, 9999, tag='cloud')
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
        elif curr_state == state_msg_proc:
            print("State machine at: ", curr_state)
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