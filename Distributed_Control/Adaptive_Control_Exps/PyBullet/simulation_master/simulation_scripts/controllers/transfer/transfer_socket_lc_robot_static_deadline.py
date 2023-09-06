import numpy as np
import matplotlib.pyplot as plt
import sys
import time
import pybullet as p
import socket
import signal
import select
import ast
from mpcpy.utils import compute_path_from_wp
import mpcpy

P = mpcpy.Params()

HEADER = 64
PORT_CLOUD = 5061
PORT_LOCAL = 5060
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
SERVER_CLOUD = "127.0.0.1"
ADDR_CLOUD = (SERVER_CLOUD, PORT_CLOUD)
SERVER_LOCAL = "127.0.0.1"
ADDR_LOCAL = (SERVER_LOCAL, PORT_LOCAL)

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
global sensing_tick
sensing_tick = 0.2
global actuation_tick
actuation_tick = 0.35

global last_state_time
last_state_time = 0
global counter
counter = 0
global x_history, y_history
x_history = []
y_history = []
global local_iter
local_iter = 0
global num_state_sent
num_state_sent = 0
global last_state_sq_no
last_state_sq_no = 1

global local_cmd_cnt, cloud_cmd_cnt
local_cmd_cnt = 0
cloud_cmd_cnt = 0

global lcl_proc_lat, lcl_net_lat, lcl_cmd_lat
global cld_proc_lat, cld_net_lat, cld_cmd_lat
lcl_proc_lat = []
lcl_net_lat = []
lcl_cmd_lat = []
cld_proc_lat = []
cld_net_lat = []
cld_cmd_lat = []

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
    global curr_state
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
    else:
        curr_state = state_rxvd

    print("Rx cmd sq no: ", str(cmd_sq_no))

def plot_results(path, x_history, y_history, timeout):
    """ """
    plt.style.use("ggplot")
    plt.figure()
    plt.title("Tracking Results - Local+Cloud")

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
    plt.savefig(f"Static_Transfer_PathNum_{path_num}_Timeout_{timeout}.png")

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
    if path_var[0] == [0, 4, 6, 8]:
        path_num = 1
    elif path_var[0] == [0, 2, 4, 8, 12, 20]:
        path_num = 2
    elif path_var[0] == [0, 1, -3, -5, -3, 2, 2, 4, 6, 6, 8, 8, 4, 4, 0, -2, -4, -6, -6]:
        path_num = 3
    elif path_var[0] == [0, 3, 4, 6, 10, 12, 10]:
        path_num = 4
    elif path_var[0] == [0, 2, 3, 5, 6, 8, 8, 10, 8, 8, 6, 5, 3, 2, 0]:
        path_num = 5

def to_sense(last_state_time):
    global sensing_tick
    # print("In to sense")
    # print("Current time = ", time.time())
    # print("Last state time = ", last_state_time)
    # print("Current time - Last state time = ", time.time() - last_state_time)
    # print("Sensing tick = ", sensing_tick)
    
    time_to_next_tick = sensing_tick - ((time.time() - last_state_time)%(sensing_tick))
    # print("Sleeping for ", time_to_next_tick)
    # time.sleep(time_to_next_tick)
    sensing_tick = 0.05

def to_actuate(last_cmd_time):
    global actuation_tick
    print("In to actuate")
    print("Current time = ", time.time())
    print("Last cmd time = ", last_cmd_time)
    print("Current time - Last cmd time = ", time.time() - last_cmd_time)
    print("Actuation tick = ", actuation_tick)
    
    time_to_next_tick = actuation_tick - ((time.time() - last_cmd_time)%(actuation_tick))
    print("Sleeping for ", time_to_next_tick)
    time.sleep(time_to_next_tick)
    actuation_tick = 0.45

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
    if now-last_state_time >= 0.08:
        print(f"Timer({timeout}) Expired!!")
        return True
    else:
        print(f"Timer({timeout}) Not Expired!!")
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

def calc_accuracy(path,  x_history, y_history):
    idx=0
    err_bound = 1
    err_array = np.zeros(len(x_history))
    trans_path = transform_path(path)
    # print(len(trans_path))
    # print(len(x_history))
    # print(trans_path)
    
    while idx!=len(x_history)-1:
        # print("idx:", idx)
        target_waypt = find_nearest_goal(trans_path, x_history[idx], y_history[idx])
        err = get_distance(target_waypt[0], target_waypt[1], x_history[idx], y_history[idx])
        # print(target_waypt[0], target_waypt[1], x_history[idx], y_history[idx], err)
        if err>err_bound:
            err = err_bound
        err_array[idx] = ((err_bound - err)/err_bound)*100
        # print(err_array[idx])
        # print("average = ", np.average(err_array))
        idx = idx+1
    return np.average(err_array)

def get_waypoints_list(trans_path):
    waypoint_lst = [tuple(x) for x in trans_path]
    print("Size before transform: ", np.shape(waypoint_lst))
    waypoint_lst = np.array(waypoint_lst)
    print("Size after transform: ", np.shape(waypoint_lst))
    return waypoint_lst

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

    if linear_velocity_control>0.5:
        linear_velocity_control = 0.5

    return linear_velocity_control, angular_velocity_control

def invoke_local_control(path, rx_state, state_sq_no):

    # print("In local control")

    local_ctrl_start = time.time()
    global pid_path, current_idx_pid, local_iter, curr_state, prev_cmd_time
    local_iter = 0
    # while local_iter<5:
    #     if len(pid_path)>0:
    goal_pt = find_nearest_goal(pid_path, rx_state[0], rx_state[1]) #pid_path[current_idx_pid] #find_nearest_goal(pid_path, rx_state) #target waypoint
    linear_v, angular_v = get_pid_control_inputs(rx_state, goal_pt, current_idx_pid)
    print("Current index", current_idx_pid)
    print("Current x,y", rx_state[0], rx_state[1])
    print("Target x,y", goal_pt[0], goal_pt[1])
    print("Linear and angular velocity", linear_v, angular_v)
    dist = get_distance(rx_state[0], rx_state[1], goal_pt[0], goal_pt[1])
    print("Dist: ", dist)
    # if dist<0.5:
    #     current_idx_pid+= 1
    #     local_iter = local_iter+1
    #     rx_state = get_state(car)
    # elif time.time()-local_ctrl_start>10:
    #     print("-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n--\n-\n-\n-\n-\n-\n-\n-\n-\n-\n--")
    #     local_iter = 10

    global local_cmd_sq_no, local_ctrl_cmd
    local_cmd_sq_no = state_sq_no
    local_ctrl_cmd = [rx_state[2], 0.5, angular_v, local_cmd_sq_no]
    # where_to_ctrl(car)

    # if local_iter == 5:
    #     curr_state = state_rxvd
    # print("Local control command generated: "+str(local_ctrl_cmd))

def to_actuate(last_cmd_time):
    global actuation_tick
    print("In to actuate")
    print("Current time = ", time.time())
    print("Last cmd time = ", last_cmd_time)
    print("Current time - Last cmd time = ", time.time() - last_cmd_time)
    print("Actuation tick = ", actuation_tick)
    
    time_to_next_tick = actuation_tick - ((time.time() - last_cmd_time)%(actuation_tick))
    print("Sleeping for ", time_to_next_tick)
    time.sleep(time_to_next_tick)
    actuation_tick = 0.85

def where_to_ctrl(car):

    global local_cmd_sq_no, cloud_cmd_sq_no
    global local_ctrl_cmd, cloud_ctrl_cmd
    global curr_state, last_state_time
    global local_cmd_cnt, cloud_cmd_cnt
    print("In Where to Control")
    now = time.time()

    # Latest cloud command received
    if cloud_cmd_sq_no == state_sq_no-1: #curr_state == state_msg_proc and
        # Apply cloud command
        print("Applying cloud command")
        cloud_cmd_cnt = cloud_cmd_cnt + 1
        set_ctrl(car, cloud_ctrl_cmd[0], cloud_ctrl_cmd[1], cloud_ctrl_cmd[2], cloud_ctrl_cmd[3], tag='cloud')
        # curr_state = state_rxvd
    # Latest local command ready
    elif local_cmd_sq_no == state_sq_no-1: #curr_state == state_timeout and 
        # Apply local command
        print("Applying local command")
        local_cmd_cnt = local_cmd_cnt + 1
        set_ctrl(car, local_ctrl_cmd[0], local_ctrl_cmd[1], local_ctrl_cmd[2], local_ctrl_cmd[3], tag='local')
        # curr_state = state_rxvd
    # If no control command is ready
    # elif now>last_state_time+1:
    #     # Halt robot        
    #     print("No command ready, halting robot")
    #     set_ctrl(car, 0, 0, 0, 9999, tag='cloud')
    #     last_state_time = time.time()
    #     curr_state = state_rxvd

def handle_timeout(sig, frame):
    print("In timeout handler!")
    signal.setitimer(signal.ITIMER_REAL, 0)
    where_to_ctrl(car)

def connect_local():
    client_local = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_local.connect(ADDR_LOCAL)
    # client_local.settimeout(0.08)
    return client_local

def connect_cloud():
    client_cloud = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_cloud.connect(ADDR_CLOUD)
    # client_cloud.settimeout(0.08)
    return client_cloud

def send_local(client_local, msg):
    global last_state_time
    message = msg.encode(FORMAT)
    msg_length = len(message)
    send_length = str(msg_length).encode(FORMAT)
    send_length += b' ' * (HEADER - len(send_length))
    client_local.send(send_length)
    client_local.send(message)
    # last_state_time = time.time()
    # print("send_time in send: ", last_state_time)

def send_cloud(client_cloud, msg):
    message = msg.encode(FORMAT)
    msg_length = len(message)
    send_length = str(msg_length).encode(FORMAT)
    send_length += b' ' * (HEADER - len(send_length))
    client_cloud.send(send_length)
    client_cloud.send(message)

def recv_local(client_local):
    msg_decode = client_local.recv(2048).decode(FORMAT)
    global last_cmd_time, curr_state
    global prev_cmd_time
    # prev_cmd_time = last_cmd_time
    last_cmd_time = time.time()
    print("Local Message received: "+msg_decode+'\n')
    if curr_state == "end_sim":
        return
    # elif curr_state == state_txd:
    #     curr_state = state_msg_proc
    
    global local_cmd_sq_no, local_ctrl_cmd, state_sq_no

    msg_decode = msg_decode.split("[")[1].split("]")[0]
    temp_state = list(msg_decode.split(","))
    temp_state[0]=float(temp_state[0])
    temp_state[1] = float(temp_state[1])
    temp_state[2] = float(temp_state[2])
    temp_state[3] = float(temp_state[3])
    temp_state[4] = float(temp_state[4])
    local_cmd_sq_no = temp_state[3]
    state = np.asarray(temp_state)

    global lcl_cmd_lat, last_state_time, lcl_proc_lat, lcl_net_lat
    if state_sq_no-1 == local_cmd_sq_no:
        lcl_proc_lat.append(state[4])
        print("send_time in recv_local: ", last_state_time)
        latency = last_cmd_time-last_state_time
        print("Local Message received latency: ", latency)
        if latency>10000:
            return
        lcl_cmd_lat.append(latency)
        net_lat_time = latency-state[4]
        lcl_net_lat.append(net_lat_time)
        print("Local Computation  latency: ", state[4])
        print("Local Network  latency: ", net_lat_time)

    local_ctrl_cmd = [state[0], state[1], state[2], local_cmd_sq_no]
    # where_to_ctrl(car)


def recv_cloud(client_cloud):
    msg_decode = client_cloud.recv(2048).decode(FORMAT)
    global last_cmd_time, curr_state
    global prev_cmd_time
    # prev_cmd_time = last_cmd_time
    last_cmd_time = time.time()
    print("Cloud Message received: "+msg_decode+'\n')
    if curr_state == "end_sim":
        return
    # elif curr_state == state_txd:
    #     curr_state = state_msg_proc
    
    global cloud_cmd_sq_no, cloud_ctrl_cmd, state_sq_no

    msg_decode = msg_decode.split("[")[1].split("]")[0]
    temp_state = list(msg_decode.split(","))
    temp_state[0]=float(temp_state[0])
    temp_state[1] = float(temp_state[1])
    temp_state[2] = float(temp_state[2])
    temp_state[3] = float(temp_state[3])
    temp_state[4] = float(temp_state[4])
    cloud_cmd_sq_no = temp_state[3]
    state = np.asarray(temp_state)

    global cld_cmd_lat, last_state_time, cld_proc_lat, cld_net_lat
    if state_sq_no-1 == cloud_cmd_sq_no:
        cld_proc_lat.append(state[4])
        print("send_time in recv_cloud: ", last_state_time)
        latency = last_cmd_time-last_state_time
        print("Cloud Message received latency: ", latency)
        if latency>10000:
            return
        cld_cmd_lat.append(latency)
        net_lat_time = latency-state[4]
        cld_net_lat.append(net_lat_time)
        print("Cloud Computation  latency: ", state[4])
        print("Cloud Network  latency: ", net_lat_time)

    cloud_ctrl_cmd = [state[0], state[1], state[2], cloud_cmd_sq_no]
    # where_to_ctrl(car)
    
global client_local, client_cloud
client_local = connect_local()
client_cloud = connect_cloud()
signal.signal(signal.SIGALRM, handle_timeout)

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
    global lcl_proc_lat, lcl_net_lat, lcl_cmd_lat
    global cld_proc_lat, cld_net_lat, cld_cmd_lat

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
    sensing_tick = 0.08
    actuation_tick = 0.45
    lcl_proc_lat = []
    lcl_net_lat = []
    lcl_cmd_lat = []
    cld_proc_lat = []
    cld_net_lat = []
    cld_cmd_lat = []
    # Get simulation start time
    sim_start_time = time.time()

    acc, rxvd_lat, timeout_cnt, state_sq_no = start_sim(path_var, timeout)
    waypoints_list = get_waypoints_list(pid_path)  
    x_waypts = []
    y_waypts = []
    for i in waypoints_list:
        x_waypts.append(i[0])
        y_waypts.append(i[1])
    return acc, cld_cmd_lat, rxvd_lat, timeout_cnt, state_sq_no, cld_proc_lat, cld_net_lat, lcl_cmd_lat, lcl_proc_lat, lcl_net_lat, x_waypts, y_waypts, x_history, y_history

def start_sim(path_var, timeout):

    global last_state_time, curr_state, state_sq_no, sim_start_time
    global x_history, y_history, num_state_sent

    # Process path variable from str to list
    path_var = ast.literal_eval(path_var)
    pid_path, path = env_setup(path_var)

    timeout_cnt = 0
    state = get_state(car)

    sim_start_time = time.time()

    sockets = [client_local, client_cloud]

    while True:
        # f.flush()
        # if curr_state == state_txd and timer_expired(last_state_time, timeout):
            # print(f"Timeout: {timeout}; halting car")
            # set_ctrl(car, 0, 0, 0, 9999) # Halt car
            # last_state_time = time.time()
            # timeout_cnt = timeout_cnt+1
            # where_to_ctrl(car)
            # curr_state = state_timeout
        # elif curr_state == state_timeout:
        #     print("State machine at: ", curr_state)
            # invoke_local_control(path, state, state_sq_no-1)
            # Resend new state after 100ms
            # time.sleep(0.08)
            # where_to_ctrl(car)
            # curr_state = state_rxvd
        if curr_state == state_rxvd:
            print("State machine at: ", curr_state)
            # send new state
            state = get_state(car)
            print("Sent new state",str(state))
            send_local(client_local, str(state))
            send_cloud(client_cloud, str(state))
            last_state_time = time.time()
            signal.setitimer(signal.ITIMER_REAL, 0.5, 0)
            print("send_time in while loop: ", last_state_time)
            state_sq_no = state_sq_no + 1
            num_state_sent = num_state_sent + 1
            # invoke_local_control(path, state, state_sq_no-1)
            curr_state = state_txd
            print("State machine at: ", curr_state)
            print('\n')
        elif curr_state == state_txd:
            # try:
            start_time = time.time()
            readable_sockets, _, _ = select.select(sockets, [], [], 0.5)
            # print(readable_sockets)
            if client_local in readable_sockets:
                recv_local(client_local)
            if client_cloud in readable_sockets:
                recv_cloud(client_cloud)
            # if time.time() - start_time > 0.1:
            #     curr_state = state_msg_proc
            # except socket.timeout:
            #     print("Timeout!!!!")
            #     timeout = last_state_time - time.time()
            #     print("Timeout at ", timeout)
            #     where_to_ctrl(car)
            global counter
            counter = counter+1
            if counter % 10 == 0:
                print("State machine at: ", curr_state)
        # elif curr_state == state_msg_proc:
        #     print("State machine at: ", curr_state)
        #     where_to_ctrl(car)
        #     counter = counter+1
        #     if counter % 10 == 0:
        #         print("State machine at: ", curr_state)
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