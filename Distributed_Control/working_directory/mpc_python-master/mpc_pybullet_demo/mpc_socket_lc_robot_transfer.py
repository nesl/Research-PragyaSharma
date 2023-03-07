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

HEADER = 64
PORT = 5059
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
SERVER = "54.241.142.188"#13.52.247.117"
ADDR = (SERVER, PORT)

global lat_list
lat_list = []
global curr_x, curr_y, reached_flag
curr_x = 0
curr_y = 0
reached_flag = 1
states = ["last_cmd_rxvd", "latest_state_txd", "timeout"]
global curr_state
curr_state = states[0]
global state_sq_no
state_sq_no = 0
global cloud_cmd_sq_no, local_cmd_sq_no
cloud_cmd_sq_no = 999
local_cmd_sq_no = 999
global local_ctrl_cmd

# Interpolated Path to follow given waypoints
path = compute_path_from_wp(
    [0, 6, 28, 28.5, 25, 26],
    [0, 7.5, -5.5, 0.8, 1.8, 6.8],
    P.path_tick,
)

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
x_history = []
y_history = []

def get_state(robotId):
    """ """
    global state_sq_no
    state_sq_no = state_sq_no+1

    robPos, robOrn = p.getBasePositionAndOrientation(robotId)
    linVel, angVel = p.getBaseVelocity(robotId)

    return np.array(
        [
            robPos[0],
            robPos[1],
            np.sqrt(linVel[0] ** 2 + linVel[1] ** 2),
            p.getEulerFromQuaternion(robOrn)[2],
            state_sq_no,
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
    
    if cmd_sq_no!=999:
        #Don't switch state for halting
        global curr_state
        curr_state = states[0]
    
    if cmd_sq_no==999:
        return

    print("Rx cmd sq no: ", str(cmd_sq_no))

def invoke_local_ctrl(car, state):

    print("In local control")

    # time.sleep(0.5)

    # x_history.append(state[0])
    # y_history.append(state[1])

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
    new_state[0] = new_state[0] + new_state[2] * np.cos(new_state[3]) * P.DT
    new_state[1] = new_state[1] + new_state[2] * np.sin(new_state[3]) * P.DT
    new_state[2] = new_state[2] + action[0] * P.DT
    new_state[3] = new_state[3] + action[0] * np.tan(action[1]) / P.L * P.DT

    # optimization loop
    start = time.time()

    # State Matrices
    A, B, C = mpcpy.get_linear_model_matrices(new_state, action)

    # Get Reference_traj -> inputs are in worldframe
    target, _ = mpcpy.get_ref_trajectory(state, path, 1.0)

    x_mpc, u_mpc = mpc.optimize_linearized_model(
        A, B, C, new_state, target, time_horizon=P.T, verbose=False
    )

    # action = np.vstack((np.array(u_mpc.value[0,:]).flatten(),
    #              (np.array(u_mpc.value[1,:]).flatten())))

    action[:] = [u_mpc.value[0, 1], u_mpc.value[1, 1]]

    global ctrl_cmd
    # global old_cmd
    # old_cmd = ctrl_cmd
    global local_cmd_sq_no
    global local_ctrl_cmd
    local_cmd_sq_no = state_sq_no
    ctrl_cmd = [state[2], action[0], action[1], local_cmd_sq_no]
    local_ctrl_cmd = [state[2], action[0], action[1], local_cmd_sq_no]
    print("Local control command generated: "+str(ctrl_cmd))
    # set_ctrl(car, state[2], action[0], action[1], local_cmd_sq_no)

def where_to_ctrl(car, last_state):

    global local_cmd_sq_no, cloud_cmd_sq_no
    global local_ctrl_cmd, cloud_ctrl_cmd
    global curr_state
    global last_state_time
    print("In Where to Control")
    now = time.time()

    # Latest cloud command received
    if cloud_cmd_sq_no == state_sq_no:
        # Apply cloud command
        print("Applying cloud command")
        set_ctrl(car, cloud_ctrl_cmd[0], cloud_ctrl_cmd[1], cloud_ctrl_cmd[2], cloud_ctrl_cmd[3])
    # Latest local command ready
    elif local_cmd_sq_no == state_sq_no:
        # Apply local command
        print("Applying local command")
        set_ctrl(car, local_ctrl_cmd[0], local_ctrl_cmd[1], local_ctrl_cmd[2], local_ctrl_cmd[3])
    # If no control command is ready
    elif now>last_state_time+1:
        # Halt robot        
        print("No command ready, halting robot")
        set_ctrl(car, 0, 0, 0, 999)
        last_state_time = time.time()
        curr_state = states[0]

    # Handle default controller
    # if avg_lat>3:
    #     print("Local control")
    #     state = get_state(car)
    #     invoke_local_ctrl(car, state)
    # else:
    #     print("Cloud control")
    #     set_ctrl(car, state[0], state[1], state[2], state[3])


def plot_results(path, x_history, y_history):
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
    plt.show()


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
    pub_time = datetime.datetime.now()
    
    # time.sleep(0.5)
    msg_decode = client.recv(2048).decode(FORMAT)
    now = datetime.datetime.now()
    print("Message received: "+msg_decode+'\n')

    msg_decode = msg_decode.split("[")[1].split("]")[0]
    temp_state = list(msg_decode.split(","))
    temp_state[0]=float(temp_state[0])
    temp_state[1] = float(temp_state[1])
    temp_state[2] = float(temp_state[2])
    temp_state[3] = float(temp_state[3])
    global cloud_cmd_sq_no
    cloud_cmd_sq_no = temp_state[3]
    state = np.asarray(temp_state)
    global cloud_ctrl_cmd
    cloud_ctrl_cmd = [state[0], state[1], state[2], cloud_cmd_sq_no]

    global avg_lat

    latency = now - pub_time
    latency = latency.total_seconds()
    print("latency: ",latency)
    lat_list.append(latency)
    avg_lat = np.average(lat_list)
    print("avg latency: ",avg_lat)
    where_to_ctrl(car, cloud_ctrl_cmd)
    # set_ctrl(car, state[0], state[1], state[2])

    
global client
client = connect()

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

# Interpolated Path to follow given waypoints
# path = compute_path_from_wp(
#     [0, 3, 4, 6, 10, 11, 12, 6, 1, 0],
#     [0, 0, 2, 4, 3, 3, -1, -6, -2, -2],
#     P.path_tick,
# )

# Get simulation start time


for x_, y_ in zip(path[0, :], path[1, :]):
    p.addUserDebugLine([x_, y_, 0], [x_, y_, 0.33], [0, 0, 1])


sim_start_time = time.time()

global last_state_time
last_state_time = 0

def timer_expired (last_state_time):
    now = time.time()
    if now-last_state_time >= 0.5:
        print("Timer Expired!!")
        return True
    else:
        return False

global last_state

def have_reached(state):
    x_history.append(state[0])
    y_history.append(state[1])

    if np.sqrt((state[0] - path[0, -1]) ** 2 + (state[1] - path[1, -1]) ** 2) < 0.2:
        print("Success! Goal Reached")
        set_ctrl(car, 0, 0, 0, 999)
        global curr_state
        curr_state = "end_sim"
        
        # Get simulation start time
        sim_end_time = time.time()
        time_taken = sim_end_time-sim_start_time
        print("Time at finish: ", sim_end_time)
        print("Total time: ", time_taken)

        plot_results(path, x_history, y_history)
        p.disconnect()

while True:
    # f.flush()
    state = get_state(car)
    have_reached(state)
    last_state_time = time.time()
    send(client, str(state))
    print("Sent new state",str(state))


    # time.sleep(3)

    # track path in bullet
    p.addUserDebugLine(
        [state[0], state[1], 0], [state[0], state[1], 0.5], [1, 0, 0]
    )

    # if timer_expired(last_state_time):
    #     # print("Halting car")
    #     # set_ctrl(car, 0, 0, 0, 999) # Halt car
    #     # last_state_time = time.time()
    #     # curr_state = states[2]
    #     where_to_ctrl(car, state)

    # time.sleep(4)
    # f.flush()

        # client.loop_stop()
        # client.disconnect()
        
        # f.close()