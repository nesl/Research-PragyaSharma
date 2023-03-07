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


state_rxvd = "last_cmd_rxvd"
state_txd = "latest_state_txd"
state_timeout = "timeout"

global curr_state
curr_state = state_rxvd
global state_sq_no
state_sq_no = 1
global cloud_cmd_sq_no, local_cmd_sq_no
cloud_cmd_sq_no = 9999
local_cmd_sq_no = 9999
global local_ctrl_cmd
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

global local_cmd_cnt, cloud_cmd_cnt
local_cmd_cnt = 0
cloud_cmd_cnt = 0

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
            targetVelocity=targetVelocity/gearRatio,
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

    print("Rx cmd sq no: ", str(cmd_sq_no))

def plot_results(path, x_history, y_history, timeout):
    """ """
    plt.style.use("ggplot")
    plt.figure()
    plt.title("MPC Tracking Results - Local+Cloud")

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

def compute_local_cmd(path, rx_state, state_sq_no):

    # print("In local control")

    state = np.asarray(rx_state[0:4])

    # for MPC car ref frame is used
    new_state = np.copy(state)
    new_state[0:2] = 0.0
    new_state[3] = 0.0

    # add 1 timestep delay to input
    # print(new_state)
    new_state[0] = new_state[0] + new_state[2] * np.cos(new_state[3]) * P.DT
    new_state[1] = new_state[1] + new_state[2] * np.sin(new_state[3]) * P.DT
    new_state[2] = new_state[2] + action[0] * P.DT
    new_state[3] = new_state[3] + action[0] * np.tan(action[1]) / P.L * P.DT

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

    global local_cmd_sq_no, local_ctrl_cmd
    local_cmd_sq_no = state_sq_no
    local_ctrl_cmd = [state[2], action[0], action[1], local_cmd_sq_no]
    # print("Local control command generated: "+str(local_ctrl_cmd))

def where_to_ctrl(car):

    global local_cmd_sq_no, cloud_cmd_sq_no
    global local_ctrl_cmd, cloud_ctrl_cmd
    global curr_state, last_state_time
    global local_cmd_cnt, cloud_cmd_cnt
    print("In Where to Control")
    now = time.time()

    # Latest cloud command received
    if cloud_cmd_sq_no == state_sq_no-1:
        # Apply cloud command
        print("Applying cloud command")
        cloud_cmd_cnt = cloud_cmd_cnt + 1
        set_ctrl(car, cloud_ctrl_cmd[0], cloud_ctrl_cmd[1], cloud_ctrl_cmd[2], cloud_ctrl_cmd[3])
    # Latest local command ready
    elif local_cmd_sq_no == state_sq_no-1:
        # Apply local command
        print("Applying local command")
        local_cmd_cnt = local_cmd_cnt + 1
        set_ctrl(car, local_ctrl_cmd[0], local_ctrl_cmd[1], local_ctrl_cmd[2], local_ctrl_cmd[3])
    # If no control command is ready
    elif now>last_state_time+1:
        # Halt robot        
        print("No command ready, halting robot")
        set_ctrl(car, 0, 0, 0, 9999)
        last_state_time = time.time()
        curr_state = state_rxvd

def on_log(client, userdata, level, buf):
    now = datetime.datetime.now()
    # f.write(str(now.time()))
    # f.write(" log: "+buf+'\n')

def on_connect(client, userdata, flags, rc):
    if rc==0:
        now = datetime.datetime.now()
        # f.write(str(now.time()))
        # f.write(" Connection ok"+'\n')
    else:
        now = datetime.datetime.now()
        # f.write(str(now.time()))
        # f.write(" Bad connection, exited with code "+str(rc)+'\n')

def on_message(client, userdata, msg):
    topic = msg.topic

    if curr_state == "end_sim":
        return
    msg_decode = str(msg.payload.decode("utf-8"))
    global last_cmd_time
    last_cmd_time = time.time()
    print("Message received: "+msg_decode+'\n')

    global cloud_cmd_sq_no, cloud_ctrl_cmd   

    msg_decode = msg_decode.split("[")[1].split("]")[0]
    temp_state = list(msg_decode.split(","))
    temp_state[0]=float(temp_state[0])
    temp_state[1] = float(temp_state[1])
    temp_state[2] = float(temp_state[2])
    temp_state[3] = float(temp_state[3])
    cloud_cmd_sq_no = temp_state[3]
    state = np.asarray(temp_state)

    global cld_cmd_lat
    latency = last_cmd_time-last_state_time
    print("On message latency: ", latency)
    cld_cmd_lat.append(latency)

    cloud_ctrl_cmd = [state[0], state[1], state[2], cloud_cmd_sq_no]
    where_to_ctrl(car)

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

    for x_, y_ in zip(path[0, :], path[1, :]):
        p.addUserDebugLine([x_, y_, 0], [x_, y_, 0.33], [0, 0, 1])

    return path

def timer_expired (last_state_time, timeout):
    now = time.time()
    if now-last_state_time >= 0.6:
        print(f"Timer({timeout}) Expired!!")
        return True
    else:
        return False

broker = "127.0.0.1"

client = mqtt.Client("Robot")

client.on_connect = on_connect
client.on_log = on_log
client.on_message = on_message
# client.on_disconnect = on_disconnect

client.connect(broker)

client.loop_start()

def reset_sim(path_var, timeout):
    global last_state_time, counter, sim_start_time, cld_cmd_lat
    global x_history, y_history, curr_state, state_sq_no
    global local_cmd_cnt, cloud_cmd_cnt, timeout_cnt

    last_state_time = 0
    counter = 0
    x_history = []
    y_history = []
    cld_cmd_lat = []
    curr_state = state_rxvd
    state_sq_no = 1
    local_cmd_cnt = 0
    cloud_cmd_cnt = 0
    timeout_cnt = 0
    # Get simulation start time
    sim_start_time = time.time()

    rxvd_lat, timeout_cnt, state_sq_no = start_sim(path_var, timeout)
    return cld_cmd_lat, rxvd_lat, timeout_cnt, state_sq_no, local_cmd_cnt, cloud_cmd_cnt


def start_sim(path_var, timeout):

    global last_state_time, curr_state, state_sq_no, sim_start_time

    # Process path variable from str to list
    path_var = ast.literal_eval(path_var)
    path = env_setup(path_var)

    timeout_cnt = 0
    state = get_state(car)

    sim_start_time = time.time()

    while True:
        # f.flush()

        state = get_state(car)
        
        # track path in bullet
        p.addUserDebugLine(
            [state[0], state[1], 0], [state[0], state[1], 0.5], [1, 0, 0]
        )

        x_history.append(state[0])
        y_history.append(state[1])

        if np.sqrt((state[0] - path[0, -1]) ** 2 + (state[1] - path[1, -1]) ** 2) < 0.4:
            print("Success! Goal Reached")
            set_ctrl(car, 0, 0, 0, 9999)
            curr_state = "end_sim"
            
            # Get simulation end time
            sim_end_time = time.time()
            time_taken = sim_end_time-sim_start_time
            print("Time at finish: ", sim_end_time)
            print("Total time: ", time_taken)

            plot_results(path, x_history, y_history, timeout)
            p.disconnect()
            return time_taken, timeout_cnt, state_sq_no

        if curr_state == "latest_state_txd" and timer_expired(last_state_time, timeout):
            timeout_cnt = timeout_cnt + 1
            # curr_state = state_rxvd
            where_to_ctrl(car)
        # elif curr_state == state_timeout:
        #     print("State machine at: ", curr_state)
        #     # Resend new state after 100ms
        #     time.sleep(0.08)
        #     curr_state = state_rxvd
        elif curr_state == "last_cmd_rxvd":
            print("State machine at: ", curr_state)
            # send new state
            state = get_state(car)
            last_state = state
            print("Sent new state",str(state))
            client.publish("/robot/state", str(state), 2)
            last_state_time = time.time()
            print("Last state time: ",last_state_time)
            state_sq_no = state_sq_no + 1
            if state_sq_no >= 2000:
                print("Failed! Car in bad state")
                set_ctrl(car, 0, 0, 0, 9999)
                curr_state = "end_sim"
                plot_results(path, x_history, y_history, timeout)
                p.disconnect()
                time_taken = None
                return time_taken, timeout_cnt, state_sq_no
            
            # also compute local command
            # compute_local_cmd(path, state, state_sq_no-1)
            curr_state = state_txd
            print("State machine at: ", curr_state)
            print('\n')
        elif curr_state == "latest_state_txd":
            client.subscribe("/robot/command")
            global counter
            counter = counter+1
            if counter % 10 == 0:
                print("State machine at: ", curr_state)
            # wait for new cmd to arrive
            # time.sleep(3.5)

if __name__ == "__main__":
    path = str(sys.argv[1])
    timeout = float(sys.argv[2])
    start_sim(path, timeout)