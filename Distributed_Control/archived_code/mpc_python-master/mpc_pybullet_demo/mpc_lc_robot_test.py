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

global curr_x, curr_y, reached_flag
curr_x = 0
curr_y = 0
reached_flag = 1

state_rxvd = "last_cmd_rxvd"
state_txd = "latest_state_txd"
state_timeout = "timeout"

global curr_state
curr_state = state_rxvd
global state_sq_no
state_sq_no = 1


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

    if cmd_sq_no!=999:
        #Don't switch state for halting
        global curr_state
        curr_state = state_rxvd

    if cmd_sq_no==999:
        return

    print("Rx cmd sq no: ", str(cmd_sq_no))
    
    # pos_check(robotId)

def plot_results(path, x_history, y_history):
    """ """
    plt.style.use("ggplot")
    plt.figure()
    plt.title("MPC Tracking Results - Cloud")

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


# with open("output2.txt", 'w') as f:
#     sys.stdout = f

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
    # f.write(str(now.time()))
    print("Message received: "+msg_decode)

    msg_decode = msg_decode.split("[")[1].split("]")[0]
    temp_state = list(msg_decode.split(","))
    temp_state[0]=float(temp_state[0])
    temp_state[1] = float(temp_state[1])
    temp_state[2] = float(temp_state[2])
    temp_state[3] = float(temp_state[3])
    state = np.asarray(temp_state)
    set_ctrl(car, state[0], state[1], state[2], state[3])
    

# def on_disconnect(client, userdata, flags, rc=0):
#     now = datetime.datetime.now()
#     f.write(str(now.time()))
#     f.write(" Disconnection with code "+str(rc))+'\n'


broker = "127.0.0.1"

client = mqtt.Client("Robot")

client.on_connect = on_connect
client.on_log = on_log
client.on_message = on_message
# client.on_disconnect = on_disconnect

now = datetime.datetime.now()
# f.write(str(now.time()))
# f.write(" Connecting to broker "+broker+'\n')

client.connect(broker)

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

path = compute_path_from_wp(
    [0, 6, 28, 28.5, 25, 26],
    [0, 7.5, -5.5, 0.8, 1.8, 6.8],
    P.path_tick,
)

# path = compute_path_from_wp(
#     [-3, -3, -5, -3, 2, 2, 4, 6, 6, 8, 8, 4, 4, 0, -2, -4, -6, -6],
#     [0, -2, -6, -7, -7, -2, -2, 0, 3, 5, 10, 10, 7, 7, 9, 9, 5, 2],
#     P.path_tick,
# )

for x_, y_ in zip(path[0, :], path[1, :]):
    p.addUserDebugLine([x_, y_, 0], [x_, y_, 0.33], [0, 0, 1])

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

# time.sleep(0.5)

client.loop_start()

# Get simulation start time
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

counter = 0

while True:
    # f.flush()
    
    if curr_state == state_txd and timer_expired(last_state_time):
        print("Timeout: halting car")
        set_ctrl(car, 0, 0, 0, 999) # Halt car
        # last_state_time = time.time()
        curr_state = state_timeout
    elif curr_state == state_timeout:
        print("State machine at: ", curr_state)
        # Resend new state after one second
        time.sleep(1)
        curr_state = state_rxvd
    elif curr_state == state_rxvd:
        print("State machine at: ", curr_state)
        # send new state
        state = get_state(car)
        print("Sent new state",str(state))
        client.publish("/robot/state", str(state))
        last_state_time = time.time()
        state_sq_no = state_sq_no + 1
        curr_state = state_txd
        print("State machine at: ", curr_state)
        print('\n')
    elif curr_state == state_txd:
        client.subscribe("/robot/command")
        counter = counter+1
        if counter % 10 == 0:
            print("State machine at: ", curr_state)
        # wait for new cmd to arrive
        # time.sleep(3.5)

    # track path in bullet
    p.addUserDebugLine(
        [state[0], state[1], 0], [state[0], state[1], 0.5], [1, 0, 0]
    )

    x_history.append(state[0])
    y_history.append(state[1])

    if np.sqrt((state[0] - path[0, -1]) ** 2 + (state[1] - path[1, -1]) ** 2) < 0.4:
        print("Success! Goal Reached")
        set_ctrl(car, 0, 0, 0, 999)
        curr_state = "end_sim"
        
        # Get simulation start time
        sim_end_time = time.time()
        time_taken = sim_end_time-sim_start_time
        print("Time at finish: ", sim_end_time)
        print("Total time: ", time_taken)

        plot_results(path, x_history, y_history)
        p.disconnect()

    # # for MPC car ref frame is used
    # state[0:2] = 0.0
    # state[3] = 0.0
    # print("State after initialization in while: ", state)
    # print(type(state))

    # # add 1 timestep delay to input
    # state[0] = state[0] + state[2] * np.cos(state[3]) * P.DT
    # state[1] = state[1] + state[2] * np.sin(state[3]) * P.DT
    # state[2] = state[2] + action[0] * P.DT
    # state[3] = state[3] + action[0] * np.tan(action[1]) / P.L * P.DT
    # print("State after differential eqs in while: ", state)

    # # optimization loop
    # start = time.time()

    # # State Matrices
    # A, B, C = mpcpy.get_linear_model_matrices(state, action)
    # print("A: ", A)
    # print("B: ", B)
    # print("C: ", C)

    # # Get Reference_traj -> inputs are in worldframe
    # target, _ = mpcpy.get_ref_trajectory(get_state(car), path, 1.0)
    # print("Ref trajectory computed: ", target)
    # get_state(car)
    # # print(get_state(car))

    # x_mpc, u_mpc = mpc.optimize_linearized_model(
    #     A, B, C, state, target, time_horizon=P.T, verbose=False
    # )
    # print("Minimized U: ", u_mpc.value)
    

    # # action = np.vstack((np.array(u_mpc.value[0,:]).flatten(),
    # #              (np.array(u_mpc.value[1,:]).flatten())))

    # action[:] = [u_mpc.value[0, 1], u_mpc.value[1, 1]]
    # print("Final action value: ", action)

    # set_ctrl(car, state[2], action[0], action[1])


    # elapsed = time.time()
    # print("CVXPY Optimization Time: {:.4f}s".format(elapsed))

    # if P.DT - elapsed > 0:
    #     time.sleep(P.DT - elapsed)


    # time.sleep(4)
    # f.flush()

        # client.loop_stop()
        # client.disconnect()
        
        # f.close()