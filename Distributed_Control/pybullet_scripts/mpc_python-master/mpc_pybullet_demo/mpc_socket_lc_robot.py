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
PORT = 5051
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
SERVER = "127.0.1.1"#13.52.247.117"
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
        ]
    )


def set_ctrl(robotId, currVel, acceleration, steeringAngle):

    print("In set_ctrl function")
    print(robotId, currVel, acceleration, steeringAngle)

    gearRatio = 1.0 / 21
    steering = [0, 2]
    wheels = [8, 15]
    maxForce = 50

    targetVelocity = (currVel + acceleration * P.DT)
    # targetVelocity=lastVel
    print(targetVelocity)

    for wheel in wheels:
        p.setJointMotorControl2(
            robotId,
            wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=targetVelocity*10, #/ gearRatio,
            force=100,
        )

    for steer in steering:
        p.setJointMotorControl2(
            robotId, steer, p.POSITION_CONTROL, targetPosition=steeringAngle
        )


def plot_results(path, x_history, y_history):
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
    
    time.sleep(4)
    msg_decode = client.recv(2048).decode(FORMAT)
    print("Message received: "+msg_decode+'\n')

    msg_decode = msg_decode.split("[")[1].split("]")[0]
    temp_state = list(msg_decode.split(","))
    temp_state[0]=float(temp_state[0])
    temp_state[1] = float(temp_state[1])
    temp_state[2] = float(temp_state[2])
    state = np.asarray(temp_state)
    set_ctrl(car, state[0], state[1], state[2])

    
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

time.sleep(0.5)

while True:
    # f.flush()
    state = get_state(car)
    send(client, str(state))
    print("Sent new state",str(state))


    time.sleep(3)

    # track path in bullet
    p.addUserDebugLine(
        [state[0], state[1], 0], [state[0], state[1], 0.5], [1, 0, 0]
    )

    if np.sqrt((state[0] - path[0, -1]) ** 2 + (state[1] - path[1, -1]) ** 2) < 0.2:
        print("Success! Goal Reached")
        set_ctrl(car, 0, 0, 0)
        plot_results(path, x_history, y_history)
        p.disconnect()


    elapsed = time.time()
    print("CVXPY Optimization Time: {:.4f}s".format(elapsed))

    if P.DT - elapsed > 0:
        time.sleep(P.DT - elapsed)


    # time.sleep(4)
    # f.flush()

        # client.loop_stop()
        # client.disconnect()
        
        # f.close()