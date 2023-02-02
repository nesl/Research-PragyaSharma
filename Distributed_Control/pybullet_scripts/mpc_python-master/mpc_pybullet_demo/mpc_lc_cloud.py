import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

from mpcpy.utils import compute_path_from_wp
import mpcpy

P = mpcpy.Params()

import sys
import time
import paho.mqtt.client as mqtt
import datetime
import pybullet as p

    
ctrl_cmd = [0]
old_cmd = [0]

# Uncomment write commands to save data on file
# with open("output.txt", 'w') as f:
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
    msg_decode = str(msg.payload.decode("utf-8"))
    now = datetime.datetime.now()
    # f.write(str(now.time()))
    print(" Message received: "+msg_decode+'\n')

    msg_decode = msg_decode.split("[ ")[1].split("]")[0]
    temp_state = list(msg_decode.split(" "))
    rx_state = np.array([0.0, 0.0, 0.0, 0.0])
    # print(temp_state)
    temp_state = list(filter(lambda a: a!= '', temp_state))
    # print(temp_state)
    rx_state[0]=float(temp_state[0])
    rx_state[1] = float(temp_state[1])
    rx_state[2] = float(temp_state[2])
    rx_state[3] = float(temp_state[3])
    state = np.asarray(rx_state)
    print(state)


    # Interpolated Path to follow given waypoints
    path = compute_path_from_wp(
        [0, 3, 4, 6, 10, 11, 12, 6, 1, 0],
        [0, 0, 2, 4, 3, 3, -1, -6, -2, -2],
        P.path_tick,
    )

    # path = compute_path_from_wp(
    #     [-3, -3, -5, -3, 2, 2, 4, 6, 6, 8, 8, 4, 4, 0, -2, -4, -6, -6],
    #     [0, -2, -6, -7, -7, -2, -2, 0, 3, 5, 10, 10, 7, 7, 9, 9, 5, 2],
    #     P.path_tick,
    # )

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

    x_history.append(state[0])
    y_history.append(state[1])

    # for MPC car ref frame is used
    new_state = state
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
        A, B, C, state, target, time_horizon=P.T, verbose=False
    )

    # action = np.vstack((np.array(u_mpc.value[0,:]).flatten(),
    #              (np.array(u_mpc.value[1,:]).flatten())))

    action[:] = [u_mpc.value[0, 1], u_mpc.value[1, 1]]

    elapsed = time.time() - start
    print("CVXPY Optimization Time: {:.4f}s".format(elapsed))

    global ctrl_cmd
    global old_cmd
    old_cmd = ctrl_cmd
    ctrl_cmd = [state[2], action[0], action[1]]
    print("Control command generated: "+str(ctrl_cmd))

    if P.DT - elapsed > 0:
        time.sleep(P.DT - elapsed)
    # client.loop_start()
    # client.publish("/robot/command", "Control command received: "+str(ctrl_cmd))


    # def on_disconnect(client, userdata, flags, rc=0):
    #     now = datetime.datetime.now()
    #     f.write(str(now.time()))
    #     f.write(" Disconnection with code "+str(rc))+'\n'


# Send command to MQTT

broker = "test.mosquitto.org"

client = mqtt.Client("controller1")

client.on_connect = on_connect
client.on_log = on_log
client.on_message = on_message
# client.on_disconnect = on_disconnect

now = datetime.datetime.now()
# f.write(str(now.time()))
# f.write(" Connecting to broker "+broker+'\n')

client.connect(broker)
client.loop_start()
while True:
    
    client.subscribe("/robot/state")

    # make sure new command diff than last one
    print(str(old_cmd))
    if any(ctrl_cmd): #and old_cmd!=ctrl_cmd:
        print("Control command received in while loop: "+str(ctrl_cmd))
        client.publish("/robot/command", str(ctrl_cmd))
    
    time.sleep(3)
    # f.flush()

        # client.loop_stop()
        # client.disconnect()
        
        # f.close()

