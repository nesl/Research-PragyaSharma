import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

from mpcpy.utils import compute_path_from_wp
import mpcpy

P = mpcpy.Params()

import sys
import time

import pybullet as p
import time
import ast

global last_state_time
last_state_time = 0
global x_history, y_history
x_history = []
y_history = []
global cmd_lat
cmd_lat = []
global state_sq_no
state_sq_no = 1
global path_num
path_num=1
global sim_start_time
sim_start_time = 0

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

    gearRatio = 1.0 / 21
    steering = [0, 2]
    wheels = [8, 15]
    maxForce = 50

    targetVelocity = currVel + acceleration * P.DT
    # targetVelocity=lastVel
    # print(targetVelocity)

    for wheel in wheels:
        p.setJointMotorControl2(
            robotId,
            wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=targetVelocity / gearRatio,
            force=maxForce,
        )

    for steer in steering:
        p.setJointMotorControl2(
            robotId, steer, p.POSITION_CONTROL, targetPosition=steeringAngle
        )

def plot_results(path, x_history, y_history):
    """ """
    plt.style.use("ggplot")
    plt.figure()
    plt.title("MPC Tracking Result - Local")

    plt.plot(
        path[0, :], path[1, :], c="tab:red", marker=".", label="Expected track"
    )
    plt.plot(
        x_history,
        y_history,
        c="tab:blue",
        marker=".",
        alpha=0.5,
        label="Vehicle track",
    )
    plt.axis("equal")
    plt.legend()
    # plt.show()
    plt.savefig(f"MPC_Local_PathNum_{path_num}.png")

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
    err_bound = 2
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

def reset_sim(path_var):
    global last_state_time, sim_start_time, cmd_lat
    global x_history, y_history, state_sq_no

    last_state_time = 0
    x_history = []
    y_history = []
    cmd_lat = []
    state_sq_no = 1
    # Get simulation start time
    sim_start_time = time.time()

    acc, cmd_lat, rxvd_lat, state_sq_no = run_sim(path_var)
    return acc, cmd_lat, rxvd_lat, state_sq_no

def run_sim(path_var):
    """ """

    global last_state_time, state_sq_no, sim_start_time, cmd_lat

    # Process path variable from str to list
    path_var = ast.literal_eval(path_var)
    path = env_setup(path_var)

    # starting guess
    action = np.zeros(P.M)
    action[0] = P.MAX_ACC / 2  # a
    action[1] = 0.0  # delta
    # print("Initial action: ", action)

    # Cost Matrices
    Q = np.diag([20, 20, 10, 20])  # state error cost
    Qf = np.diag([30, 30, 30, 30])  # state final error cost
    R = np.diag([10, 10])  # input cost
    R_ = np.diag([10, 10])  # input rate of change cost

    mpc = mpcpy.MPC(P.N, P.M, Q, R)

    sim_start_time = time.time()

    while True:
        state = get_state(car)
        print("State in while loop: ", state)
        last_state_time = time.time()
        state_sq_no = state_sq_no+1

        new_x = state[0] 
        new_y = state[1]
        x_history.append(new_x)
        y_history.append(new_y)

        # track path in bullet
        p.addUserDebugLine(
            [state[0], state[1], 0], [state[0], state[1], 0.5], [1, 0, 0]
        )

        if np.sqrt((state[0] - path[0, -1]) ** 2 + (state[1] - path[1, -1]) ** 2) < 0.2:
            print("Success! Goal Reached")
            set_ctrl(car, 0, 0, 0)
            total_time = time.time() - sim_start_time
            print("Total elapsed time", total_time)
            plot_results(path, x_history, y_history)
            accuracy = calc_accuracy(path, x_history, y_history)
            print("Accuracy: ", accuracy)
            p.disconnect()
            return accuracy, cmd_lat, total_time, state_sq_no

        last_state_time = time.time()
        # for MPC car ref frame is used
        state[0:2] = 0.0
        state[3] = 0.0
        # print("State after initialization in while: ", state)
        # print(type(state))

        # add 1 timestep delay to input
        state[0] = state[0] + state[2] * np.cos(state[3]) * P.DT
        state[1] = state[1] + state[2] * np.sin(state[3]) * P.DT
        state[2] = state[2] + action[0] * P.DT
        state[3] = state[3] + action[0] * np.tan(action[1]) / P.L * P.DT
        # print("State after differential eqs in while: ", state)

        # State Matrices
        A, B, C = mpcpy.get_linear_model_matrices(state, action)
        # print("A: ", A)
        # print("B: ", B)
        # print("C: ", C)

        # Get Reference_traj -> inputs are in worldframe
        target, _ = mpcpy.get_ref_trajectory(get_state(car), path, 1.0)
        # print("Ref trajectory computed: ", target)
        get_state(car)
        # print(get_state(car))

        x_mpc, u_mpc = mpc.optimize_linearized_model(
            A, B, C, state, target, time_horizon=P.T, verbose=False
        )
        # print("Minimized U: ", u_mpc.value)
        

        # action = np.vstack((np.array(u_mpc.value[0,:]).flatten(),
        #              (np.array(u_mpc.value[1,:]).flatten())))

        action[:] = [u_mpc.value[0, 1], u_mpc.value[1, 1]]
        # print("Final action value: ", action)

        ctrl_time = time.time()
        cmd_lat.append(ctrl_time-last_state_time)
        set_ctrl(car, state[2], action[0], action[1])

    
if __name__ == "__main__":
    path = str(sys.argv[1])
    reset_sim(path)
