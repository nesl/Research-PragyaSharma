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

global prev_error_angle, prev_error_position, prev_waypoint_idx, prev_body_to_goal
global kp_linear, kd_linear, kp_angular, kd_angular
prev_error_position = 0
prev_error_angle = 0
prev_body_to_goal = 0
prev_waypoint_idx = -1
kp_linear = 0.5
kd_linear = 0.1
ki_linear = 0
kp_angular = 2
kd_angular = 0.5
ki_angular = 0

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

    targetVelocity = acceleration
    #currVel + acceleration * P.DT
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
    plt.title("PID Tracking Result - Local")

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
    plt.savefig(f"PID_Local_PathNum_{path_num}.png")

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
    if path_var[0] == [0, 6, 28, 28.5]:
        path_num = 1
    elif path_var[0] == [0, 2, 4, 8, 12, 20]:
        path_num = 2
    elif path_var[0] == [0, 1, -3, -5, -3, 2, 2, 4, 6, 6, 8, 8, 4, 4, 0, -2, -4, -6, -6]:
        path_num = 3
    elif path_var[0] == [0, 3, 4, 6, 10, 12, 10]:
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

    pid_path = transform_path(path)
    # print(pid_path)

    for x_, y_ in zip(path[0, :], path[1, :]):
        p.addUserDebugLine([x_, y_, 0], [x_, y_, 0.33], [0, 0, 1])

    return pid_path, path

def get_distance(x1, y1, x2, y2):
	return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def get_angle(x1, y1, x2, y2):
	# return np.arctan2(y2 - y1, x2 - x1)
	return np.arctan2(y2 - y1, x2 - x1)

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

    return 0.5, angular_velocity_control

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
    pid_path, path = env_setup(path_var)

    current_idx_pid = 0
    sim_start_time = time.time()

    while True:
        state = get_state(car)
        # print("State in while loop: ", state)
        # f.write("Current state in while loop: "+str(state)+'\n')
        new_x = state[0]
        new_y = state[1]
        x_history.append(new_x)
        y_history.append(new_y)
        # f.write("state[0]: "+str(state[0])+" new_x: "+str(new_x)+'\n')
        # f.write("state[1]: "+str(state[1])+" new_x: "+str(new_y)+'\n')

        # track path in bullet
        p.addUserDebugLine(
            [state[0], state[1], 0], [state[0], state[1], 0.5], [1, 0, 0]
        )

        if np.sqrt((state[0] - path[0, -1]) ** 2 + (state[1] - path[1, -1]) ** 2) < 0.5:
            print("Success! Goal Reached")
            set_ctrl(car, 0, 0, 0)
            total_time = time.time() - sim_start_time
            print("Total time", total_time)
            accuracy = calc_accuracy(pid_path, x_history, y_history)
            print("Accuracy: ", accuracy)
            plot_results(path, x_history, y_history)
            p.disconnect()
            return accuracy, cmd_lat, total_time, state_sq_no

        state = get_state(car) #returns x,y coordinates
        last_state_time = time.time()
        state_sq_no = state_sq_no+1
        if len(pid_path)>0 and current_idx_pid != len(pid_path):
            goal_pt = pid_path[current_idx_pid] #target waypoint
            print(type(goal_pt))
            linear_v, angular_v = get_pid_control_inputs(state, goal_pt, current_idx_pid)
            ctrl_time = time.time()
            cmd_lat.append(ctrl_time-last_state_time)
            set_ctrl(car, state[2], linear_v, angular_v)
            # track path in bullet
            p.addUserDebugLine(
                [state[0], state[1], 0], [state[0], state[1], 0.5], [1, 0, 0]
            )
            print("Current index", current_idx_pid)
            print("Current x,y", state[0], state[1])
            print("Target x,y", goal_pt[0], goal_pt[1])
            print("Linear and angular velocity", linear_v, angular_v)
            dist = get_distance(state[0], state[1], goal_pt[0], goal_pt[1])
            print("dist: ", dist)
            if dist<0.8:
                current_idx_pid+= 1
        # else:
        #     linear_v = 0
        #     angular_v = 0
        # ctrl_time = time.time()
        # cmd_lat.append(ctrl_time-last_state_time)
        # set_ctrl(car, state[2], linear_v, angular_v)

if __name__ == "__main__":
    path = str(sys.argv[1])
    reset_sim(path)
