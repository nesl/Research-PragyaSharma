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
    plt.show()


def run_sim():
    """ """
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

    car = p.loadURDF("racecar/f10_racecar/racecar_differential.urdf", [-15, 0, 0.3])
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
        [-15, -9, 13, 13.5, 10, 11],
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

    # path = compute_path_from_wp(
    #     [-12, -9, -8, 13, 13.5, 10, 11],
    #     [0, 6.2, 7, -5.5, 0.8, 1.8, 6.8],
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
    input("\033[92m Press Enter to continue... \033[0m")

    start_time = time.time()

    with open("commands.txt", 'w') as f:

        while 1:
            f.flush()
            state = get_state(car)
            # f.write("Current state: "+str(state)+'\n')
            new_x = state[0] #+ state[0]*0.02*np.random.uniform(0,1)
            # if state[0]<-10:
            #     new_x = state[0] + state[0]*0.01*np.random.uniform(0,1)
            # if state[0]>10:
            #     new_x = state[0] + state[0]*0.01*np.random.uniform(0,1)
            new_y = state[1] #+ state[1]*0.02*np.random.uniform(0,1)
            x_history.append(new_x)
            y_history.append(new_y)
            f.write("state[0]: "+str(state[0])+" new_x: "+str(new_x)+'\n')
            f.write("state[1]: "+str(state[1])+" new_x: "+str(new_y)+'\n')

            # track path in bullet
            p.addUserDebugLine(
                [state[0], state[1], 0], [state[0], state[1], 0.5], [1, 0, 0]
            )

            if np.sqrt((state[0] - path[0, -1]) ** 2 + (state[1] - path[1, -1]) ** 2) < 0.2:
                print("Success! Goal Reached")
                set_ctrl(car, 0, 0, 0)
                total_time = time.time() - start_time
                print("Total elapsed time", total_time)
                path = compute_path_from_wp(
                [-15, -9, 13, 13.5, 10, 11],
                [0, 7.5, -5.5, 0.8, 1.8, 6.8],
                P.path_tick,)
                plot_results(path, x_history, y_history)
                input("Press Enter to continue...")
                p.disconnect()
                return

            # for MPC car ref frame is used
            state[0:2] = 0.0
            state[3] = 0.0
            print(state)
            print(type(state))

            # add 1 timestep delay to input
            state[0] = state[0] + state[2] * np.cos(state[3]) * P.DT
            state[1] = state[1] + state[2] * np.sin(state[3]) * P.DT
            state[2] = state[2] + action[0] * P.DT
            state[3] = state[3] + action[0] * np.tan(action[1]) / P.L * P.DT

            # optimization loop
            start = time.time()

            # State Matrices
            A, B, C = mpcpy.get_linear_model_matrices(state, action)

            # Get Reference_traj -> inputs are in worldframe
            target, _ = mpcpy.get_ref_trajectory(get_state(car), path, 1.0)
            print(get_state(car))

            x_mpc, u_mpc = mpc.optimize_linearized_model(
                A, B, C, state, target, time_horizon=P.T, verbose=False
            )
            

            # action = np.vstack((np.array(u_mpc.value[0,:]).flatten(),
            #              (np.array(u_mpc.value[1,:]).flatten())))

            action[:] = [u_mpc.value[0, 1], u_mpc.value[1, 1]]

            elapsed = time.time() - start
            print("CVXPY Optimization Time: {:.4f}s".format(elapsed))

            set_ctrl(car, state[2], action[0], action[1])

            ctrl_cmd = [state[2], action[0], action[1]]
            f.write(str(ctrl_cmd)+'\n')

            if P.DT - elapsed > 0:
                time.sleep(P.DT - elapsed)
            
    




if __name__ == "__main__":
    run_sim()
