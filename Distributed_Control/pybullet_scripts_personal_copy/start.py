#Test script for F10 racecar+env - NOT USED

from turtle import distance
import numpy as np
import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)

# Load Assets
p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1])
targid = p.loadURDF("racecar/racecar.urdf", [0,0,0], [0,0,0,1], useFixedBase=False)
obj_of_focus = targid

for step in range(300):
    focus_position, _ = p.getBasePositionAndOrientation(targid)
    p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=focus_position)
    p.stepSimulation()
    # time.sleep(0.01)

# Get Joint Info
for i in range(p.getNumJoints(targid)):
    print(p.getJointInfo(targid, i))

# Get min-max range of joint
joint_id = 5
joint_type = p.getJointInfo(targid, joint_id)[2]
joint_lower = p.getJointInfo(targid, joint_id)[8]
joint_upper = p.getJointInfo(targid, joint_id)[9]
print(joint_type, joint_lower, joint_upper)

dist = 100000

while True:

    pos, orn = p.getBasePositionAndOrientation(targid)
    yaw = p.getEulerFromQuaternion(orn)[-1]

    xA, yA, zA = pos
    zA = zA + 0.3

    xB = xA + math.cos(yaw)*dist
    yB = yA + math.sin(yaw)*dist
    zB = zA

    viewMatrix = p.computeViewMatrix(cameraEyePosition=[xA, yA, zA],cameraTargetPosition=[xB, yB, zB],cameraUpVector=[0,0,1.0])

    projectionMatrix = p.computeProjectionMatrixFOV(fov=90.0,aspect=1.5,nearVal=0.02,farVal=3.5)

    width, height, rgbImage, depthImage, segImage = p.getCameraImage(width=120, height=80, viewMatrix=viewMatrix, projectionMatrix=projectionMatrix)

    # objId, link, hitFrac, hitPos, hitNorm = p.rayTest(rayFromPosition=[xA,yA,zA],rayToPosition=[xB,yB,zB])

    # Move joints
    joint_five_target = 0
    joint_seven_target = 0
    joint_two_target = 0                    # left rear wheel
    joint_three_target = 0

    for step in range(500):
        joint_five_target = joint_five_target+0.1 #np.random.uniform(joint_lower, joint_upper)
        joint_seven_target = joint_seven_target+0.1 #np.random.uniform(joint_lower, joint_upper)
        joint_two_target = joint_two_target+1 #np.random.uniform(joint_lower, joint_upper)
        joint_three_target = joint_three_target+1 #np.random.uniform(joint_lower, joint_upper)
        p.setJointMotorControlArray(targid, [5,7], p.POSITION_CONTROL, targetPositions = [joint_five_target, joint_seven_target])
        p.stepSimulation()
        time.sleep(0.01)