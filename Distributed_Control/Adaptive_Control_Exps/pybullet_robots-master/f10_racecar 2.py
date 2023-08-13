import pybullet as p
import time
import math

p.connect(p.GUI)

p.resetSimulation()

p.setGravity(0,0,-10)
useRealTimeSim = 0

p.setTimeStep(1./120.)
p.setRealTimeSimulation(useRealTimeSim) # either this

plane = p.loadURDF("plane.urdf")
track = p.loadSDF("f10_racecar/meshes/barca_track.sdf", globalScaling=1)
# otherCar = p.loadURDF("f10_racecar/racecar_differential.urdf", [0,1,.3])
startOrn = p.getQuaternionFromEuler([0,0,1])
car = p.loadURDF("f10_racecar/racecar_differential.urdf", [-0.4,0,.3], startOrn)



	
for wheel in range(p.getNumJoints(car)):
	print("joint[",wheel,"]=", p.getJointInfo(car,wheel))
	p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
	p.getJointInfo(car,wheel)	

wheels = [8,15]
print("----------------")

#p.setJointMotorControl2(car,10,p.VELOCITY_CONTROL,targetVelocity=1,force=10)
c = p.createConstraint(car,9,car,11,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=1, maxForce=10000)

c = p.createConstraint(car,10,car,13,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-1, maxForce=10000)

c = p.createConstraint(car,9,car,13,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-1, maxForce=10000)

c = p.createConstraint(car,16,car,18,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=1, maxForce=10000)


c = p.createConstraint(car,16,car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-1, maxForce=10000)

c = p.createConstraint(car,17,car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-1, maxForce=10000)

c = p.createConstraint(car,1,car,18,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-1, gearAuxLink = 15, maxForce=10000)
c = p.createConstraint(car,3,car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-1, gearAuxLink = 15,maxForce=10000)


steering = [0,2]

hokuyo_joint=4
zed_camera_joint = 5

targetVelocitySlider = p.addUserDebugParameter("wheelVelocity",-50,50,0)
maxForceSlider = p.addUserDebugParameter("maxForce",0,50,20)
steeringSlider = p.addUserDebugParameter("steering",-1,1,0)

replaceLines=True
numRays=100
rayFrom=[]
rayTo=[]
rayIds=[]
rayHitColor = [1,0,0]
rayMissColor = [0,1,0]
rayLen = 8
rayStartLen=0.25
for i in range (numRays):
	#rayFrom.append([0,0,0])
	rayFrom.append([rayStartLen*math.sin(-0.5*0.25*2.*math.pi+0.75*2.*math.pi*float(i)/numRays), rayStartLen*math.cos(-0.5*0.25*2.*math.pi+0.75*2.*math.pi*float(i)/numRays),0])
	rayTo.append([rayLen*math.sin(-0.5*0.25*2.*math.pi+0.75*2.*math.pi*float(i)/numRays), rayLen*math.cos(-0.5*0.25*2.*math.pi+0.75*2.*math.pi*float(i)/numRays),0])
	if (replaceLines):
		rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor,parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint ))
	else:
		rayIds.append(-1)
		


def getCarYaw(car):
	carPos,carOrn = p.getBasePositionAndOrientation(car)
	carEuler = p.getEulerFromQuaternion(carOrn)
	carYaw = carEuler[2]*360/(2.*math.pi)-90
	return carYaw

prevCarYaw = getCarYaw(car)
frame = 0

lineId = p.addUserDebugLine([0,0,0],[0,0,1],[1,0,0])
lineId2 = p.addUserDebugLine([0,0,0],[0,0,1],[1,0,0])
lineId3= p.addUserDebugLine([0,0,0],[0,0,1],[1,0,0])
print("lineId=",lineId)

camInfo = p.getDebugVisualizerCamera()
lastTime = time.time()
lastControlTime = time.time()
lastLidarTime = time.time()

frame=0
count=10
maxDistLoc = [0,0,0]
turn=1

x_history = []
y_history = []

while (True):
	
	nowTime = time.time()
	#render Camera at 10Hertz
	if (nowTime-lastTime>.1):
		ls = p.getLinkState(car,zed_camera_joint, computeForwardKinematics=True)
		camPos = ls[0]
		camOrn = ls[1]
		camMat = p.getMatrixFromQuaternion(camOrn)
		upVector = [0,0,1]
		forwardVec = [camMat[0],camMat[3],camMat[6]]
		#sideVec =  [camMat[1],camMat[4],camMat[7]]
		camUpVec =  [camMat[2],camMat[5],camMat[8]]
		camTarget = [camPos[0]+forwardVec[0]*10,camPos[1]+forwardVec[1]*10,camPos[2]+forwardVec[2]*10]
		camUpTarget = [camPos[0]+camUpVec[0],camPos[1]+camUpVec[1],camPos[2]+camUpVec[2]]
		viewMat = p.computeViewMatrix(camPos, camTarget, camUpVec)
		projMat = camInfo[3]
		#p.getCameraImage(320,200,viewMatrix=viewMat,projectionMatrix=projMat, flags=p.ER_NO_SEGMENTATION_MASK, renderer=p.ER_BULLET_HARDWARE_OPENGL)
		p.getCameraImage(320,200,viewMatrix=viewMat,projectionMatrix=projMat, renderer=p.ER_BULLET_HARDWARE_OPENGL)
		lastTime=nowTime
	
	nowControlTime = time.time()
	
	nowLidarTime = time.time()
	#lidar at 20Hz
	if (nowLidarTime-lastLidarTime>.3):
		#print("Lidar!")
		numThreads=0
		results = p.rayTestBatch(rayFrom,rayTo,numThreads, parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
		# print("Result: ", results)
		for i in range (numRays):
			maxDist = 0
			hitObjectUid=results[i][0]
			# print("hitObjectUid: ", hitObjectUid)
			hitFraction = results[i][2]
			# print("hitFraction: ", hitFraction)
			hitPosition = results[i][3]
			# print("hitPosition: ", hitPosition)
			if (hitFraction==1.):
				p.addUserDebugLine(rayFrom[i],rayTo[i], rayMissColor,replaceItemUniqueId=rayIds[i],parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
				localMissAt = [rayFrom[i][0]+hitFraction*(rayTo[i][0]-rayFrom[i][0]),
											rayFrom[i][1]+hitFraction*(rayTo[i][1]-rayFrom[i][1]),
											rayFrom[i][2]+hitFraction*(rayTo[i][2]-rayFrom[i][2])]
				# print("localMissAt["+str(i)+"]: ", localMissAt)
				carPos,carOrn = p.getBasePositionAndOrientation(car)
				localMissAtDist = math.dist(carPos, localMissAt)
				if localMissAtDist>maxDist:
					maxDist = localMissAtDist
					maxDistLoc = localMissAt
				# print("Max Dist", maxDist)
			else:
				localHitTo = [rayFrom[i][0]+hitFraction*(rayTo[i][0]-rayFrom[i][0]),
											rayFrom[i][1]+hitFraction*(rayTo[i][1]-rayFrom[i][1]),
											rayFrom[i][2]+hitFraction*(rayTo[i][2]-rayFrom[i][2])]
				# print("localHitTo: ", localHitTo)
				p.addUserDebugLine(rayFrom[i],localHitTo, rayHitColor,replaceItemUniqueId=rayIds[i],parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
		lastLidarTime = nowLidarTime
		
	#control at 100Hz
	if (nowControlTime-lastControlTime>.01):
		carPos,carOrn = p.getBasePositionAndOrientation(car)
		print("Current Position: ", carPos)
		new_x = carPos[0]
		new_y = carPos[1]
		x_history.append(new_x)
		y_history.append(new_y)
		# print(new_x)
		# print(new_y)
		# f.write("state[0]: "+str(state[0])+" new_x: "+str(new_x)+'\n')
		# f.write("state[1]: "+str(state[1])+" new_x: "+str(new_x)+'\n')

		# track path in bullet
		p.addUserDebugLine(
			[new_x, new_y, 0], [new_x, new_y, 0.5], [1, 0, 0]
		)

		# Keep the previous orientation of the camera set by the user.
		
		yaw = camInfo[8]
		pitch = camInfo[9]
		distance = camInfo[10]
		targetPos = camInfo[11]
		camFwd = camInfo[5]
		carYaw = getCarYaw(car)

		#the car yaw is clamped between -90 and 270, make sure to deal with angles that wrap around
		if (carYaw-prevCarYaw>45):
			yaw+=360
		if (carYaw-prevCarYaw<-45):
			yaw-=360
		prevCarYaw = carYaw

		#print("carYaw=", carYaw)
		#print("camYaw=", yaw)
		
		#slowly rotate the camera behind the car
		diffYaw = (carYaw-yaw)*0.03

		#track the position of the car as target
		p.resetDebugVisualizerCamera(distance, -(yaw+diffYaw), pitch, carPos)

		maxForce = p.readUserDebugParameter(maxForceSlider)
		targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
		steeringAngle = p.readUserDebugParameter(steeringSlider)
		#print(targetVelocity)

		targetVelocity = 20

		tau_p = 10
		tau_d = 0.01
		tau_i = 0.01

		prev_cte = carPos[1]
		sum_cte = 0
		
		
		sum_cte += carPos[1]
		dev = carPos[1] - prev_cte
		prev_cte = carPos[1]
		steeringAngle = -tau_p * carPos[1] - tau_d * dev - tau_i*sum_cte


		steeringAngle = -math.atan2(maxDistLoc[1]-carPos[1]-1, maxDistLoc[0]-carPos[0]-1)
		steeringAngle = math.radians(steeringAngle)
		if maxDistLoc[1] == carPos[1]:
			steeringAngle=0
		print(maxDistLoc, steeringAngle)

		if turn==1 and carPos[0]>6.5:
			steeringAngle = 0.9
			turn=0
	
		for wheel in wheels:
			p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=maxForce)
			
		for steer in steering:
			p.setJointMotorControl2(car,steer,p.POSITION_CONTROL,targetPosition=-steeringAngle)
		
			
		steering
		if (useRealTimeSim==0):
			frame+=1
			p.stepSimulation()
		lastControlTime=nowControlTime
	# count = count-1
