import pybullet as p
import time

p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")
p.setGravity(0,0,-9.8)
p.setTimeStep(1./500)
#p.setDefaultContactERP(0)
#urdfFlags = p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS 
urdfFlags = p.URDF_USE_SELF_COLLISION
quadruped = p.loadURDF("/home/kumaran/pybullet_robots/data/laikago/laikago_toes.urdf",[0,0,.5],[0,0.5,0.5,0], flags = urdfFlags,useFixedBase=False)


link_name_to_index = {}


jointIds=[]
paramIds=[]
jointOffsets=[]
jointDirections=[-1,1,1,1,1,1,-1,1,1,1,1,1]
jointAngles=[0,0,0,0,0,0,0,0,0,0,0,0]

maxForceId = p.addUserDebugParameter("maxForce",0,100,20)







#print all the joints
for j in range (p.getNumJoints(quadruped)):
		print(p.getJointInfo(quadruped,j))


# Iterate over all joints
for i in range(p.getNumJoints(quadruped)):
    # Get joint info
    joint_info = p.getJointInfo(quadruped, i)
    # joint_info[12] is the link name
    link_name = joint_info[12].decode('utf-8')
    link_name_to_index[link_name] = i


print(link_name_to_index)

toe_indices = [link_name_to_index['toeRL'], link_name_to_index['toeRR'], link_name_to_index['toeFL'], link_name_to_index['toeFR']]

print(toe_indices)

#enable collision between lower legs
#2,5,8 and 11 are the lower legs
lower_legs = [2,5,8,11]
for l0 in lower_legs:
	for l1 in lower_legs:
		if (l1>l0):
			enableCollision = 1
			print("collision for pair",l0,l1, p.getJointInfo(quadruped,l0)[12],p.getJointInfo(quadruped,l1)[12], "enabled=",enableCollision)
			p.setCollisionFilterPair(quadruped, quadruped, l0,l1,enableCollision)


#some code
for i in range (4):
	jointOffsets.append(0)
	jointOffsets.append(-0.7)
	jointOffsets.append(0.7)
for j in range (p.getNumJoints(quadruped)):
        p.changeDynamics(quadruped,j,linearDamping=0, angularDamping=0)
        info = p.getJointInfo(quadruped,j)
        #print(info)
        jointName = info[1]
        jointType = info[2]
        if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
                jointIds.append(j)

		
p.getCameraImage(480,320)
p.setRealTimeSimulation(0)

joints=[]


#build the robot in the sim
index = 0
for j in range (p.getNumJoints(quadruped)):
        p.changeDynamics(quadruped,j,linearDamping=0, angularDamping=0)
        info = p.getJointInfo(quadruped,j)
        js = p.getJointState(quadruped,j)
        #print(info)
        jointName = info[1]
        jointType = info[2]
        if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
                paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),-4,4,(js[0]-jointOffsets[index])/jointDirections[index]))
                index=index+1


p.setRealTimeSimulation(1)



while (1):
	for toe_index in toe_indices:
	    contact_points = p.getContactPoints(quadruped, -1, toe_index)
	    if len(contact_points) > 0:
	        print(f"Contact detected for toe with index {toe_index}")


	pos, quat = p.getBasePositionAndOrientation(quadruped)
	roll, pitch, yaw = p.getEulerFromQuaternion(quat)
	print( 'roll', roll, 'pitch', pitch)
	com_x, com_y, com_z = pos
	print("Center of Mass position (x, y, z): ", com_x, com_y, com_z)


	
	for i in range(len(paramIds)):
		c = paramIds[i]
		targetPos = p.readUserDebugParameter(c)
		maxForce = p.readUserDebugParameter(maxForceId)
		p.setJointMotorControl2(quadruped,jointIds[i],p.POSITION_CONTROL,jointDirections[i]*targetPos+jointOffsets[i], force=maxForce)

		joint_state = p.getJointState(quadruped, jointIds[i])
		
		

		current_angle = joint_state[0]
		#print(f"Joint {i}: Current angle = {current_angle}")
	
