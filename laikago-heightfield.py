import pybullet as p
import pybullet_data as pd  # Importing pybullet_data for additional resources
import time
import random  # Added for generating random terrain height

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())  # Added to set data search path

# Create terrain (the following block is from your first script)
useProgrammatic = 0
heightfieldSource = useProgrammatic
heightPerturbationRange = 0.05
random.seed(10)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
if heightfieldSource==useProgrammatic:
  numHeightfieldRows = 256
  numHeightfieldColumns = 256
  heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns 
  for j in range (int(numHeightfieldColumns/2)):
    for i in range (int(numHeightfieldRows/2)):
      height = random.uniform(0,heightPerturbationRange)
      heightfieldData[2*i+2*j*numHeightfieldRows]=height
      heightfieldData[2*i+1+2*j*numHeightfieldRows]=height
      heightfieldData[2*i+(2*j+1)*numHeightfieldRows]=height
      heightfieldData[2*i+1+(2*j+1)*numHeightfieldRows]=height
      
  terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.05,.05,1], heightfieldTextureScaling=(numHeightfieldRows-1)/2, heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows, numHeightfieldColumns=numHeightfieldColumns)
  terrain  = p.createMultiBody(0, terrainShape)
  p.resetBasePositionAndOrientation(terrain,[0,0,0], [0,0,0,1])
p.changeVisualShape(terrain, -1, rgbaColor=[1,1,1,1])  # end of terrain creation

p.setGravity(0,0,-9.8)
p.setTimeStep(1./500)

# Loading of quadruped remains unchanged
urdfFlags = p.URDF_USE_SELF_COLLISION
quadruped = p.loadURDF("laikago/laikago_toes.urdf",[0,0,5],[0,0.5,0.5,0], flags = urdfFlags,useFixedBase=False)



#enable collision between lower legs

for j in range (p.getNumJoints(quadruped)):
		print(p.getJointInfo(quadruped,j))

#2,5,8 and 11 are the lower legs
lower_legs = [2,5,8,11]
for l0 in lower_legs:
	for l1 in lower_legs:
		if (l1>l0):
			enableCollision = 1
			print("collision for pair",l0,l1, p.getJointInfo(quadruped,l0)[12],p.getJointInfo(quadruped,l1)[12], "enabled=",enableCollision)
			p.setCollisionFilterPair(quadruped, quadruped, l0,l1,enableCollision)

jointIds=[]
paramIds=[]
jointOffsets=[]
jointDirections=[-1,1,1,1,1,1,-1,1,1,1,1,1]
jointAngles=[0,0,0,0,0,0,0,0,0,0,0,0]

for i in range (4):
	jointOffsets.append(0)
	jointOffsets.append(-0.7)
	jointOffsets.append(0.7)

maxForceId = p.addUserDebugParameter("maxForce",0,100,20)

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

'''with open("data1.txt","r") as filestream:
	for line in filestream:
		maxForce = p.readUserDebugParameter(maxForceId)
		currentline = line.split(",")
		frame = currentline[0]
		t = currentline[1]
		joints=currentline[2:14]
		for j in range (12):
			targetPos = float(joints[j])
			p.setJointMotorControl2(quadruped,jointIds[j],p.POSITION_CONTROL,jointDirections[j]*targetPos+jointOffsets[j], force=maxForce)
		p.stepSimulation()
		for lower_leg in lower_legs:
			#print("points for ", quadruped, " link: ", lower_leg)
			pts = p.getContactPoints(quadruped,-1, lower_leg)
			#print("num points=",len(pts))
			#for pt in pts:
			#	print(pt[9])
		time.sleep(1./500.)'''


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
	
	for i in range(len(paramIds)):
		c = paramIds[i]
		targetPos = p.readUserDebugParameter(c)
		maxForce = p.readUserDebugParameter(maxForceId)
		p.setJointMotorControl2(quadruped,jointIds[i],p.POSITION_CONTROL,jointDirections[i]*targetPos+jointOffsets[i], force=maxForce)
	
