import pybullet as p
import time
import threading
import os
os.environ['PYBULLET_USE_CUDA'] = '1'
class QuadrupedSim:
	def __init__(self, gui=True, tstep = 1./500, maxforce = 20):
		if(gui == True):
			p.connect(p.GUI)
		else:
			p.connect(p.DIRECT)
		self.tstep = tstep
		self.plane = p.loadURDF("plane.urdf")
		p.setGravity(0,0,-9.8)
		p.setTimeStep(tstep)
		urdfFlags = p.URDF_USE_SELF_COLLISION
		self.quadruped = p.loadURDF("/home/kumaran/pybullet_robots/data/laikago/laikago_toes.urdf",[0,0,.5],[0,0.5,0.5,0], flags = urdfFlags,useFixedBase=False)
		self.done = False


		self.link_name_to_index = {}
		self.jointIds=[]
		self.paramIds=[]
		self.jointOffsets=[]
		self.initial_joint_angles = []
		self.initial_joint_velocities = []
		self.jointDirections=[-1,1,1,1,1,1,-1,1,1,1,1,1]
		self.jointAngles=[0,0,0,0,0,0,0,0,0,0,0,0]#i think this is the current state but its not being used
		self.target_angles = [0,0,0,0,0,0,0,0,0,0,0,0]#target state in action space?
		#self.maxForceId = p.addUserDebugParameter("maxForce",0,100,20)
		self.initial_max_force = maxforce#p.readUserDebugParameter(self.maxForceId)


		self.setup_simulation()

		self.timers = []
		self.simulationStep = 0
		self.toenamekeys = ['toeRL', 'toeRR', 'toeFL', 'toeFR']
		self.lock = threading.Lock()


	def start_timer(self, freq, callback):# do not call this in anything other than the openAI GYM constructor or it will cause race conditions
		timer_step = 1./freq
		current_time = self.simulationStep*self.tstep  
		self.timers.append({
		'freq': freq,
		'callback': callback,
		'next_call': current_time+timer_step,
		'timer_step': timer_step
		})

	def check_timers(self):
		current_time = self.simulationStep*self.tstep
		for timer in self.timers:
			if current_time >= timer['next_call']:
				print('permission given', self.simulationStep)
				timer['callback']()
				timer['next_call'] += timer['timer_step']



	def setup_simulation(self):
		

                #print all the joints
		for j in range (p.getNumJoints(self.quadruped)):
			print(p.getJointInfo(self.quadruped,j))

                # Iterate over all joints
		for i in range(p.getNumJoints(self.quadruped)):
			# Get joint info
                	joint_info = p.getJointInfo(self.quadruped, i)
                	# joint_info[12] is the link name
                	link_name = joint_info[12].decode('utf-8')
                	self.link_name_to_index[link_name] = i


		print('link_name_to_index', self.link_name_to_index)

		self.toe_indices = [self.link_name_to_index['toeRL'], self.link_name_to_index['toeRR'], self.link_name_to_index['toeFL'], self.link_name_to_index['toeFR']]

		print(self.toe_indices)

                #enable collision between lower legs
                #2,5,8 and 11 are the lower legs
		lower_legs = [2,5,8,11]
		for l0 in lower_legs:
			for l1 in lower_legs:
				if (l1>l0):
					enableCollision = 1
					print("collision for pair",l0,l1, p.getJointInfo(self.quadruped,l0)[12],p.getJointInfo(self.quadruped,l1)[12], "enabled=",enableCollision)
					p.setCollisionFilterPair(self.quadruped, self.quadruped, l0,l1,enableCollision)


                #some code
		for i in range (4):
			self.jointOffsets.append(0)
			self.jointOffsets.append(-0.7)
			self.jointOffsets.append(0.7)
		for j in range (p.getNumJoints(self.quadruped)):
			p.changeDynamics(self.quadruped,j,linearDamping=0, angularDamping=0)
			info = p.getJointInfo(self.quadruped,j)
			#print(info)


			jointName = info[1]
			jointType = info[2]

			if jointType==p.JOINT_PRISMATIC:
				print('JOINT_PRISMATIC')

			if jointType==p.JOINT_REVOLUTE:
				print('JOINT_REVOLUTE')

			if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
				self.jointIds.append(j)

		
		p.getCameraImage(480,320)
		p.setRealTimeSimulation(0)

		joints=[]

        
		#build the robot in the sim
		index = 0
		for j in range (p.getNumJoints(self.quadruped)):
			p.changeDynamics(self.quadruped,j,linearDamping=0, angularDamping=0)
			info = p.getJointInfo(self.quadruped,j)
			js = p.getJointState(self.quadruped,j)

			self.initial_joint_angles.append(js[0])
			self.initial_joint_velocities.append(js[1])

			
			jointName = info[1]
			jointType = info[2]
			if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
				self.paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),-4,4,(js[0]-self.jointOffsets[index])/self.jointDirections[index]))
				print("INITAL ANGLE: ", js[0], (js[0]-self.jointOffsets[index])/self.jointDirections[index] )# MY UNDERSTANDING OF HOW JOINT ANGLES WORKS ARE WRONG COMPLETELY
				index=index+1


		for i in range(len(self.paramIds)):
			c = self.paramIds[i]
			targetPos = p.readUserDebugParameter(c)


			self.target_angles[i] = targetPos

		print( 'self.target_angles', self.target_angles)
		#p.setRealTimeSimulation(1)

	def get_object_state(self):

		with self.lock:
			ret = []




			pos, quat = p.getBasePositionAndOrientation(self.quadruped)
			roll, pitch, yaw = p.getEulerFromQuaternion(quat)
			#print( 'roll', roll, 'pitch', pitch)
			com_x, com_y, com_z = pos
			#print("Center of Mass position (x, y, z): ", com_x, com_y, com_z)

			angs = []
			vels = []

			for i in range(len(self.paramIds)):
				joint_state = p.getJointState(self.quadruped, self.jointIds[i])
				current_angle = joint_state[0]
				angular_velocity = joint_state[1]

				angs.append(current_angle)
				vels.append(angular_velocity)


			ret = angs + vels + [roll] + [pitch]



			ctr = 0
			for toe_index in self.toe_indices:
				contact_points = p.getContactPoints(self.quadruped, -1, toe_index)
				#if len(contact_points) > 0:
				#	print(f"Contact detected for toe with index {toe_index}")

				ret.append ( 1*(len(contact_points) > 0))
				ctr+=0

			ret = ret + [com_z, com_y, com_x] 

			#print('OBSERVATION 2:', com_z, com_y, com_x)
			return ret



	def set_actions(self, actions):
		with self.lock:

			#check if angles are withing valid range


			for i in range(len(self.paramIds)):

				self.target_angles[i] = actions[i]
				targetPos = actions[i]
				maxForce = self.initial_max_force
				p.setJointMotorControl2(self.quadruped, self.jointIds[i],p.POSITION_CONTROL,self.jointDirections[i]*targetPos+self.jointOffsets[i], force=maxForce)

				joint_state = p.getJointState(self.quadruped, self.jointIds[i])
		
		

				current_angle = joint_state[0]
			return True # should return something else for invalid action


	def stepSim(self):
		with self.lock:
			for i in range(len(self.paramIds)):
				c = self.paramIds[i]
				#targetPos = p.readUserDebugParameter(c)
				maxForce = self.initial_max_force
				p.setJointMotorControl2(self.quadruped, self.jointIds[i],p.POSITION_CONTROL,self.jointDirections[i]*self.target_angles[i]+self.jointOffsets[i], force=maxForce)

				joint_state = p.getJointState(self.quadruped, self.jointIds[i])
		
		

				current_angle = joint_state[0]
				#print('targetPos: ', targetPos)
				#print('joint_state len: ', len(joint_state))
				#print(f"Joint {i}: Current angle = {current_angle}")

			p.stepSimulation()
			self.simulationStep+=1



	def reset_sim(self):
		with self.lock:
			# Reset the base position and orientation
			p.resetBasePositionAndOrientation(self.quadruped, [0,0,.5], [0,0.5,0.5,0])

			# Reset the joint states to the initial values
			for i in range(p.getNumJoints(len(self.paramIds) )):

				p.resetJointState(self.quadruped, i, targetValue=self.jointDirections[i]*self.initial_joint_angles[i]+self.jointOffsets[i], targetVelocity=self.initial_joint_velocities[i])


			# Reset the simulation time
			self.simulationStep = 0

			# Reset the timers
			for timer in self.timers:
				timer['next_call'] = 0

			# Reset the target angles
			self.target_angles = self.initial_joint_angles.copy()


			# Reset the maxForceId to its default value
			#p.resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0,0,0])
			#self.maxForceId = p.addUserDebugParameter("maxForce",0,100,20)



	def run_simulation(self):
		while (not self.done):
			self.stepSim()
			#self.get_object_state()
			self.check_timers()


			#time.sleep(self.tstep)


#qsim = QuadrupedSim()
#qsim.run_simulation()
#a = 0 
#while(1):
#	a +=1
#	a-=1
#	time.sleep(1) 





