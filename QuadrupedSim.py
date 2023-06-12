import pybullet as p
import time
import numpy as np
import os
import builtins
os.environ['PYBULLET_USE_CUDA'] = '1'
class QuadrupedSim:
	def __init__(self, gui=True, tstep = 1./400, maxforce = 20):
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

		#save initial state
		self.initial_joint_angles = []
		self.initial_joint_velocities = []


		self.jointDirections=[-1,1,1,1,1,1,-1,1,1,1,1,1]
		#self.jointAngles=[0,0,0,0,0,0,0,0,0,0,0,0]#i think this is the current state but its not being used
		self.target_angles = []#[0,0,0,0,0,0,0,0,0,0,0,0]#target state in action space?

		self.initial_max_force = [maxforce]*12#p.readUserDebugParameter(self.maxForceId)


		self.setup_simulation()

		self.simulationStep = 0
		self.toenamekeys = ['toeRL', 'toeRR', 'toeFL', 'toeFR']
		#self.lock = threading.Lock()


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

			self.initial_joint_velocities.append(js[1])

			
			jointName = info[1]
			jointType = info[2]
			if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
				angle_offset = (js[0]-self.jointOffsets[index])/self.jointDirections[index]
				self.initial_joint_angles.append(js[0])
				self.paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),-4,4, angle_offset) )
				print("INITAL ANGLE: ", js[0], angle_offset )# MY UNDERSTANDING OF HOW JOINT ANGLES WORKS ARE WRONG COMPLETELY
				self.target_angles.append(js[0] )
				index=index+1


		#for i in range(len(self.paramIds)):
		#	c = self.paramIds[i]
		#	targetPos = p.readUserDebugParameter(c)


		#	self.target_angles[i] = targetPos

		print( 'self.target_angles', self.target_angles)
		#p.setRealTimeSimulation(1)

		print('Mass: ' , p.getDynamicsInfo(self.quadruped, -1)[0] )

		print('Friction of the ground: ' , p.getDynamicsInfo(self.plane, -1)[1] )

		self.initial_state = p.saveState()




	def get_object_state(self):# joints state should match action

		ret = []


		##############################################################################################################
		#RELATED TO WHOLE BODY

		pos, quat = p.getBasePositionAndOrientation(self.quadruped)
		pitch, roll, yaw = p.getEulerFromQuaternion(quat)


		roll = min(np.pi, max(roll, -np.pi) )
		pitch = min(np.pi, max(pitch, -np.pi) )
		yaw = min(np.pi, max(yaw, -np.pi) )

		com_y, com_x, com_z = pos
		base_velocity = p.getBaseVelocity(self.quadruped)
		vy, vx, vz = base_velocity[0]  # linear velocity in X, Y, and Z axes

		wp, wr, wy = angular_base_velocity = base_velocity[1]  # angular velocity in X, Y, and Z axes


		if self.simulationStep%100 == True:
			print("com_x: ", com_x )
			print("\n\n")

		#RELATED TO WHOLE BODY
		##############################################################################################################


		##############################################################################################################
		#RELATED TO JOINTS

		angs = []
		vels = []
		torques = []
		for i in range(len(self.paramIds)):
			joint_state = p.getJointState(self.quadruped, self.jointIds[i])
			current_angle = joint_state[0]
			angular_velocity = joint_state[1]
			joint_torque = joint_state[3]



			angle_offset = (current_angle-self.jointOffsets[i])/self.jointDirections[i]
			velocity_directioned = angular_velocity/self.jointDirections[i]


			angle_offset = min(4, max(angle_offset, -4) ) #constrain the angle range
			velocity_directioned = min(8, max(velocity_directioned, -8) ) #constrain the angle range



			angs.append(angle_offset)
			vels.append(velocity_directioned)
			torques.append(joint_torque)

		#if self.simulationStep%100 == True:
			#print("joint torque: ", torques )
			#print("\n\n\n")

		#RELATED TO JOINTS
		##############################################################################################################



		##############################################################################################################
		#RELATED TO TOE

		toe_velocities = []
		toe_contact = []
		toe_ground_reaction_force = []
		toe_orientations = []
		toe_heights = []

		for toe_index in self.toe_indices:
			contact_points = p.getContactPoints(self.quadruped, -1, toe_index)
			#if len(contact_points) > 0:
			#	print(f"Contact detected for toe with index {toe_index}")

			ground_reaction_force = np.array([0,0,0])


			for contact_point in contact_points:
				normalForce = contact_point[9]  # 9th index in the tuple represents the contact force
				normal_direction = contact_point[7]  # Normal direction from ground to the toe

				normal_force_vector = normalForce * np.array(normal_direction)

				lateralFriction1 = contact_point[10]
				lateralFrictionDir1 = contact_point[11]

				lateralFriction1_vector = lateralFriction1*np.array(lateralFrictionDir1)

				lateralFriction2 = contact_point[12]
				lateralFrictionDir2 = contact_point[13]

				lateralFriction2_vector = lateralFriction2*np.array(lateralFrictionDir2)

				contact_force_vector = normal_force_vector + lateralFriction1_vector + lateralFriction2_vector

				ground_reaction_force = ground_reaction_force + contact_force_vector

	

			toe_ground_reaction_force.append(ground_reaction_force)


			toe_contact.append ( 1*(len(contact_points) > 0))


			link_state = p.getLinkState(self.quadruped, toe_index, computeLinkVelocity=1, computeForwardKinematics=1)


			# Get toe velocity
			toe_velocity = link_state[6]  # link_state[6] contains the world frame linear velocity
			toe_velocities.append(toe_velocity)

			# Get toe orientation: im not using this
			toe_orientation = p.getEulerFromQuaternion(link_state[1])  # link_state[1] contains the toe orientation
			toe_orientations.append(toe_orientation)

			# Get toe position
			ground_z = 0
			toe_pos = link_state[0]
			toe_z = toe_pos[2]
			toe_height = toe_z - ground_z
			toe_heights.append(toe_height)


		toe_velocities = np.array(toe_velocities)
		toe_ground_reaction_force = np.array(toe_ground_reaction_force)
		#RELATED TO TOE
		##############################################################################################################






		###  ####  #  ##  #  #####
 		###  ###   #  # # #    #
		#    #  #  #  #  ##    #
		
		##############################################################################################################
		#PRINT STATE FOR MODEL
		stateVariables = angs + vels + [roll] + [pitch] + toe_contact

		#if self.simulationStep%100 == True:
			#print( 'Joint angles (vector length =', len(angs), '): ', angs)
			#print( 'Joint velocities (vector length =', len(angs), '): ', vels)
			#print( 'roll (vector length =', 1, '): ', roll)
			#print( 'pitch (vector length =', 1, '): ', pitch)
			#print( 'toe_contact (vector length =', len(toe_contact), '): ', toe_contact)
			#print( 'full array stateVariables (vector length =', len(stateVariables), '): ', stateVariables)
			#print("\n")

		#PRINT STATE FOR MODEL
		##############################################################################################################



		##############################################################################################################
		#PRINT STATE FOR REWARD

		rewardvariables = [vx, vy, wy, torques, toe_ground_reaction_force, vz, toe_velocities]

		#if self.simulationStep%100 == True:
			#print( 'X velocity (vector length =', 1, '): ', vx)
			#print( 'Y velocity (vector length =', 1, '): ', vy)
			#print( 'Angular velocity Yaw (vector length =', 1, '): ', wy)
			#print( 'joint torque (vector length =', len(torques), '): ', torques)
			#print( 'ground reaction force (vector length =', len(toe_ground_reaction_force), '): ', toe_ground_reaction_force)
			#print( 'Z velocity (vector length =', 1, '): ', vz)

			#print( 'foot velocity (vector length =', len(toe_velocities), '): ', toe_velocities)
			#print("\n")

		#PRINT STATE FOR REWARD
		##############################################################################################################



		##############################################################################################################
		#PRINT STATE FOR ENCODER

		ground_friction = p.getDynamicsInfo(self.plane, -1)[1]
		maxforcemotors = self.initial_max_force.copy()
		body_mass = p.getDynamicsInfo(self.quadruped, -1)[0]

		encodervariables = [body_mass, maxforcemotors, ground_friction, np.max(toe_heights) ]

		#if self.simulationStep%100 == True:
			#print( 'toe_orientations (vector length =', len(toe_orientations), '): ', toe_orientations)
			#print( 'toe_heights (vector length =', len(toe_heights), '): ', toe_heights)

			#print( 'encodervariables (vector length =', len(encodervariables), '): ', encodervariables)
			#print("\n")
		#PRINT STATE FOR ENCODER
		##############################################################################################################




		##############################################################################################################
		#PRINT STATE FOR RESET

		resetvariables = [com_z]

		#if self.simulationStep%100 == True:
			#print( 'com_z (vector length =', 1, '): ', com_z)
			#print("\n")
		#PRINT STATE FOR RESET
		##############################################################################################################





#low_bounds=joint_low_bounds+joint_low_velocity+roll_low+pitch_low+contact_indicators_low



		ret.append(stateVariables)
		ret.append(rewardvariables)
		ret.append(encodervariables)
		ret.append(resetvariables)

		return ret



	def set_actions(self, actions):
		#check if angles are withing valid range


		for i in range(len(self.paramIds)):

			self.target_angles[i] = actions[i]
			targetPos = actions[i]
			maxForce = self.initial_max_force[i]
			p.setJointMotorControl2(self.quadruped, self.jointIds[i],p.POSITION_CONTROL, targetPos, force=maxForce)

			joint_state = p.getJointState(self.quadruped, self.jointIds[i])
		


			current_angle = joint_state[0]
		return True # should return something else for invalid action


	def stepSim(self):

		for i in range(len(self.paramIds)):
			c = self.paramIds[i]
			#targetPos = p.readUserDebugParameter(c)
			maxForce = self.initial_max_force[i]
			p.setJointMotorControl2(self.quadruped, self.jointIds[i],p.POSITION_CONTROL, self.target_angles[i] , force=maxForce)
			#p.setJointMotorControl2(self.quadruped, self.jointIds[i],p.POSITION_CONTROL,self.jointDirections[i]*targetPos+self.jointOffsets[i], force=maxForce)
			joint_state = p.getJointState(self.quadruped, self.jointIds[i])
		
		

			current_angle = joint_state[0]
			#print('targetPos: ', targetPos)
			#print('joint_state len: ', len(joint_state))
			#print(f"Joint {i}: Current angle = {current_angle}")

		p.stepSimulation()
		self.simulationStep+=1
		#print(self.simulationStep)

		#time.sleep(self.tstep)


	def reset_sim(self, body_mass = -1, ground_friction = -1, max_forces = [-1]*12):



		if max_forces != [-1]*12:
			self.initial_max_force = max_forces

		if body_mass == -1:
			body_mass = p.getDynamicsInfo(self.quadruped, -1)[0]

		if ground_friction == -1:
			ground_friction = p.getDynamicsInfo(self.plane, -1)[1]

		p.restoreState(self.initial_state)

		p.changeDynamics(self.quadruped, -1, mass=body_mass)
		p.changeDynamics(self.plane, -1, lateralFriction=ground_friction)

		self.simulationStep = 0

		# Reset the target angles
		self.target_angles = self.initial_joint_angles.copy()

		#print('self.initial_joint_angles: ', self.initial_joint_angles ) 







#qsim = QuadrupedSim()


#start = time.time()
 
#for it in range(10000):
#	qsim.stepSim()
#end = time.time()


#print(end - start)




