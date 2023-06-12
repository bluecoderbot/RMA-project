import gym
from gym.spaces import Box
import numpy as np
import threading
import copy
from laikago_class import QuadrupedSim
import os
#os.environ['PYBULLET_USE_CUDA'] = '1'

# Suppose each joint has an angle between -pi and pi and a velocity between -10 and 10
joint_low_bounds = [-np.inf] * 12#CONVERT TO -PI
joint_high_bounds = [np.inf] * 12#CONVERT TO PI

joint_low_velocity = [-np.inf] * 12#CONVERT TO -PI
joint_high_velocity = [np.inf] * 12#CONVERT TO PI

roll_low = [-np.pi]  # -180 degrees in radians
roll_high = [np.pi]  # 180 degrees in radians

pitch_low = [-np.pi]  # -180 degrees in radians
pitch_high = [np.pi]  # 180 degrees in radians



contact_indicators_low = [0]*4 #binarized foot contact indicators
contact_indicators_high = [1]*4 #binarized foot contact indicators

MAX_DISTANCE = 1000

minCartesian = [-MAX_DISTANCE, -MAX_DISTANCE, -MAX_DISTANCE]
maxCartesian = [MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE]

TARGET_LIMITS = 50
TARGET_THRESHOLD = 2

minTarget = [-TARGET_LIMITS, -TARGET_LIMITS]
maxTarget = [TARGET_LIMITS, TARGET_LIMITS]


low_bounds=joint_low_bounds+joint_low_velocity+roll_low+pitch_low+contact_indicators_low+minCartesian+minTarget
high_bounds=joint_high_bounds+joint_high_velocity+roll_high+pitch_high+contact_indicators_high+maxCartesian+maxTarget

low_bounds = np.array(low_bounds)
high_bounds = np.array(high_bounds)

observation_space = Box(low=low_bounds, high=high_bounds)

# Suppose each joint can be commanded to go to an angle between -pi and pi
action_low_bounds = np.array([-4.0] * 12)
action_high_bounds = np.array([4.0] * 12)
action_space = Box(low=action_low_bounds, high=action_high_bounds)
toenamekeys = ['toeRL', 'toeRR', 'toeFL', 'toeFR']
class QuadrupedEnv(gym.Env):


	def __init__(self, agentFrequency=100, maxseconds = 1000):
		super().__init__()
		self.observation_space = observation_space
		self.action_space = action_space


		self.maxsteps = agentFrequency*maxseconds

		self.Y_TARGET = 0
		self.X_TARGET = 0
		self.num_steps = 0

		self.create_target()


		self.sim_init_lock = threading.Lock()
		self.sim_init_lock.acquire()  # Lock will be released once sim is initialized

		self.stepFlag = False
		self.condition = threading.Condition()

		# Initialize and start the simulation in a separate thread
		self.sim_thread = threading.Thread(target=self.init_sim, args=(agentFrequency,))
		self.sim_thread.start()

		#self.sim.run_simulation()
	def init_sim(self, agentFrequency):
		# Initialize the simulation
		self.sim = QuadrupedSim()

		# Start the timer
		self.sim.start_timer(agentFrequency, self.on_step)

		#self.sim.stepSim()

		self.sim_init_lock.release()  # Release the lock now that sim is initialized

		# Run the simulation
		self.sim.run_simulation()

	def create_target(self):

		self.Y_TARGET = np.random.rand()*TARGET_LIMITS*2 - TARGET_LIMITS
		self.X_TARGET = np.random.rand()*TARGET_LIMITS*2 - TARGET_LIMITS

		print('TARGET CREATED',self.Y_TARGET  , self.X_TARGET)

	def on_step(self):
		with self.condition:
			self.stepFlag = True
			self.condition.notify()
	def reward(self, posy, posx):
		reward = (posy - self.Y_TARGET)**2 + (posx - self.X_TARGET)**2
		reward = reward*(-1)
		return reward

	def step(self, action):

		print('step: ', self.num_steps)

		self.sim_init_lock.acquire()  # This will block until the lock is released in init_sim
		self.sim_init_lock.release()  # Immediately release the lock so it doesn't block next time
		#print('have released the lock')

		# call self.sim.set_actions()
		# Use action to set joint angles

		self.sim.set_actions(action)



		# Compute the observation
		#keep track of min and max motor angles and velocities
		with self.condition:
			while not self.stepFlag:
				self.condition.wait()
			obs = self.sim.get_object_state()
			self.stepFlag = False
#low_bounds=joint_low_bounds+joint_low_velocity+roll_low+pitch_low+contact_indicators_low+minCartesian+minTarget



		#reward is negative distance squared
		reward = self.reward(obs[-2], obs[-1])

		observation = obs + [copy.copy(self.Y_TARGET)] + [copy.copy(self.X_TARGET)]
		#print('OBSERVATION: ', obs)


		self.num_steps+=1

		done = False
		if -1*reward < TARGET_THRESHOLD**2 or self.num_steps>=self.maxsteps:
			done = True
			self.sim.done = True

		#implement timed cutoff

		info = {}



#you could implement a method that 

		# Compute the reward
		# Check if the episode is done
		return observation, reward, done, info


	def reset(self):

		self.num_steps=0

		self.stepFlag = False

		self.create_target()

		# Reset the simulation
		self.sim_init_lock.acquire()

		print("GOT THIS FAR")

		self.sim.reset_sim()
		self.sim.done = False
		#self.sim.run_simulation()# THE ORDERING PLACEMENT OF THIS COULD BE COMPLETELY WRONG
		self.sim_init_lock.release()


		#IMPLEMENT LOCKING OR SEMAPHOR OR WHATEVER I THINK IDK THOUGH SHOULD I??



		# Compute and return the initial observation
		initial_observation = self.sim.get_object_state() + [copy.copy(self.Y_TARGET)] + [copy.copy(self.X_TARGET)]
		# return observation
		return initial_observation




#qe = QuadrupedEnv()


