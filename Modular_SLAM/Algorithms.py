from Robots import *
from Landmarks import *
import Log_Manager
from numpy import *
from scipy.linalg import block_diag,inv

import time

def print_timing(func):
    def wrapper(*arg):
        t1 = time.time()
        res = func(*arg)
        t2 = time.time()
        print '%s took %0.3f ms' % (func.func_name, (t2-t1)*1000.0)
        return res
    return wrapper

class EXTENDED_KALMAN_FILTER(object):
	"""Class performing the Extended Kalman Filter based Slam"""

	def __init__(self,VisualizerObject):
		self.state = array([])
		self.covariance = array([])
		self.map=[]
		self.numberOfRobots = 0
		self.numberOfLandmarks = 0
		self.Visualizer = VisualizerObject

	def add_to_map(self,obj):
		"""Add any given object to the map. Modify the index members of the object"""
		obj.mapIndex = len(self.map)
		obj.stateIndex = self.state.shape[0]
		self.map.append(obj)
		self.state = append(self.state,obj.position)
		if not self.covariance.size:
			self.covariance = obj.covariance
		else:
			self.covariance = block_diag(self.covariance,obj.covariance)

		if obj.__class__.__name__ == "ROBOT":
			self.numberOfRobots = self.numberOfRobots + 1
		else:
			self.numberOfLandmarks = self.numberOfLandmarks + 1
	@print_timing
	def run(self):
		""" Main loop of the Kalman Filter"""
		mapRobots=[Obj for Obj in self.map if Obj.__class__.__name__ == "ROBOT"]
		for robot in mapRobots:
#PREDICTION STEP:
			newPosition, newCovariance = robot.predict_self()
			self.state[robot.stateIndex:robot.stateIndex+robot.dimension] = newPosition
			self.covariance[robot.stateIndex:robot.stateIndex+robot.dimension, robot.stateIndex:robot.stateIndex+robot.dimension] = newCovariance
#OBSERVATION STEP:
			observedLandmarks =robot.observe()
			innovation = array([])
			jacobian = array([])
			observerError=array([])
			toBeAdded = []
			for landmarkObj in observedLandmarks:
#DATA ASSOCIATION				
				existingLandmark = landmarkObj.check_if_in_map(self.map)
				self.Visualizer.plot_association(landmarkObj, existingLandmark)
				if Log_Manager.Data.timeStamps.index(Log_Manager.currentTime) > 175:
					print "data association wait:"
				if existingLandmark:
#Set up for CORRECTION
					landmarkJacobian = zeros((landmarkObj.dimension,len(self.state)))
					landmarkJacobian[:,robot.stateIndex:robot.stateIndex+robot.dimension] = existingLandmark.position_jacobian(robot.position)
					landmarkJacobian[:,existingLandmark.stateIndex:existingLandmark.stateIndex+existingLandmark.dimension] = existingLandmark.self_jacobian(robot.position)
					if not jacobian.size:
						jacobian=landmarkJacobian
					else:
						jacobian = vstack((jacobian,landmarkJacobian))

					expectedMeasurement = existingLandmark.in_robot_frame(robot.position)
					actualMeasurement   = landmarkObj.in_robot_frame(robot.position)

					subInnovation = actualMeasurement - expectedMeasurement
					subInnovation[1] = (subInnovation[1] + pi) % (2*pi) - pi
					# print "Alpha innovation: ", subInnovation[1]
					innovation = append(innovation,subInnovation)

					if not observerError.size:
						observerError=landmarkObj.covariance
					else:
						observerError=block_diag(observerError,landmarkObj.covariance)
				elif existingLandmark is None:
					"Landmark too close"
					pass
				elif existingLandmark == False:
					toBeAdded.append(landmarkObj)
		# set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
		# print "Before"
		# print "State:", self.state
		# print "covariance", self.covariance
#CORRECTION STEP
		if jacobian.size:
			kalmanGain = inv(dot(dot(jacobian,self.covariance),jacobian.T)+observerError)
			kalmanGain = dot(dot(self.covariance,jacobian.T),kalmanGain)

			# self.state[3:] = self.state[3:] + dot(kalmanGain,innovation)[3:]
			# self.covariance[3:,3:] = dot((eye(len(self.state)) - dot(kalmanGain,jacobian))[3:,3:],self.covariance[3:,3:])
			correction =dot(kalmanGain,innovation)
			# print "Theta correction:", correction[2]
			self.state = self.state + correction
			self.covariance = dot((eye(len(self.state)) - dot(kalmanGain,jacobian)),self.covariance)

		# print "After"
		# print "State:", self.state
		# print "covariance", self.covariance
		# print "Eigen Values: ",linalg.eigvals(self.covariance)
#UPDATING THE MAP
		for components in self.map:
			components.update_self(self.state[components.stateIndex:components.stateIndex+components.dimension], self.covariance[components.stateIndex:components.stateIndex+components.dimension,components.stateIndex:components.stateIndex+components.dimension])
		
		for newLandmarks in toBeAdded:
			self.add_to_map(newLandmarks)
		return