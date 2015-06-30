from numpy import *
from Proprioceptives import *
from Sensors import *
from Paths import *

class ROBOT(object):
	"""Class holding current data of a robot. Has to contain it's position
		and covariance along with all the sensors on it. Both proprioceptive and 
		exteroceptive. It should also contain methods to loop over all it's sensors"""

	def __init__(self,initialPosition,initialCovariance):
		self.position = initialPosition			#Holds the current position of the robot 
		self.covariance = initialCovariance		#Holds current error in position
		self.mapIndex = 0						#Holds index in map
		self.stateIndex = 0						#Holds index in state vector	
		self.dimension = 3						#Holds the number of dimensions required to completely define it's position	
		self.pSensors=[]						#Holds objects of proprioceptive sensors
		self.eSensors=[]						#Holds objects of exteroceptive sensors
		self.Path=PATH_CLASS(self.dimension)	#Holds path object
		self.Path.add_to_path(initialPosition)	#Add initial position to path

	def predict_self(self):
		""" The update step for it's position and error using proprioceptive sensors
			Return new position and error to be used for future steps """ 

		for Obj in self.pSensors:
			newPosition,newCovariance = Obj.update(self.position,self.covariance)

		self.position = newPosition
		self.covariance = newCovariance

		return newPosition, newCovariance

	def observe(self):
		"""Observes through each of it's exteroceptive sensors
			Returns a list of features observed """
		featureList=[]
		
		for Obj in self.eSensors:
			featuresObserved = Obj.observe(self.position)
			featureList = featureList +featuresObserved
		return featureList

	def update_self(self,newPosition,newCovariance):
		self.position = newPosition
		self.covariance = newCovariance
		self.Path.add_to_path(newPosition)
