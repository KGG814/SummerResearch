from numpy import *
from math import degrees, sin, cos, hypot, atan2, pi
from scipy.optimize import curve_fit


class DERIVATIVES(object):
	"""Class to find cylinders using first derivative combined with pre knowledge of the cylinders"""

	def __init__(self,derivativeThreshold, cylinderRadius, shortRangeAccuracy, longRangeAccuracy, unobservableLandmarkArea, lineFittingThreshold, minSamples):
		self.derivativeThreshold = derivativeThreshold
		self.cylinderRadius = cylinderRadius					 #Radius of cylinders (unit = meter)
		self.shortRangeAccuracy = shortRangeAccuracy			 #Accuracy of lidar sensor  for distance less than 1 meter (Unit = meter)
		self.longRangeAccuracy = longRangeAccuracy				 #Accuracy of lidar sensor  for distance larger than 1 meter (Unit = percentage)
		self.unobservableLandmarkArea = unobservableLandmarkArea #The percentage of the landmark's area that we suppose the lidar misses
		self.lineFittingThreshold = lineFittingThreshold		 #Threshold for line fitting error
		self.minSamples = minSamples 							 #Threshold for minimum number of samples
		self.landmarkType = "Cylinder"				

	@staticmethod
	def _line(x,m,c):
		"""Equation of a line used as a cost function for least square optimization"""
		return m*x+c

	@staticmethod
	def _pole_to_cart(Set):
		"""Polar to Cartesian conversion of a whole list"""
		xS=[]
		yS=[]
		for s in Set:
			angle = s[0]
			distance = s[1] 
			if distance !=0:
				xs, ys = distance*cos(angle), distance*sin(angle)
				xS.append(xs)
				yS.append(ys)
		return xS,yS

	def get_landmarks(self,angles,distances):
		#Initialize empty list of landmarks
		landmarks = []

		#Initially no thresholds are found
		previousThresholdFound = False

		startingIndex = 1
		#Look at each pair of angles and distances
		for i in xrange(1,len(angles)):

			#Check if there is a large jump in distance
			if abs(distances[i]-distances[i-1])>self.derivativeThreshold :

				#Store starting index
				stoppingIndex = i-1

				#Check if we are already on a probable landMark 
				if previousThresholdFound:
					# Then this is at the end of a probable landmark
					
					#Find the parameters of the probable landmark. [startingIndex from previous threshold]
					beamIndex = int((startingIndex+stoppingIndex)/2)
					beamDistance = distances[beamIndex]+self.cylinderRadius
					beamAngle = angles[beamIndex]

					#Convert parameters to cartesian space. 
					beamX = beamDistance*cos(beamAngle)
					beamY = beamDistance*sin(beamAngle)

					#Find the angular width of the probable landmark
					angularWidth = abs(angles[stoppingIndex]-angles[startingIndex])*180/pi

					#Calculate the theoretical angular width for known landmarks
					expectedWidth = 2*atan2(self.cylinderRadius,distances[startingIndex])*180/pi

					# Find acceptable levels of deviation from the angular width
					# Check if it is near range
					if distances[startingIndex] > 1:
						#Find the maximum error possible by taking the tangential beam 
						distanceError = distances[startingIndex] - self.cylinderRadius - self.shortRangeAccuracy
						#Calculate acceptable deviation
						angleDeviation = abs(self.unobservableLandmarkArea*2*atan2(self.cylinderRadius,distanceError)*180/pi)
					else:
						#Find the maximum error possible by taking the tangential beam 
						distanceError = distances[startingIndex] - self.cylinderRadius - self.longRangeAccuracy*distances[startingIndex]
						#Calculate acceptable deviation
						angleDeviation = abs(self.unobservableLandmarkArea*2*atan2(self.cylinderRadius,distanceError)*180/pi)


					# Compute least square fit on the model of a straight line. 
					samples=[(angles[i],distances[i]) for i in xrange(startingIndex,stoppingIndex)]
					xSamples,ySamples=self._pole_to_cart(samples) 
					xSamples=array(xSamples)
					ySamples=array(ySamples)  
					parameters,covariance=curve_fit(self._line,xSamples,ySamples)
					fittingError = linalg.norm(diag(covariance))

					#Check for different conditions:
					#		1) Check if there are enough points 
					#		2) Check if it's angular width is as expected
					#		3) Check if it is not a line, i.e. line fitting threshold is large
					
					if abs(expectedWidth-angularWidth) < angleDeviation \
							and len(samples) > self.minSamples \
							and fittingError > self.lineFittingThreshold:

						landmarks.append([beamAngle,beamDistance],1)

				else: 
					previousThresholdFound = True
					startingIndex = i 

		return landmarks
