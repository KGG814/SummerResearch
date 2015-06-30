from numpy import *
from math import atan2

############################
# Superclass for landmarks
############################
class LANDMARKS(object):
	"""Class to hold different type of land mark classes and generate respective objects"""

	@staticmethod
	def new_landmark(landmarkType,landmarkPosition,landmarkError,robotPosition, lidarData = None):
		""""Factory method to generate objects of the child"""

		if landmarkType == "Cylinder":
			return CYLINDER(landmarkPosition,landmarkError,robotPosition)
		elif landmarkType == "Wall":
			return WALL(landmarkPosition,landmarkError,robotPosition)
	  elif landmarkType == "Abstract":
	    return ABSTRACT(lidarData, landmarkError,robotPosition)

	def update_self(self,newPosition,newCovariance):
		self.position = newPosition
		self.covariance = newCovariance
		
###################################
# Categories for landmark types		
###################################W		
class POINT_FEATURES(LANDMARKS):
	
	def in_robot_frame(self,robotPosition):
		"""Takes a (x, y, theta) state and a (x, y) landmark, and returns the
			measurement (range, bearing)."""
		dx = self.position[0] - robotPosition[0]
		dy = self.position[1] - robotPosition[1]
		r = sqrt(dx * dx + dy * dy)
		alpha = (atan2(dy, dx) - robotPosition[2] + pi)%(2*pi)-pi
		return array([r, alpha])

	def position_jacobian(self,robotPosition):
		"""Differential of detection model with respect to the position of the sensor"""
		dx = self.position[0] - robotPosition[0]
		dy = self.position[1] - robotPosition[1]
		q = dx * dx + dy * dy
		sqrtq = sqrt(q)
		drdx = -dx / sqrtq
		drdy = -dy / sqrtq
		theta = robotPosition[2]
		cost, sint = cos(theta), sin(theta)
		drdtheta = 0
		dalphadx =  dy / q
		dalphady = -dx / q
		dalphadtheta = -1

		return array([[drdx, drdy, drdtheta],
						[dalphadx, dalphady, dalphadtheta]])

	def self_jacobian(self,robotPosition):
		"""Differential of the landmark with respect to itself"""
		dx = self.position[0] - robotPosition[0]
		dy = self.position[1] - robotPosition[1]
		q = dx * dx + dy * dy
		sqrtq = sqrt(q)
		drdx = -dx / sqrtq
		drdy = -dy / sqrtq
		dalphadx =  dy / q
		dalphady = -dx / q

		return array([[drdx, drdy],
						[dalphadx, dalphady]])
		
	@staticmethod
	def in_world_frame(point, robotPosition):
		"""Given a sensor pose (rx, ry, heading) and a point (theta, r) in the
			scanner's coordinate system, return the point's coordinates in the
			world coordinate system."""
		x, y = point[1]*cos(point[0]-pi/2), point[1]*sin(point[0]-pi/2)

		dx = cos(robotPosition[2])
		dy = sin(robotPosition[2])
		return array([x * dx - y * dy + robotPosition[0], x * dy + y * dx + robotPosition[1]])

class LINE_FEATURES(LANDMARKS):

	def in_robot_frame(self,robotPosition):
		"""Takes a (x, y, theta) state and a (x, y) landmark, and returns the
			measurement (range, bearing)."""
		xc,yc=self.position[0],self.position[1]
		xo,yo,tho=robotPosition[0],robotPosition[1],robotPosition[2]
		common = (xc*yo - xo*yc)/(xc**2 + yc**2)
		x1 = xc - yc*common
		y1 = yc + xc*common
		dx = x1 - xo
		dy = y1 - yo
		r     = hypot(dy, dx)
		alpha = (atan2(dy, dx) - robotPosition[2] + pi)%(2*pi)-pi
		return array([r, alpha])

	def position_jacobian(self,robotPosition):
		"""Differential of detection model with respect to the position of the sensor"""
		xc,yc=self.position[0],self.position[1]
		xo,yo,tho=robotPosition[0],robotPosition[1],robotPosition[2]

		common1 = 2*(- xc**2 + xo*xc - yc**2 + yo*yc)/(xc**2 + yc**2)
		drdxo = xc*common1
		drdyo = yc*common1
		drdtho = 0

		dalphadxo = 0
		dalphadyo = 0
		dalphadtho = -1

		return array([[drdxo, drdyo, drdtho],
						[dalphadxo, dalphadyo, dalphadtho]])

	def self_jacobian(self,robotPosition):
		"""Differential of the landmark with respect to itself"""
		xc,yc=self.position[0],self.position[1]
		xo,yo,tho=robotPosition[0],robotPosition[1],robotPosition[2]

		comm2 = (- xc**2 + xo*xc - yc**2 + yo*yc)/(xc**2 + yc**2)
		drdxc = -2*xc*comm2**2-2*(2*xc-xo)*comm2
		drdyc = -2*yc*comm2**2-2*(2*yc-yo)*comm2

		dalphadxc=-yc/(xc**2 + yc**2)
		dalphadyc=xc/(xc**2 + yc**2)

		return array([[drdxc, drdyc],
						[dalphadxc, dalphadyc]])
	
	@staticmethod
	def in_world_frame(point, robotPosition):
		"""Given a sensor pose (rx, ry, heading) and a point (x, y) in the
			scanner's coordinate system, return the point's coordinates in the
			world coordinate system."""
		dx = cos(robotPosition[2]-pi/2)
		dy = sin(robotPosition[2]-pi/2)
		x, y = point
		x0,y0=robotPosition[0],robotPosition[1]
		x1,y1=x * dx - y * dy + robotPosition[0], x * dy + y * dx + robotPosition[1]
		x2=((x0 - x1)*(- x1**2 + x0*x1 - y1**2 + y0*y1))/(x0**2 - 2*x0*x1 + x1**2 + y0**2 - 2*y0*y1 + y1**2)
		y2=((y0 - y1)*(- x1**2 + x0*x1 - y1**2 + y0*y1))/(x0**2 - 2*x0*x1 + x1**2 + y0**2 - 2*y0*y1 + y1**2)
		return array([x2,y2])

####################################################
# Different types of objects detected, 
# which instantiate one of the landmark categories
####################################################
class CYLINDER(POINT_FEATURES):
	"""Class for cylinder objects"""
		
	def __init__(self,cylinderPosition,error,robotPosition):
		self.position = self.in_world_frame(cylinderPosition,robotPosition)
		self.covariance = error
		self.mapIndex = 0
		self.stateIndex = 0
		self.dimension = 2
		self.max_distance = 1

	def check_if_in_map(self,ExistingList):
		"""Method to check if this landmark has been previously observed"""
		best_match = False
		best_distance = self.max_distance
		for Obj in ExistingList:
			if Obj.__class__.__name__ == "CYLINDER":
				dx,dy = abs(self.position[0] - Obj.position[0]), abs(self.position[1]
				            - Obj.position[1])
				dist = sqrt(dx**2 + dy**2)
				if dist < best_distance:
					best_distance = dist
					best_match = Obj
		return best_match
		
class ABSTRACT(POINT_FEATURES):
  """Class for abstract landmarks"""
  #TODO
  def __init__(self, lidarData, error,robotPosition):
    self.position = robotPosition
    self.covariance = error
    self.mapIndex = 0
    self.stateIndex = 0
    self.dimension = 2
    self.max_distance = 1
    self.lidarData = lidarData
    
  #TODO Remove landmarks that aren't close from consideration, and that aren't
  # abstract
  # Do a rigid transform to verify the landmark, pick the most likely one
  # Shift the lidar image, while shifting the landmark position accordingly
  def check_if_in_map(self, ExistingList)
		best_distance = self.max_distance
		for Obj in ExistingList:
			if Obj.__class__.__name__ == "ABSTRACT":
				dx,dy = abs(self.position[0] - Obj.position[0]), abs(self.position[1] - Obj.position[1])
				dist = sqrt(dx**2 + dy**2)
				if dist < best_distance:
					best_distance = dist
					best_match = Obj

class WALL(LINE_FEATURES):
	"""Class to hold the detected Wall landmarks"""

	def __init__(self,wallPosition,error,robotPosition):
		self.position = self.in_world_frame(wallPosition,robotPosition)
		self.covariance = error
		self.index = 0
		self.stateIndex = 0
		self.dimension = 2
		self.max_distance = 1
		self.max_angle = radians(45)

	def check_if_in_map(self,ExistingList):
		"""Method to check if this landmark has been previously observed"""
		best_match = False
		#FIXME
		best_distance = self.max_distance
		for Obj in ExistingList:
			if Obj.__class__.__name__ == "WALL":
				dx,dy = abs(self.position[0] - Obj.position[0]), abs(self.position[1] - Obj.position[1])
				dist = sqrt(dx**2 + dy**2)
				if dist < best_distance:
					best_distance = dist
					best_match = Obj

		try:
			if not best_match:
				print "Seperation:",dist
		except:
			pass
		return best_match
		
		# tSelf = abs((atan2(self.position[1], self.position[0])+pi)%(2*pi)-pi)
		# print "angle", degrees(tSelf)
		# for Obj in ExistingList:
		# 	if Obj.__class__.__name__=="WALL":
		# 		# print Obj.position
		# 		# print self.position
		# 		tObj = atan2(Obj.position[1], Obj.position[0])
		# 		tSelf = atan2(self.position[1], self.position[0])
		# 		best_angle = self.max_angle

		# 		# and ((Obj.position[1]>=0) == (self.position[1]>=0))
		# 		if ((Obj.position[0]>=0) == (self.position[0]>=0)) and abs((tObj - tSelf+pi)%(2*pi)-pi)<best_angle:
		# 			# print "Matched"
		# 			best_match = Obj
		# 			break
		
		# best_distance = self.max_distance
		# best_angle = self.max_angle
		# for Obj in ExistingList:
		# 	if Obj.__class__.__name__ == "WALL":
		# 		rObj = hypot(Obj.position[0], Obj.position[1])
		# 		tObj = atan2(Obj.position[1], Obj.position[0])
		# 		rSelf = hypot(self.position[0], self.position[1])
		# 		tSelf = atan2(self.position[1], self.position[0])
		# 		# abs(rObj - rSelf)<best_distance and 
		# 		if abs((tObj - tSelf+pi)%(2*pi)-pi)<best_angle:
		# 			best_distance = abs(rObj - rSelf)
		# 			best_angle = abs((tObj - tSelf+pi)%(2*pi)-pi)
		# 			best_match = Obj
		# return best_match
