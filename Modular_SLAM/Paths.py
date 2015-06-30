from numpy import *

tempPath=[]
class PATH_CLASS(object):
	"""Class of Path and path based operation"""

	def __init__(self,dimension):
		self.pathHolder=[]
		self.dimension = dimension		#No of features the path holds

	def get_path(self):
		paths=[]
		for i in xrange(0,self.dimension):
			subPath=[]
			for pose in self.pathHolder:
				subPath.append(pose[i])
			paths.append(subPath)

		return paths

	def add_to_path(self,position):
		# print "Position"
		# print position
		self.pathHolder.append(position.copy())

