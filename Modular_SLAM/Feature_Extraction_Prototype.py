import Libraries

class CLASS_NAME(object):
	"""docstring for CLASS_NAME"""
	def __init__(self, args):
		
		self.args = args
		self.landmarkType = "Type of Landmark"

	def get_landmarks(self, angles, distances):
		"""Finds landmark based on _____
			Takes in angles and distances as lists"""
		landmarkList=[]
		
		#Find landmarks
		#eg: landmarkPosition = [xL,yL]
		#	 probability = 0.3

		landmarkList.append((landmarkPosition,probability))
		return landmarkList

	#All other methods have to be either 
	#	static/class or start with _
