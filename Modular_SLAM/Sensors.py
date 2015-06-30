import Log_Manager
from numpy import *
from FeatureDetectors import *
from Landmarks import *
import pickle

class LIDAR(object):
	"""Class for LIDAR related Functions. 
		Has to contain a list of feature detection objects for this lidar
		Has to contain an observe function that returns list of landmarks seen by this lidar. """

	def __init__(self,minLidarRange,maxLidarRange,lidarOffset,rangeError,bearingError,VisualizerObject):
		self.minLidarRange = minLidarRange 	#Min Range of lidar (m)
		self.maxLidarRange = maxLidarRange 	#Max range of lidar (m) 
		self.lidarOffset   = lidarOffset   	#Distance from center of GMM (m)

		self.lidarError    = diag([rangeError,bearingError])	#Base error for all landmarks observed by this lidar
		self.Visualizer    = VisualizerObject
		self.detectors     = []				#List to hold feature detection objects
 
	def observe(self,robotPosition):
		"""Method to iterate through all the feature detection objects associated with this sensor.
			Returns the list of landmark objects observed"""
		observedLandmarks = []
		angles,distances = Log_Manager.get_data("Lidar")
		# # Remove spurious readings that are out of range
		for i in xrange(len(angles)):
			if not self.minLidarRange < distances[i] < self.maxLidarRange:
				angles[i] = -1
				distances[i] = -1
		angles    = filter(lambda a: a != -1, angles)
		distances = filter(lambda d: d != -1, distances)
		# Compute lidar position from robot position.
		lidarPosition = (
			robotPosition[0] + cos(robotPosition[2]) * self.lidarOffset,
			robotPosition[1] + sin(robotPosition[2]) * self.lidarOffset,
			robotPosition[2])

		for Obj in self.detectors:
			landmarkType = Obj.landmarkType		#Find out the type of landmark the detector is going to return
			landmarks = Obj.get_landmarks(angles,distances)		#Get a list of landmarks in form of tuples [(position, certainty),...]
			# print "Length of landmark list:"
			# print len(landmarks)
			if landmarks:
				for (landmarkPosition,certainty) in landmarks:
					#Calculate error for each landmark and convert to world coordinates
					landmarkError = self.lidarError * certainty
					#Create an object of the landmark type and add to list
					landmarkObject = LANDMARKS.new_landmark(landmarkType,landmarkPosition,landmarkError,lidarPosition)
					observedLandmarks.append(landmarkObject)
		self.Visualizer.plot_scan(angles,distances,landmarks)
		return observedLandmarks

class CAMERA(object):
	"""Class for CAMERA related Functions. 
		Has to contain a list of feature detection objects for this lidar
		Has to contain an observe function that returns list of landmarks seen by this lidar. """

	def __init__(self,calibrationFile,cameraError,cameraOffset,VisualizerObject):

		#Files containing camera information
		self.calibrationFile = calibrationFile 		

		self.cameraOffset   = cameraOffset   	#Distance from center of GMM (m)

		self.cameraError   = cameraError	#Base error for all landmarks observed by this lidar
		self.Visualizer    = VisualizerObject
		self.detectors     = []				#List to hold feature detection objects
 
	def _cleanImage(self, image):
		with open(self.calibrationFile) as f:
			obj = pickle.load(f)
		(tf,camera_matrix,dist_coefs,Hom)=obj
		h,  w = image.shape[:2]
		newcameramtx, roi=cv2.getOptimalNewCameraMatrix(camera_matrix,dist_coefs,(w,h),1,(w,h))
		dst = cv2.undistort(image, camera_matrix, dist_coefs, None, newcameramtx)
		x,y,w,h = roi
		dst = dst[y:y+h, x:x+w]
		return dst

	def observe(self,robotPosition):
		"""Method to iterate through all the feature detection objects associated with this sensor.
			Returns the list of landmark objects observed"""
		observedLandmarks = []
		originalImage = Log_Manager.get_data("Camera")
		# # Rectify Image to remove spherical distortion 
		image = self._cleanImage(image)
		# Compute camera position from robot position.
		cameraPosition = (
			robotPosition[0] + cos(robotPosition[2]) * self.cameraOffset,
			robotPosition[1] + sin(robotPosition[2]) * self.cameraOffset,
			robotPosition[2])

		for Obj in self.detectors:
			landmarkType = Obj.landmarkType		#Find out the type of landmark the detector is going to return
			landmarks = Obj.get_landmarks(image)		#Get a list of landmarks in form of tuples [(position, certainty),...]
			print "Length of camera landmark list:",len(landmarks)
			
			for (landmarkPosition,certainty) in landmarks:
				#Calculate error for each landmark and convert to world coordinates
				landmarkError = self.cameraError * certainty
				#Create an object of the landmark type and add to list
				landmarkObject = LANDMARKS.new_landmark(landmarkType,landmarkPosition,landmarkError,lidarPosition)
				observedLandmarks.append(landmarkObject)

		#self.Visualizer.plot_image(image,landmarks)
		return observedLandmarks
