import Log_Manager		#Only for OffLine 
from Algorithms import *
from Robots import *
from Proprioceptives import *
from Sensors import *
from FeatureDetectors import *
from Visualization import *
from matplotlib import pyplot as plt
from math import radians
from numpy import *

def main():

#Only for off-line SLAM [replaced for on-line]
	Log_Manager.init("./Data/run12/SlamData.txt","./Data/run11_th/Image_")
#----------------------------------------------

#Set up Visualizer/logger
	maxArenaLengthRange = [-1.3716,6.858]
	maxArenaWidthRange  = [-1.3716,3.2004]
	Visualizer=VISUALIZER(maxArenaWidthRange, maxArenaLengthRange)
	

	Storage = Log_Manager.STORE_DATA("./Data/run10_th/corr.txt")


#Create all object of type ROBOT with it's initial position and error	

	#ROBOT 1: General Mobile Manipulator [GMM]
	####-0.0842518,0.0402793
	initialPosition = array([0,0,radians(90)])
	#initialPosition = array([0,-0.475,radians(0)])
	#initialPosition = array([-0.239103, -0.262773,radians(0)])
	initialCov = zeros((3,3))
	#initialCov = diag([0.5,0.5,radians(10)])
	Gmm = ROBOT(initialPosition,initialCov)

#Create objects of each of all sensors

	#SENSOR 1: Hokoyu Lidar on GMM
	minLidarRange = 0.2			#Min Range of hokoyu (m)
	maxLidarRange = 5			#Max range of hokoyu (m) 
	lidarPosition = 0.1778		#Distance from center of GMM (m)
	rangeError    = .05 		#Error in distance measurement (m)
	bearingError  = radians(0.01)	#Error in angle measurement (deg)
	
	Hokoyu = LIDAR(minLidarRange,maxLidarRange,lidarPosition,rangeError,bearingError,Visualizer)

	#SENSOR 2: Encoders on GMM wheels
	wheelDia = 0.1778			#Diameter of Wheels (m)
	wheelSeperation = 0.4318	#Width of the robot (m)
	motionFactor = 100			#Error due to encoder reading  (m /m)
	turnFactor = 50		    #Additional Error due to wheel slippage during turns  (m /m)

	Encoder = ENCODER(wheelDia,wheelSeperation,motionFactor,turnFactor)

	#SENSOR 3: Camera on GMM
	calibrationFile = 'CamCalib.pckl'	#File containing Calibration constants
	cameraError = 1 					#Level of trust in Camera. 0-1

	#OnBoard = CAMERA(calibrationFile,cameraError,Visualizer)

#Create Feature extractor objects and assign them to the sensors that use them.

	#Extractor 1: Spike method. Use Lidar data to find prominent features.
	prominenceFactor = 50		#Difference in distance from the robot that registers as a feature (m)
	avgWidth         = 5 		#Average number of rays that hit a feature
	SpikeExtractor = SPIKE(prominenceFactor,avgWidth)
	#Hokoyu.detectors.append(SpikeExtractor)

	#Extractor 2: Ransac method. Detect straight lines in Lidar data.
	numberOfTrials = 100			#Number of times to run the algorithm
	neighborhood   = radians(15)	#Range around each randomly selected point to find a line (deg)
	samples        = 5				#Number of samples within neighborhood used for least squares curve fitting
	consensus      = 200 			#Number of points that has to lie on a particular line to be considered
	errorRange     = 0.1 			#Distance around the wall that can be assumed part of the wall. (m)
	avgWidth       = 200			#Average number of rays that hit a feature

	RansacExtractor = RANSAC(numberOfTrials,neighborhood,samples,consensus,errorRange,avgWidth) 
	#Hokoyu.detectors.append(RansacExtractor)

	#Extractor 3: Hough Transform to detect straight lines in Lidar data.
	maxRange 		 = 500			# Max range of lidar data given to this extrator (cm)
	radialResolution = 2 			# Distance resolution of line in cm
	angularResolution= radians(10)	# Angular resolution of lines
	threshold		 = 30 			# Min Number of points to be on a line to be considered

	HoughExtractor = HOUGH(maxRange,radialResolution,angularResolution,threshold)
	Hokoyu.detectors.append(HoughExtractor)

	#Extractor 4: Hough transform for lines in images
	#parameters for median blur 
	kernalSize = 11							#Aperture linear size; it must be odd and greater than 1, for example: 3, 5, 7 ...

	#adaptiveThreshold parameters
	thresholdingBlockSize = 15				#Size of a pixel neighborhood that is used to calculate a threshold value for the pixel: 3, 5, 7, and so on.
	thresholdingConstant = 2				#Constant subtracted from the mean or weighted mean 

	#Canny edge detector parameters
	edgeThreshold1 = 100					#First threshold for the hysteresis procedure
	edgeThreshold2 = 150					#Second threshold for the hysteresis procedure
	edgeApertureSize = 3					#Aperture size for the Sobel() operator

	#HOUGH transform parameters
	angleRange =  pi/180					#Angle resolution of the accumulator in radians
	distanceRange = 1 						#Distance resolution of the accumulator in pixels
	lineThreshold =  100					#Accumulator threshold parameter. Only those lines are returned that get enough votes (>threshold)

	fieldOfView = radians(72)				#Horizontal Field of View

	angularError = radians(5)				#Error in angle of detected line

	#imgLineDetector = IMAGE_LINES(kernalSize,thresholdingBlockSize,thresholdingConstant,edgeThreshold1,edgeThreshold2,edgeApertureSize,angleRange,distanceRange,lineThreshold,fieldOfView,angularError)
	#OnBoard.detectors.append(imgLineDetector)

	#Extractor 5: Corner detection on LIDAR
	CornerExtractor = CORNER_EXTRACTOR(maxRange,radialResolution,angularResolution,threshold)
	Hokoyu.detectors.append(CornerExtractor)

#Assign sensors to the respective robots
	
	#ROBOT 1:
	#Assign proprioceptive sensors
	Gmm.pSensors.append(Encoder) 
	#Assign exteroceptive sensors
	Gmm.eSensors.append(Hokoyu)
	#Gmm.eSensors.append(OnBoard)

#Initialize SLAM Algorithm
	Ekf = EXTENDED_KALMAN_FILTER(Visualizer)

#Add all robots into the map
	index = Ekf.add_to_map(Gmm)

## GENERATE KNOWN LANDMARK OBJECTS AND ADD TO MAP HERE

#Begin SLAM algorithm
	for t in Log_Manager.Data.timeStamps[50:300]:#[260:270]:#[350:1500]:
		print "timestamp: ",Log_Manager.Data.timeStamps.index(t)
		Log_Manager.currentTime=t 
		Ekf.run()
		# Visualizer.plot_map(Ekf.map)
		Visualizer.add_to_complete_map(Ekf.map[0])
		Storage.write_map(Ekf.map)
		#raw_input()
	print "SLAM Over"
	print len(Ekf.map)
	#Visualizer.plot_map(Ekf.map)
	Visualizer.plot_complete_map(Ekf.map)
	raw_input()
	Storage.release()


if __name__ == '__main__':
	main()