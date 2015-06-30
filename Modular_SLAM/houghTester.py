from Log_Manager import *
from matplotlib.pyplot import *
from FeatureDetector_temp import *


#Extractor 3: Hough Transform to detect straight lines in Lidar data.
maxRange 		 = 500			# Max range of lidar data given to this extrator (cm)
radialResolution = 2 			# Distance resolution of line in cm
angularResolution= radians(10)	# Angular resolution of lines
threshold		 = 50 			# Min Number of points to be on a line to be considered

HoughExtractor = HOUGH(maxRange,radialResolution,angularResolution,threshold)


data=LOG_FILE()
data.read("./Data/run5/SlamData0.txt")
ion()
figureObj = figure()
plotObj   = figureObj.add_subplot(111)
show()


for t in data.timeStamps:

	cla()
	angles,distances = data.scanAngles[t],data.scanDistances[t]
	plotObj.scatter(angles,distances,marker=u'x',c='g')

	landmarks = derivativeExtractor.get_landmarks(angles,distances)
	
	for (landmarkPosition,certainity) in landmarks:
		plotObj.scatter(landmarkPosition[0],landmarkPosition[1],marker='o',c='r',s=50)
	
	plotObj.set_rmax(5)
	
	draw()
	print "Press any key:"
	raw_input()
	#pause(0.0000000001)
