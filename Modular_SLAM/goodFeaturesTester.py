from Log_Manager import *
from matplotlib.pyplot import *
from FeatureDetectors import *


derivativeThreshold = 0.5
cylinderRadius = 0.1524
shortRangeAccuracy = 0.03
longRangeAccuracy = 0.03
unobservableLandmarkArea = 0.4
lineFittingThreshold = 0.09
minSamples = 11
		
derivativeExtractor = DERIVATIVES(derivativeThreshold, cylinderRadius, shortRangeAccuracy, longRangeAccuracy, unobservableLandmarkArea, lineFittingThreshold, minSamples)

data=LOG_FILE()
data.read("./Data/run5/SlamData0.txt")
ion()
figureObj = figure()
plotObj   = figureObj.add_subplot(111,polar=True)
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
