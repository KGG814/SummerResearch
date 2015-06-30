from Log_Manager import *
from matplotlib.pyplot import *
from FeatureDetectors import *


#Extractor 1: Spike method. Use Lidar data to find prominent features.
prominenceFactor = 3		#Difference in distance from the robot that registers as a feature (m)
avgWidth         = 2 		#Average number of rays that hit a feature
SpikeExtractor = SPIKE(prominenceFactor,avgWidth)

data=LOG_FILE()
data.read("./Data/run6/SlamData0.txt")
ion()
figureObj = figure()
plotObj1   = figureObj.add_subplot(121,polar=True)
plotObj2   = figureObj.add_subplot(122)

show()


for t in data.timeStamps[30:]:

	cla()
	angles,distances = data.scanAngles[t],data.scanDistances[t]

	for i in xrange(len(angles)):
		if not 0.3 < distances[i] < 3:
			angles[i] = -1
			distances[i] = -1
	angles    = filter(lambda a: a != -1, angles)
	distances = filter(lambda d: d != -1, distances)
	for (a,d) in zip(angles,distances):
		plotObj1.plot([a,a],[0,d],'g-')
	plotObj1.scatter(angles,distances,marker='x',c='g')
	
	#plotObj1.plot(angles,distances,'-x')
	# # Remove spurious readings that are out of range

	landmarks,derivative = SpikeExtractor.get_landmarks(angles,distances)
	print "landmarks",landmarks
	plotObj2.plot(angles,distances,'b',label='Scan distances')
	#plotObj2.plot(angles,derivative,'g',label = 'Derivarive of distances')
	plotObj2.set_xlabel(r'Angles($\theta$)$\rightarrow$')
	plotObj2.set_ylabel(r'Distances(m)$\rightarrow$')
	plotObj2.legend(loc=2)
	for (landmarkPosition,certainity) in landmarks:
		plotObj2.scatter(landmarkPosition[0],landmarkPosition[1],marker='o',c='r',s=50)
		plotObj1.scatter(landmarkPosition[0],landmarkPosition[1],marker='o',c='r',s=50)
	
	plotObj1.set_rmax(3)
	
	draw()
	raw_input()
	#pause(0.0000000001)
