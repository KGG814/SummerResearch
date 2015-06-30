import Log_Manager
from numpy import *
from FeatureDetectors import *


Log_Manager.init("./Data/run2/SlamData.txt")

Log_Manager.currentTime=Log_Manager.Data.timeStamps[150]

#Extractor 2: Ransac method. Detect straight lines in Lidar data.
numberOfTrials = 100			#Number of times to run the algorithm
neighborhood = radians(15)		#Range around each randomly selected point to find a line (deg)
samples = 5						#Number of samples within neighborhood used for least squares curve fitting
consensus = 50 					#Number of points that has to lie on a particular line to be considered
errorRange = 0.05 				#Distance around the wall that can be assumed part of the wall. (m)

RansacExtractor = RANSAC(numberOfTrials,neighborhood,samples,consensus,errorRange) 

angles,distances = Log_Manager.get_data("Lidar")
walls = RansacExtractor.get_landmarks(angles,distances)
wallPositions = [p for (p,c) in walls]
print len(wallPositions)