from __future__ import division
from FeatureDetectors import *
import Log_Manager
import cv2

Log_Manager.init("./SlamData.txt","./Image_")
maxArenaLengthRange = [-1.3716,6.858]
maxArenaWidthRange  = [-1.3716,3.2004]
Storage = Log_Manager.STORE_DATA("./corr.txt")

################

with open("Hokoyu") as f:
  distances = f.readline().split()

numSamples = len(distances)
degreeStep = 240/(numSamples-1)
distances = map(float, distances)
angles = []
for x in range(0, numSamples):
    angles.append(radians(x*degreeStep-30))

maxRange         = 500          # Max range of lidar data given to this extrator (cm)
radialResolution = 2            # Distance resolution of line in cm
angularResolution= radians(10)  # Angular resolution of lines
threshold        = 50           # Min Number of points to be on a line to be considered

CornerExtractor = CORNER_EXTRACTOR(maxRange,radialResolution,angularResolution,threshold)
for t in Log_Manager.Data.timeStamps[50:176]:
	print "timestamp: ",Log_Manager.Data.timeStamps.index(t)
	Log_Manager.currentTime=t
	angles,distances = Log_Manager.get_data("Lidar")
	features, image = CornerExtractor.get_landmarks(angles, distances)
	#print features
	cv2.imshow("image1", image)
	cv2.waitKey(0)