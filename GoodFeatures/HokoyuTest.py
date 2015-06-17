from __future__ import division
import sys
import math
import cv2
import numpy
import struct

cartesianPoints = []

with open("Hokoyu") as f:
  lidarData = f.readline().split()

numSamples = len(lidarData)
degreeStep = 240/(numSamples)
lidarData = map(float,lidarData)

for x in range(0, numSamples):
  angle = (x*degreeStep-30)/180*math.pi
  xCoord = lidarData[x]*math.cos(angle)
  yCoord = lidarData[x]*math.sin(angle)
  newPoint = (xCoord,yCoord)
  cartesianPoints.append(newPoint)
scalingFactor = 400//8
cartesianPoints = cartesianPoints

image = numpy.ones((400,400))*255

for point in cartesianPoints:
  x = int(point[0]*scalingFactor)+200
  y = int(point[1]*scalingFactor)+200
  image[x][y] = 0
cv2.imwrite("test.jpg", image)
cv2.imshow("image", image)
cv2.waitKey(0)