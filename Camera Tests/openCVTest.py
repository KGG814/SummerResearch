#!/usr/bin/env python

import cv2
import scipy.ndimage.filters as scp
import numpy as np
np.set_printoptions(threshold='nan')
import math
import sys
from scipy import ndimage



def diffImg(t0, t1, t2):
  d1 = cv2.absdiff(t2, t1)
  d2 = cv2.absdiff(t1, t0)
  diff = cv2.bitwise_and(d1, d2)
  mvt = np.where(diff >= 50)
  print np.mean(diff)
  sys.stdout.write("\033[F")
  sys.stdout.write("\033[K")
  return diff, mvt


cam = cv2.VideoCapture(1)

winName1 = "Movement Indicator (Unfiltered)"
cv2.namedWindow(winName1, cv2.WINDOW_AUTOSIZE)
#winName2 = "Movement Indicator (Gaussian)"
#cv2.namedWindow(winName2, cv2.WINDOW_AUTOSIZE)
#winName3 = "Current Image"
#cv2.namedWindow(winName3, cv2.WINDOW_AUTOSIZE)

t0 = cv2.cvtColor(cam.read()[1], cv2.COLOR_RGB2GRAY)
t1 = cv2.cvtColor(cam.read()[1], cv2.COLOR_RGB2GRAY)
t2 = cv2.cvtColor(cam.read()[1], cv2.COLOR_RGB2GRAY)
while True:
  image = cam.read()[1]
  
  #image2 = np.copy(image)
  diff, mvt = diffImg(t0, t1, t2)
  #b = np.zeros([480,640,3], dtype=np.uint8)
  for x, y in zip(mvt[0],mvt[1]):
    image[x][y] = (0, 115, 255)
    #b[x,y,0] = image2[x,y,0]
    #b[x,y,1] = image2[x,y,1]
    #b[x,y,2] = image2[x,y,2]
  cv2.imshow(winName1,image)
  #cv2.imshow(winName2,diff)
  t0 = t1
  t1 = t2
  t2 = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
  #t2 = scp.uniform_filter(t2, size=5, mode='constant')
  key = cv2.waitKey(10)
  if key == 27:
    cv2.destroyWindow(winName2)
    cv2.destroyWindow(winName3)
    break




