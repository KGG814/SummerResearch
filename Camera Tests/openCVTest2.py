#!/usr/bin/env python

import cv2
import numpy as np
 
c = cv2.VideoCapture(1)
_,f = c.read()
 
avg1 = np.float32(f)
avg2 = np.float32(f)
 
while(1):
    _,f = c.read()
     
    cv2.accumulateWeighted(f,avg2,0.1)
     
    res2 = cv2.convertScaleAbs(avg2)
 
    cv2.imshow('img',f)
    cv2.imshow('avg2',res2)
    k = cv2.waitKey(20)
 
    if k == 27:
        break
 
cv2.destroyAllWindows()
c.release()
