
import cv2
import numpy as np
from matplotlib import pyplot as plt
from math import asin,atan2,degrees,sqrt, sin 
import pickle
import os
import string
import math
import sys
from subprocess import Popen, PIPE

calibrationFile = 'CamCalib.pckl'
with open(calibrationFile) as f:
	obj = pickle.load(f)
(tf,camera_matrix,dist_coefs,Hom)=obj

LoweRatio = 0.8

def _cleanImage(image):
	h,  w = image.shape[:2]
	newcameramtx, roi=cv2.getOptimalNewCameraMatrix(camera_matrix,dist_coefs,(w,h),1,(w,h))
	dst = cv2.undistort(image, camera_matrix, dist_coefs, None, newcameramtx)
	x,y,w,h = roi
	dst = dst[y:y+h, x:x+w]
	return dst

def skew(a):
	""" Skew matrix A such that a x v = Av for any v. """
	return np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])

def compute_P_from_essential(E):
	""" Computes the second camera matrix (assuming P1 = [I 0])
	from an essential matrix. Output is a list of four
	possible camera matrices. """
	# make sure E is rank 2
	U,S,V = np.linalg.svd(E)
	if np.linalg.det(np.dot(U,V))<0:
		V = -V
	E = np.dot(U,np.dot(np.diag([1,1,0]),V))
	# create matrices (Hartley p 258)
	Z = skew([0,0,-1])
	W = np.array([[0,-1,0],[1,0,0],[0,0,1]])
	e = np.array(V[-1]).T
	t = e/e[2]
	# return all four solutions
	# P2 = [np.vstack((np.dot(U,np.dot(W,V)).T,U[:,2])).T,
	# 	np.vstack((np.dot(U,np.dot(W,V)).T,-U[:,2])).T,
	# 	np.vstack((np.dot(U,np.dot(W.T,V)).T,U[:,2])).T,
	# 	np.vstack((np.dot(U,np.dot(W.T,V)).T,-U[:,2])).T]

	P2 = [np.vstack((np.dot(U,np.dot(W,V)).T,t)).T,
		np.vstack((np.dot(U,np.dot(W,V)).T,-t)).T,
		np.vstack((np.dot(U,np.dot(W.T,V)).T,t)).T,
		np.vstack((np.dot(U,np.dot(W.T,V)).T,-t)).T]		
	return P2

def traceit(frame, event, arg):
    if event == "line":
        lineno = frame.f_lineno
        print "line", lineno

    return traceit
    
def main():
  # FLANN parameters
  # Not current used as OpenCV FLANN is bugged
  FLANN_INDEX_KDTREE = 0
  index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
  search_params = dict(checks=50)
  directory = os.getcwd() + "/HSI_Viewer.exe"
  pipe = Popen(directory, shell=True, bufsize=100, stdin=PIPE).stdin
  doorDetection=0
  roll=0
  pitch=0
  yaw=0
  firstTime = True
  cap = cv2.VideoCapture(1)
  
  while True:
    ret, currentImage = cap.read()

    if firstTime:
	    previousImage = currentImage[:,:]
	    previousImage = _cleanImage(previousImage)
	    previousImage = cv2.cvtColor(previousImage,cv2.COLOR_BGR2GRAY)
	    firstTime = False
	    continue

    currentImage = _cleanImage(currentImage)
    currentImage = cv2.cvtColor(currentImage,cv2.COLOR_BGR2GRAY)


	  # find the keypoints and descriptors with SIFT
    sift = cv2.xfeatures2d.SIFT_create()
    kp1, des1 = sift.detectAndCompute(currentImage,None)
    kp2, des2 = sift.detectAndCompute(previousImage,None)

    # Unused FLANN based matcher
    # matcher = cv2.FlannBasedMatcher(index_params,search_params)
    
    # Currently using a brute force matcher
    matcher = cv2.BFMatcher()
    matches = matcher.knnMatch(des1,des2,k=2)
    good = []
    pts1 = []
    pts2 = []
    
    
    # ratio test as per Lowe's paper
    for i,(m,n) in enumerate(matches):
      if m.distance < LoweRatio*n.distance:
        good.append(m)
        pts2.append(kp2[m.trainIdx].pt)
        pts1.append(kp1[m.queryIdx].pt)
      
    # Find the fundamental matrix between the first set and second set of points
    F, mask = cv2.findFundamentalMat(np.float32(pts1),
                                     np.float32(pts2),cv2.FM_LMEDS)
                                     
    E = np.dot(np.dot(camera_matrix.T,F),camera_matrix)
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
    for H_possible in compute_P_from_essential(E):
      R = H_possible[:,0:3]
      roll_t = atan2(R[2,1],R[2,2])
      pitch_t =  atan2(-R[2,0],sqrt(R[2,1]**2+R[2,2]**2))
      yaw_t = atan2(R[1,0],R[0,0])
      t = H_possible[:,3]
      if (abs(roll_t) >= np.pi/2 or abs(pitch_t) >= np.pi/2 or 
          abs(yaw_t) >= np.pi/2): 
        continue
      else:
        break

    previousImage = currentImage[:,:]

    # If we detect a significant change, update our attitude
    if (abs(degrees(roll_t)) >= 1 or abs(degrees(pitch_t)) >= 1 or 
        abs(degrees(yaw_t)) >= 1):
      roll = roll + yaw_t
      pitch = pitch +roll_t
      yaw = yaw + pitch_t

    # If there is a large change detected, set attitude back to zero
    if (abs(degrees(roll_t)) >= 30 or abs(degrees(pitch_t)) >= 30 or 
        abs(degrees(yaw_t)) >= 30):
      roll = 0
      pitch = 0
      yaw = 0
    os.system('clear')
    print "roll: ",degrees(roll)
    print "pitch: ",degrees(pitch)
    print "yaw: ",degrees(yaw)

    pipe.write("%lf %lf %lf"%(roll,pitch,yaw))

  cap.release()

if __name__ == '__main__':
  #sys.settrace(traceit)
  main()