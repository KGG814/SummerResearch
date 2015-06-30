import Log_Manager
import cv2
from find_obj import filter_matches,explore_match
import numpy as np
from matplotlib import pyplot as plt
from math import asin,atan2,degrees,degrees, sin, cos, hypot, pi
import pickle
from numpy import *

Log_Manager.init("./Data/run5/SlamData0.txt","./Data/run5/Img_")

calibrationFile = 'CamCalib.pckl'
with open(calibrationFile) as f:
	obj = pickle.load(f)
(tf,camera_matrix,dist_coefs,Hom)=obj

#LoweRatio = 0.8
LoweRatio = 1

def get_image(t):
	path = "./Data/run5/Img_"
	imgName = path+str(t)+".jpg"
	img = cv2.imread(imgName)
	img = _cleanImage(img)
	img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	return img

def _cleanImage(image):
	h,  w = image.shape[:2]
	newcameramtx, roi=cv2.getOptimalNewCameraMatrix(camera_matrix,dist_coefs,(w,h),1,(w,h))
	dst = cv2.undistort(image, camera_matrix, dist_coefs, None, newcameramtx)
	x,y,w,h = roi
	dst = dst[y:y+h, x:x+w]
	return dst


def make_image(time_index):
	"""Get lidar data and make an image out of it"""

	Log_Manager.currentTime=Log_Manager.Data.timeStamps[time_index]
	angles,distances = Log_Manager.get_data("Lidar")
#Convert distances to centimeters
	distances=[d*100 for d in distances]

#Convert from polar t cartesian coordinates
	def _pole_to_cart(angles,distances):
		""" Converts from polar coordinates to cartesian coordinates"""
		cart=[]
		for i in xrange(0,len(angles)-1):
			angle = angles[i]
			distance = distances[i] 
			xs, ys = distance*cos(angle), distance*sin(angle)
			cart.append(tuple((xs,ys)))
		return cart
	points=_pole_to_cart(angles,distances)

# Create new blank 1000x1000 black image
	image = zeros((1000, 1000, 3), uint8)
	trans_matrix = array([[0,-1,500],[1,0,500],[0,0,1]])
# Place White dots for each Lidar reading
	for (xo,yo) in points:
		lid_pnt=array([xo,yo,1])
		img_pnt=floor(dot(trans_matrix,lid_pnt))
		image[img_pnt[0],img_pnt[1]]=(255,255,255)

	image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	return image

t1=20
t2=40
# img1 = get_image(t1)
# img2 = get_image(t2)
img1 = make_image(t1)
img2 = make_image(t2)
# img1 = img1[240:480,:]
# img2 = img2[240:480,:]
# img1 = cv2.imread('myleft.jpg',0)  #queryimage # left image
# img2 = cv2.imread('myright.jpg',0) #trainimage # right image

sift = cv2.SIFT()

# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)

des1 = cv2.goodFeaturesToTrack(img1,25,0.01,10)
des2 = cv2.goodFeaturesToTrack(img2,25,0.01,10)
#corners = np.int0(corners)

# for i in corners:
#     x,y = i.ravel()
#     cv2.circle(img,(x,y),3,255,-1)

# FLANN parameters
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50)

flann = cv2.FlannBasedMatcher(index_params,search_params)
matches = flann.knnMatch(des1,des2,k=2)

good = []
pts1 = []
pts2 = []


# ratio test as per Lowe's paper
for i,(m,n) in enumerate(matches):
    if m.distance < LoweRatio*n.distance:
        good.append(m)
        pts2.append(kp2[m.trainIdx].pt)
        pts1.append(kp1[m.queryIdx].pt)

# pts1 = np.float32(pts1)
# pts2 = np.float32(pts2)

# F, mask = cv2.findFundamentalMat(pts1,pts2,cv2.FM_LMEDS)


# K = np.matrix(camera_matrix)
# E = K.T*F*K

# u,s,v=np.linalg.svd(E)
# W=np.matrix([	[0,-1,0],
# 				[1,0,0],
# 				[0,0,1]])
# Z=np.matrix([	[0,1,0],
# 				[-1,0,0],
# 				[0,0,1]])

# R = u*W*v.T
# t=u[:,2]
# #R = R/t[2]
# #t=t/t[2]

# roll = atan2(R[2,1],R[2,2])
# pitch =  asin(-R[2,0])
# yaw = atan2(R[0,0],R[1,0])
# # print "F: ",F
# # print "K: ",K
# # print "camera_matrix: ",camera_matrix
# # print "E: ",E
# # #print "u: ",u
# print "s: ",s
# # #print "v: ",v
# print "R: ",R
# print "t: ",t
# print "roll: ",degrees(roll)
# print "pitch: ",degrees(yaw)
# print "yaw: ",degrees(yaw)

##----------------------------------Visualizations-------------------------------------------

# p1, p2, kp_pairs = filter_matches(kp1, kp2, matches, ratio = LoweRatio)
# print kp_pairs
# explore_match('find_obj', img1,img2,kp_pairs)#, status=mask)
cv2.waitKey()
cv2.destroyAllWindows()
