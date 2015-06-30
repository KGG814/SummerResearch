from Log_Manager import *
import numpy as np 
import cv2
from matplotlib import pyplot as plt
import time

def _pole_to_cart(angles,distances):
	cart=[]
	for i in xrange(0,len(angles)-1):
		angle = angles[i]
		distance = distances[i] 
		xs, ys = distance*np.cos(angle), distance*np.sin(angle)
		cart.append(tuple((xs,ys)))
	return cart



width, height = 1000, 1000
trans_matrix=np.array([[0,-1,500],[1,0,500],[0,0,1]])


data=LOG_FILE()
data.read("./Data/run2/SlamData.txt")
t=data.timeStamps[50]
angles,distances = data.scanAngles[t],data.scanDistances[t]
for i in xrange(len(angles)):
	if not 0.2 < distances[i] < 5:
		angles[i] = -1
		distances[i] = -1
angles    = filter(lambda a: a != -1, angles)
distances = filter(lambda d: d != -1, distances)

distances=[d*100 for d in distances]
points=_pole_to_cart(angles,distances)

# Create new blank 1000x1000 white image
image = np.zeros((height, width, 3), np.uint8)

for (xo,yo) in points:
	#ax2.plot(xo/100,yo/100,'rx')
	lid_pnt=np.array([xo,yo,1])
	img_pnt=np.floor(np.dot(trans_matrix,lid_pnt))
	image[img_pnt[0],img_pnt[1]]=(255,255,255)



# contours, hierarchy = cv2.findContours(image[:,0,0].copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
# print contours
# cv2.drawContours(image, contours, -1, (0,255,0), 3)
circles = cv2.HoughCircles(image[:,0,0],cv2.cv.CV_HOUGH_GRADIENT,2,1)#,param1=50,param2=30,minRadius=0,maxRadius=0)
print circles
plt.subplot(111),plt.imshow(image)
plt.title('Image'), plt.xticks([]), plt.yticks([])
plt.show()