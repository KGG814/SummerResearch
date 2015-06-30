from Log_Manager import *
from numpy import *
from cv2 import *
from matplotlib import pyplot as plt
import time
from math import atan2,hypot

def print_timing(func):
    def wrapper(*arg):
        t1 = time.time()
        res = func(*arg)
        t2 = time.time()
        print '%s took %0.3f ms' % (func.func_name, (t2-t1)*1000.0)
        return res
    return wrapper

def _cart_to_pole(xPoints,yPoints):
	rPoints=[]
	tPoints=[]
	for (x,y) in zip(xPoints,yPoints):
		tPoints.append(atan2(y,x))
		rPoints.append(hypot(x,y))
	return tPoints,rPoints


def _pole_to_cart(angles,distances):
	cart=[]
	for i in xrange(0,len(angles)-1):
		angle = angles[i]
		distance = distances[i] 
		xs, ys = distance*cos(angle), distance*sin(angle)
		cart.append(tuple((xs,ys)))
	return cart


#@print_timing
def get_circles(angles,distances):
	
	distances=[d*100 for d in distances]
	points=_pole_to_cart(angles,distances)

	# Create new blank 1000x1000 white image
	image = zeros((height, width, 3), uint8)

	for (xo,yo) in points:
		#ax2.plot(xo/100,yo/100,'rx')
		lid_pnt=array([xo,yo,1])
		img_pnt=floor(dot(trans_matrix,lid_pnt))
		image[img_pnt[0],img_pnt[1]]=(255,255,255)

	circles = HoughCircles(image[:,0,0],cv.CV_HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)

	return image,circles

width, height = 1000, 1000
trans_matrix=array([[0,-1,500],[1,0,500],[0,0,1]])
inv_trans_matrix=linalg.inv(trans_matrix)# array([[0,1,500],[-1,0,500],[0,0,1]])

plt.ion()
fig1=plt.figure(1)
#ax1 = fig1.add_subplot(121,polar=True)
#ax2=fig1.add_subplot(132)


if __name__ == '__main__':
	data=LOG_FILE()
	data.read("./Data/run2/SlamData.txt")
	# plt.show()
	for t in data.timeStamps:
		angles,distances = data.scanAngles[t],data.scanDistances[t]
		for i in xrange(len(angles)):
			if not 0.2 < distances[i] < 5:
				angles[i] = -1
				distances[i] = -1
		angles    = filter(lambda a: a != -1, angles)
		distances = filter(lambda d: d != -1, distances)


		image,circles, = get_circles(angles,distances)
		try:
			print len(circles[0])
			circles = uint16(around(circles))
			for i in circles[0,:]:
				# draw the outer circle
				circle(image,(i[0],i[1]),i[2],(0,255,0),2)
				# draw the center of the circle
				circle(image,(i[0],i[1]),2,(0,0,255),3)

		except:
			print "no lines"
			pass
		
		#plt.title('Cleaned Image'), plt.xticks([]), plt.yticks([])
		plt.subplot(111),plt.imshow(image)
		plt.title('Image with Lines'), plt.xticks([]), plt.yticks([])
		plt.draw()
		plt.pause(0.00001)
		plt.cla()

	raw_input()