from Log_Manager import *
from numpy import *
from cv2 import *
from matplotlib import pyplot as plt
import time

def print_timing(func):
    def wrapper(*arg):
        t1 = time.time()
        res = func(*arg)
        t2 = time.time()
        print '%s took %0.3f ms' % (func.func_name, (t2-t1)*1000.0)
        return res
    return wrapper


def _pole_to_cart(angles,distances):
	cart=[]
	for i in xrange(0,len(angles)-1):
		angle = angles[i]
		distance = distances[i] 
		if 0<distance<5:
			distance = distance*100
			xs, ys = distance*cos(angle), distance*sin(angle)
			cart.append(tuple((xs,ys)))
	return cart


@print_timing
def get_lines(angles,distances):
	
	points=_pole_to_cart(angles,distances)
	# Create new blank 1000x1000 white image
	image = zeros((height, width, 3), uint8)

	for (xo,yo) in points:
		lid_pnt=array([xo,yo,1])
		img_pnt=floor(dot(trans_matrix,lid_pnt))
		image[img_pnt[0],img_pnt[1]]=(255,255,255)

	#image1 = morphologyEx(image, MORPH_CLOSE, kernel)
	image1 = dilate(image,kernel,iterations = 2)
	image1 = erode(image1,kernel1,iterations = 1)
	#edges = Canny(image1,100,150,apertureSize = 3,L2gradient= True)
	minLineLength = 30
	maxLineGap = 10
	lines = HoughLinesP(image1[:,:,0],1,pi/180,80,minLineLength=minLineLength,maxLineGap=maxLineGap)[0]
	return image,lines,image1

width, height = 1000, 1000
trans_matrix=array([[0,-1,500],[1,0,500],[0,0,1]])
kernel = getStructuringElement(MORPH_RECT,(5,5))
kernel1 = getStructuringElement(MORPH_CROSS,(5,5))


data=LOG_FILE()
data.read("./Data/run2/SlamData.txt")
# plt.ion()
# plt.figure(1)
# plt.show()
for t in data.timeStamps:
#t=data.timeStamps[50]
	
	angles,distances = data.scanAngles[t],data.scanDistances[t]
	image,lines,image1 = get_lines(angles,distances)
	if lines.any():
		print len(lines)
		for x1,y1,x2,y2 in lines:
			line(image,(x1,y1),(x2,y2),(0,255,0),2)

	# corners = goodFeaturesToTrack(edges,1,0.5,3)
	# corners = int0(corners)

	# for i in corners:
	# 	x,y = i.ravel()
	# 	circle(image,(x,y),3,255,-1)

	# im = imread('50.jpg')
	# imgray = cvtColor(im,COLOR_BGR2GRAY)
	# ret,thresh = threshold(imgray,127,255,0)
	# contours,hierarchy = findContours(edges,RETR_TREE,CHAIN_APPROX_SIMPLE)
	# print len(contours)
	# drawContours(image, contours, -1, (0,255,0), 3)

	plt.subplot(121),plt.imshow(image1)
	plt.title('Image'), plt.xticks([]), plt.yticks([])
	plt.subplot(122),plt.imshow(image)
	plt.title('Image'), plt.xticks([]), plt.yticks([])
	plt.draw()
	plt.pause(0.00001)

raw_input()

