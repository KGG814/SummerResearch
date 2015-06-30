import cv2
from matplotlib.pyplot import*
from numpy import *
import pickle
from FeatureDetectors import *

calibrationFile = 'CamCalib.pckl'

#parameters for median blur 
kernalSize = 11							#Aperture linear size; it must be odd and greater than 1, for example: 3, 5, 7 ...

#adaptiveThreshold parameters
thresholdingBlockSize = 15		#Size of a pixel neighborhood that is used to calculate a threshold value for the pixel: 3, 5, 7, and so on.
thresholdingConstant = 2		#Constant subtracted from the mean or weighted mean 

#Canny edge detector parameters
edgeThreshold1 = 100					#First threshold for the hysteresis procedure
edgeThreshold2 = 150					#Second threshold for the hysteresis procedure
edgeApertureSize = 3				#Aperture size for the Sobel() operator

#HOUGH transform parameters
angleRange =  pi/180							#Angle resolution of the accumulator in radians
distanceRange = 1 						#Distance resolution of the accumulator in pixels
lineThreshold =  100						#Accumulator threshold parameter. Only those lines are returned that get enough votes (>threshold)

fieldOfView = 72							#Horizontal Field of View

angularError = 1

detectorObj = IMAGE_LINES(kernalSize,thresholdingBlockSize,thresholdingConstant,edgeThreshold1,edgeThreshold2,edgeApertureSize,angleRange,distanceRange,lineThreshold,fieldOfView,angularError)


def vis(img):
	figObj = figure()
	imgObj = figObj.add_subplot(111)
	imgObj.set_xticks([])
	imgObj.set_yticks([])
	imgObj.imshow(img)
	draw()

def _cleanImage(image):
	with open(calibrationFile) as f:
		obj = pickle.load(f)
	(tf,camera_matrix,dist_coefs,Hom)=obj
	h,  w = image.shape[:2]
	newcameramtx, roi=cv2.getOptimalNewCameraMatrix(camera_matrix,dist_coefs,(w,h),1,(w,h))
	dst = cv2.undistort(image, camera_matrix, dist_coefs, None, newcameramtx)
	x,y,w,h = roi
	dst = dst[y:y+h, x:x+w]
	return dst

def get_points(rho,theta=0):
	points = []
	a = np.cos(theta)
	b = np.sin(theta)
	x0 = a*rho
	y0 = b*rho
	points.append((x0,y0))
	delta = 100
	for i in xrange(1,5):
		x1 = int(x0 + i*delta*(-b))
		y1 = int(y0 + i*delta*(a))
		x2 = int(x0 - i*delta*(-b))
		y2 = int(y0 - i*delta*(a))
	points.append((x1,y1))
	points.append((x2,y2))
	return points

def get_image(t):
	path = "./Data/run5/Img_"
	imgName = path+str(t)+".jpg"
	img = cv2.imread(imgName)
	image = _cleanImage(img)
	return image

def drawLines(image,rho,theta=0):
	a = np.cos(theta)
	b = np.sin(theta)
	x0 = a*rho
	y0 = b*rho
	x1 = int(x0 + 1000*(-b))
	y1 = int(y0 + 1000*(a))
	x2 = int(x0 - 1000*(-b))
	y2 = int(y0 - 1000*(a))
	cv2.line(image,(x1,y1),(x2,y2),(255,0,0),1)
	return image

def main():
	t1=70
	t2=75
	image_old = get_image(t1)
	image_new = get_image(t2)

	distances_old = detectorObj.get_landmarks(image_old)
	distances_old.sort()
	print distances_old
	old_points = []
	if distances_old:
		for d in distances_old:
			old_points=old_points + get_points(d)
			drawLines(image_old,d)
			print old_points
	vis(image_old)

	distances_new = detectorObj.get_landmarks(image_new)
	distances_new.sort()
	print distances_new
	new_points = []
	if distances_new:
		for d in distances_new:
			new_points=new_points + get_points(d)
			drawLines(image_new,d)
			print new_points
	vis(image_new)

	
	
	show()

if __name__ == '__main__':
	main()