import cv2
from matplotlib.pyplot import*
import numpy as np
import pickle
from FeatureDetectors import *

calibrationFile = 'CamCalib.pckl'

#parameters for median blur 
kernalSize = 11              #Aperture linear size; it must be odd and greater than 1, for example: 3, 5, 7 ...

#adaptiveThreshold parameters
thresholdingBlockSize = 15    #Size of a pixel neighborhood that is used to calculate a threshold value for the pixel: 3, 5, 7, and so on.
thresholdingConstant = 2    #Constant subtracted from the mean or weighted mean 

#Canny edge detector parameters
edgeThreshold1 = 100          #First threshold for the hysteresis procedure
edgeThreshold2 = 150          #Second threshold for the hysteresis procedure
edgeApertureSize = 3        #Aperture size for the Sobel() operator

#HOUGH transform parameters
angleRange =  pi/180              #Angle resolution of the accumulator in radians
distanceRange = 1             #Distance resolution of the accumulator in pixels
lineThreshold =  100            #Accumulator threshold parameter. Only those lines are returned that get enough votes (>threshold)

fieldOfView = 72              #Horizontal Field of View

detectorObj = IMAGE_DOORS(kernalSize,thresholdingBlockSize,thresholdingConstant,
                          edgeThreshold1,edgeThreshold2,edgeApertureSize,
                          angleRange,distanceRange,lineThreshold,fieldOfView,1)
                          
def vis(img):
  figObj = figure()
  imgObj = figObj.add_subplot(111)
  imgObj.set_xticks([])
  imgObj.set_yticks([])
  imgObj.imshow(img,cmap="gray")
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
  
def main():  
  # Variable that determines whether we have detected a door yet, requiring
  # a certain number of successive door frame detections
  cam = cv2.VideoCapture(1)  
  while True:
    ret, img = cam.read()
    image = _cleanImage(img)
    detectorObj.drawImage(image)
    
if __name__ == '__main__':
  main()
