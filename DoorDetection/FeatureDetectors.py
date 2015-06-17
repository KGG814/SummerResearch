import numpy as np
from math import degrees, sin, cos, hypot, atan2, pi
import random
from scipy.optimize import curve_fit
import cv2
import os
from datetime import datetime

class DOOR(object):
  def __init__(self, left, right, top, bottom):
    self.left = left
    self.right = right
    self.top = top
    self.bottom = bottom
    
  # Extracts the corners from this door object
  def getCorners(self):
    # m = -cos(theta)/sin(theta)
    mTop = -cos(self.top[1])/sin(self.top[1])  
    mBottom = -cos(self.bottom[1])/sin(self.bottom[1]) 
    # b = rho/sin(theta)
    bTop = self.top[0]/sin(self.top[1])  
    bBottom = self.bottom[0]/sin(self.bottom[1])
    
    # Calculation of top left and bottom left corners of door
    if self.left[1] == 0.0:
      # Vertical line case
      topLeft_x = self.left[0]
      bottomLeft_x = self.left[0]
    else:
      # m = -cos(theta)/sin(theta)
      mLeft = -cos(self.left[1])/sin(self.left[1])
      # b = rho/sin(theta)
      bLeft = self.left[0]/sin(self.left[1])
      # x =  (b_2-b_1)/(m_2-m_1) 
      topLeft_x = abs((bTop-bLeft)/(mTop-mLeft))
      bottomLeft_x = abs((bBottom-bLeft)/(mBottom-mLeft))    
      
    # Calculation of rop right and bottom right corners of door
    if self.right[1] == 0.0:
      # Vertical line case
      topRight_x = self.right[0]
      bottomRight_x = self.right[0]     
    else: 
      # m = -cos(theta)/sin(theta)
      mRight = abs(-cos(self.right[1])/sin(self.right[1]))
      # b = rho/sin(theta)
      bRight = self.right[0]/sin(self.right[1])       
      # x =  (b_2-b_1)/(m_2-m_1) 
      topRight_x = abs((bTop-bRight)/(mTop-mRight))
      bottomRight_x = abs((bBottom-bRight)/(mBottom-mRight))
      
    # y = mx + b
    topLeft = int(topLeft_x), int(mTop*topLeft_x+bTop)
    topRight = int(topRight_x), int(mTop*topRight_x+bTop)
    bottomLeft = int(bottomLeft_x), int(mBottom*bottomLeft_x+bBottom)
    bottomRight = int(bottomRight_x), int(mBottom*bottomRight_x+bBottom)
    return topLeft, topRight, bottomLeft, bottomRight

class IMAGE_DOORS(object):
  """Class to find doors in on-board camera images""" 

  def __init__(self,kernalSize,thresholdingBlockSize,thresholdingConstant,
               edgeThreshold1,edgeThreshold2,edgeApertureSize,angleRange,
               distanceRange,lineThreshold,fieldOfView,angularError):
    self.doorDetection=0.0
    #parameters for median blur 
    self.kernalSize = kernalSize
    #Aperture linear size; it must be odd and greater than 1, 
    #for example: 3, 5, 7 ...

    #adaptiveThreshold parameters
    #Size of a pixel neighborhood that is used to calculate a threshold value
    #for the pixel: 3, 5, 7, and so on.
    self.thresholdingBlockSize = thresholdingBlockSize
    #Constant subtracted from the mean or weighted mean     
    self.thresholdingConstant = thresholdingConstant    

    #Canny edge detector parameters
    #First threshold for the hysteresis procedure
    self.edgeThreshold1 = edgeThreshold1
    #Second threshold for the hysteresis procedure
    self.edgeThreshold2 = edgeThreshold2      
    #Aperture size for the Sobel() operator    
    self.edgeApertureSize = edgeApertureSize        

    #HOUGH transform parameters
    #Angle resolution of the accumulator in radians
    self.angleRange = angleRange               
    #Distance resolution of the accumulator in pixels
    self.distanceRange = distanceRange             
    #Accumulator threshold parameter.
    #Only those lines are returned that get enough votes (>threshold)
    self.lineThreshold = lineThreshold            
    #Horizontal Field of View
    self.fieldOfView = fieldOfView  
    #EKF Error variable for Covariance            
    self.angularError = angularError            
    self.landmarkType = "Image Line"

  def get_doors(self,image):
    verticalLines = []
    horizontalLines = []
    #Convert image to gray
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    #deNoise image
    gray = cv2.medianBlur(gray,self.kernalSize)
    #Convert into a binary image
    th2 = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,
                                cv2.THRESH_BINARY,self.thresholdingBlockSize,
                                self.thresholdingConstant)
    #Find edges in image
    edges = cv2.Canny(th2,self.edgeThreshold1,self.edgeThreshold2,
                  apertureSize = self.edgeApertureSize)
                  
    #Find vertical lines in image
    lines = cv2.HoughLines (edges,self.distanceRange,
                        self.angleRange,100)
    if lines is not None:
      for line in lines:      
        for rho,theta in line:
          # Vertical lines at ~ +-28.6 degrees
          # 0 is vertical
          if theta < 0.5 or theta > pi-0.5:
            angle = (rho*self.fieldOfView/image.shape[1])-(self.fieldOfView/2)
            verticalLines.append((rho, theta))
            
    #Find horizontal lines in image
    lines = cv2.HoughLines (edges,self.distanceRange,
                        self.angleRange,120)  
    if lines is not None:
      for line in lines:      
        for rho,theta in line:
          # Horizontal lines between 88.9 and 90.9 degrees
          if 1.396 < theta < 1.745:
            angle = (rho*self.fieldOfView/image.shape[1])-(self.fieldOfView/2)
            horizontalLines.append((rho, theta))
    horizontalLines.sort(key = lambda x: x[0])
    horizontalLines = self.cleanLines(horizontalLines)
   
    doors = self.findDoors(verticalLines,horizontalLines)
    return doors, verticalLines + horizontalLines 

  # Removes multiple lines near each other
  # Somehow this works at the moment, but this should be something better
  # TODO needs to be tuned properly, or replaced with something else
  def cleanLines(self, lines):
    prevRho = 0.0
    cleanedLines = []
    for line in lines:
      rho = line[0]
      if rho - prevRho > 20 or prevRho == 0.0:
        prevRho = rho
        cleanedLines.append(line)
    return cleanedLines
    
  # Find the door in an image, given the lines
  def findDoors(self, verticalLines, horizontalLines):
    doors = []
    centre = 233.0
    leftFrame = (0.0,0.0)
    rightFrame = (centre*2,0.0)
    if horizontalLines:
      for line in verticalLines:
        rho = abs(line[0])
        theta = line[1]
        if leftFrame[0] < rho < centre:
          leftFrame = [line[0], theta]
        elif centre < rho < rightFrame[0]:
          rightFrame = [line[0], theta]
      if leftFrame != (0.0,0.0) and rightFrame != (centre*2,0.0):
          ret,top,bot = self.findDoorHorizontals(horizontalLines)
          if ret == True:
            doors.append(DOOR(leftFrame, rightFrame,top,bot))
    return doors
    
  # Find the horizontals of a door in the centre of the image
  # Requires a top frame, but does not require a bottom
  def findDoorHorizontals(self, horizontalLines):
    top = (0, pi/2) 
    for line in horizontalLines:
      if ((0 < line[0] < 158) and
          line[0] > top[0]):
        top = line
    bot = (480, pi/2) 
    for line in horizontalLines:
      if ((158 < line[0] < 316) and
          line[0] < bot[0]):
        bot = line
    if top == (0, pi/2):
      return False, top, bot
    elif bot[0]-top[0] > 100:
      return True, top, bot
    else:
      return False, top, bot
      
  # Show an image highlighting the doors
  def drawImage(self, image):
    doors, lines = self.get_doors(image)
    # Uncomment these two lines if you want to see all the candidate lines
    # in the image
    #for line in lines:
    #  image = self.drawLines(image, line[0], line[1])
    if self.doorDetection < 0:
      self.doorDetection += 1
    if doors:
      if self.doorDetection < 5:
        self.doorDetection += 1
      elif self.doorDetection >= 5:
        print str(datetime.time(datetime.now())), ": Door Detected"
        self.doorDetection = -20
      for door in doors:
        image=self.drawDoor(door, image)
    else:    
      if 0 <= self.doorDetection <= 2:
        self.doorDetection = 0
      else:
        self.doorDetection -= 0.5
    cv2.imshow("Image", image)
    key = cv2.waitKey(10)
   
  # Draw doors given door objects
  def drawDoor(self, door, image):
    topLeft,topRight, botLeft, botRight = door.getCorners()
    red = (0,0,255)
    green = (0,255,0)
    # Draw lines connecting the corners
    cv2.line(image,topLeft,topRight,red,4)
    cv2.line(image,topLeft,botLeft,red,4)
    cv2.line(image,topRight,botRight,red,4)
    cv2.line(image,botRight,botLeft,red,4)
    # Draw circles at the corners
    image = cv2.circle(image, topLeft, 4, green, thickness = -1)
    image = cv2.circle(image, botLeft, 4, green, thickness = -1)
    image = cv2.circle(image, topRight, 4, green, thickness = -1)
    image = cv2.circle(image, botRight, 4, green, thickness = -1)
    return image
  
  # Draws lines on an image given Hough coordinates
  def drawLines(self,image,rho,theta=0):
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
    
