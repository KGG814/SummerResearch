from numpy import *
from math import degrees, sin, cos, hypot, atan2, pi
import random
from scipy.optimize import curve_fit
from cv2 import *

"""All classes Has to contain a get_landmark class which returns landMarks 
Has to contain a landmarkType member containing the type of landmark"""

class RANSAC(object):
  """Class to perform RANSAC Based feature detection"""

  def __init__(self,N,D,S,C,X,avgWidth):
    self.numberOfTrials = N
    self.neighbourhood = D
    self.numberOfSamples = S 
    self.consensusRequired = C
    self.errorRange = X
    self.landmarkType = "Wall"
    self.avgWidth = avgWidth

  @staticmethod
  def _line(x,m,c):
    """Equation of a line used as a cost function for least square optimization"""
    return m*x+c

  @staticmethod
  def _cart_to_pole(Set,deg=True):
    """Cartesian to Polar conversion of a whole list"""
    PSet=[]
    for (x,y) in Set:
      r = hypot(x, y)
      t=atan2(y, x)
      # if t<0:
      #     t=2*pi+t
      if deg:
        PSet.append(tuple((180*t/pi,r)))
      else:
        PSet.append(tuple((t,r)))
    return PSet

  @staticmethod
  def _pole_to_cart(Set):
    """Polar to Cartesian conversion of a whole list"""
    xS=[]
    yS=[]
    for s in Set:
      angle = s[0]
      distance = s[1] 
      if distance !=0:
        xs, ys = distance*cos(angle), distance*sin(angle)
        xS.append(xs)
        yS.append(ys)
    return xS,yS

  def get_landmarks(self, angles, distances):
    
    wallList = [] #Initialize list of walls
    FullScan=[(angles[i],distances[i]) for i in xrange(len(angles))]
    #print len(FullScan)
    xData,yData=self._pole_to_cart(FullScan)

    #Generate Set of Unassociated scans
    UnAsc=[]
    for x, y in zip(xData,yData):
      UnAsc.append((x, y))

    Trials=0
    m=0
    c=0

    while UnAsc and (Trials<self.numberOfTrials):
      Trials=Trials+1
      #Select a random laser data reading.
      (seedAngle,seedDistance)=random.choice(FullScan)
      #Randomly sample data readings within neighborhood of this laser data reading
      shortList=[(a,d) for (a,d) in FullScan 
                  if seedAngle-self.neighbourhood < a < seedAngle+self.neighbourhood]

      if not len(shortList)>self.numberOfSamples:
        continue

      samples=random.sample(FullScan,self.numberOfSamples)

      #calculate a least squares best fit line  
      xSamples,ySamples=self._pole_to_cart(samples) 
      xSamples=array(xSamples)
      ySamples=array(ySamples)  
      m,c=curve_fit(self._line,xSamples,ySamples)[0]


      #Determine how many laser data readings 
      #lie within self.error_range meters of this best fit line       
      pointsOnLine=[]
      for u in UnAsc:
        x1=u[0]
        y1=u[1]
        d=abs(m*x1+c-y1)/sqrt((m**2)+1)
        if d<=self.errorRange:
          pointsOnLine.append(u)
      xLine=[]
      yLine=[]

       #If the number of laser data readings on the line
       #is above some consensus self.consensus
      if len(pointsOnLine)>=self.consensusRequired:
        #Calculate new least squares best fit line based on all the 
        # laser readings determined to lie on the old best fit line.
        for p in pointsOnLine:
          xLine.append(p[0])
          yLine.append(p[1])
          #Remove the number of readings lying on the line from the 
          # total set of unassociated readings.
          UnAsc.remove(p) 

        xLine=array(xLine)
        yLine=array(yLine)
        m,c=curve_fit(self._line,xLine,yLine)[0]
        #Add this best fit line to the lines we have extracted.
        yL=c/((m**2)+1)
        xL=-m*yL

        probablity = len(pointsOnLine)/(self.avgWidth)
        print "length of points"
        print len(pointsOnLine)
        wallList.append(([xL,yL],probablity))

      # print Trials
      # print len(UnAsc)
      # print len(pointsOnLine)
      # print len(wallList)
      # print "----------------"
    return wallList



class SPIKE(object):

  def __init__(self,jump, avgWidth):
    self.threshold = jump
    self.avgWidth  = avgWidth
    self.landmarkType = "Cylinder"

  def _computeDerivative(self,angles, distances):
    """Find the derivative in scan data, ignoring invalid measurements."""
    slope=[]
    slope.append(0)
    for i in xrange(1,len(angles)):
      der = (distances[i]-distances[i-1])/(angles[i]-angles[i-1])
      slope.append(der)
    #slope.append(0)
    return slope


  def get_landmarks(self, angles, distances):
    cylinder_list = []
    on_cylinder = False
    sum_ray, sum_depth, rays = 0.0, 0.0, 0
    scan_derivative=self._computeDerivative(angles,distances)
    for i in xrange(len(scan_derivative)):
      # --->>> Insert your cylinder code here.
      # Whenever you find a cylinder, add a tuple
      # (average_ray, average_depth) to the cylinder_list.
      if (scan_derivative[i]<0 and abs(scan_derivative[i])>self.threshold):
        on_cylinder=True
        sum_ray, sum_depth, rays = 0.0, 0.0, 0

      if on_cylinder == True:
        sum_depth=sum_depth+distances[i]
        rays=rays+1
        sum_ray=sum_ray+angles[i]

        if (scan_derivative[i]>0 and abs(scan_derivative[i])>self.threshold):
          print abs(scan_derivative[i])
          on_cylinder=False
          average_ray=sum_ray/rays
          average_depth=sum_depth/rays
          probablity = 1#rays/self.avgWidth
          cylinder_list.append(([average_ray,average_depth], probablity))
          sum_ray, sum_depth, rays = 0.0, 0.0, 0

    return cylinder_list,scan_derivative


class HOUGH(object):
  """Class to find walls in Lidar data using Hough Transform"""

  def __init__(self,maxRange,radialResolution,angularResolution,threshold):
    #Max range of Lidar to be used for defining blank image (cm). 
    self.maxRange = maxRange  
    #Resolution for line detection 
    self.radialResolution = radialResolution  
    self.angularResolution = angularResolution
    self.threshold = threshold
    #Transformation matrix from lidar frame to image frame
    self.trans_matrix=array([[0,-1,maxRange],[1,0,maxRange],[0,0,1]])
    #Transformation matrix from image frame to lidar frame
    self.inv_trans_matrix=linalg.inv(self.trans_matrix)
    self.landmarkType = "Wall"

  def _pole_to_cart(self,angles,distances):
    """ Converts from polar coordinates to cartesian coordinates"""
    cart=[]
    for i in xrange(0,len(angles)-1):
      angle = angles[i]
      distance = distances[i] 
      xs, ys = distance*cos(angle), distance*sin(angle)
      cart.append(tuple((xs,ys)))
    return cart


  def get_landmarks(self,angles,distances):

    landMarks=[]
    #Convert distances to centimeters
    distances=[d*100 for d in distances]

    #Convert from polar t cartesian coordinates
    points=self._pole_to_cart(angles,distances)
  
    # Create new blank 1000x1000 black image
    image = zeros((2*self.maxRange, 2*self.maxRange, 3), uint8)
    # Place White dots for each Lidar reading
    for (xo,yo) in points:
      lid_pnt=array([xo,yo,1])
      img_pnt=floor(dot(self.trans_matrix,lid_pnt))
      image[img_pnt[0],img_pnt[1]]=(255,255,255)


    #Use hough transform to detect lines.
    #lines = HoughLines(image[:,:,0],self.radialResolution,self.angularResolution,self.threshold)
    lines = HoughLines(image[:,:,0],2,pi/18,50)
    #If there are lines transform back to lidar frame.
    try:
      # print "No of lines:"
      # print len(lines[0])
      for rho,theta in lines[0]:
        a = cos(theta)
        b = sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = (x0 + 100*(-b))
        y1 = (y0 + 100*(a))
        x2 = (x0 - 100*(-b))
        y2 = (y0 - 100*(a))
        img_pnt0=array([y1,x1,1])
        img_pnt1=array([y2,x2,1])
        lid_pnt0=dot(self.inv_trans_matrix,img_pnt0)
        lid_pnt1=dot(self.inv_trans_matrix,img_pnt1)
        dx=lid_pnt1[0]-lid_pnt0[0]
        dy=lid_pnt1[1]-lid_pnt0[1]
        K=((dx*lid_pnt0[1])-(dy*lid_pnt0[0]))/(hypot(dx,dy)**2)
        xL=-K*dy/100
        yL=K*dx/100
        landMarks.append(([xL,yL],1))

    except:
      print "no lines"
      pass

    return landMarks


class DERIVATIVES(object):
  """Class to find cylinders using first derivative combined with pre knowledge of the cylinders"""

  def __init__(self,derivativeThreshold, cylinderRadius, shortRangeAccuracy, longRangeAccuracy, unobservableLandmarkArea, lineFittingThreshold, minSamples):
    self.derivativeThreshold = derivativeThreshold
    #Radius of cylinders (unit = meter)
    self.cylinderRadius = cylinderRadius           
    #Accuracy of lidar sensor  for distance less than 1 meter (Unit = meter)
    self.shortRangeAccuracy = shortRangeAccuracy  
    #Accuracy of lidar sensor  for distance larger than 1 meter (Unit = percentage)     
    self.longRangeAccuracy = longRangeAccuracy     
    #The percentage of the landmark's area that we suppose the lidar misses    
    self.unobservableLandmarkArea = unobservableLandmarkArea 
    #Threshold for line fitting error
    self.lineFittingThreshold = lineFittingThreshold     
     #Threshold for minimum number of samples
    self.minSamples = minSamples               
    self.landmarkType = "Cylinder"        

  @staticmethod
  def _line(x,m,c):
    """Equation of a line used as a cost function for least square optimization"""
    return m*x+c

  @staticmethod
  def _pole_to_cart(Set):
    """Polar to Cartesian conversion of a whole list"""
    xS=[]
    yS=[]
    for s in Set:
      angle = s[0]
      distance = s[1] 
      if distance !=0:
        xs, ys = distance*cos(angle), distance*sin(angle)
        xS.append(xs)
        yS.append(ys)
    return xS,yS

  def get_landmarks(self,angles,distances):
    #Initialize empty list of landmarks
    landmarks = []

    #Initially no thresholds are found
    previousThresholdFound = False

    startingIndex = 1
    #Look at each pair of angles and distances
    for i in xrange(1,len(angles)):

      #Check if there is a large jump in distance
      if abs(distances[i]-distances[i-1])>self.derivativeThreshold :

        #Store starting index
        stoppingIndex = i-1

        #Check if we are already on a probable landMark 
        if previousThresholdFound:
          # Then this is at the end of a section of scan
          
          #Find the parameters of the probable landmark. 
          #[startingIndex from previous threshold]
          beamIndex = int((startingIndex+stoppingIndex)/2)
          beamDistance = distances[beamIndex]+self.cylinderRadius
          beamAngle = angles[beamIndex]

          #Convert parameters to cartesian space. 
          beamX = beamDistance*cos(beamAngle)
          beamY = beamDistance*sin(beamAngle)

          #Find the angular width of the probable landmark
          angularWidth = abs(angles[stoppingIndex]-angles[startingIndex])*180/pi

          #Calculate the theoretical angular width for known landmarks
          expectedWidth = 2*atan2(self.cylinderRadius,distances[startingIndex])*180/pi

          # Find acceptable levels of deviation from the angular width
          # Check if it is near range
          if distances[startingIndex] > 1:
            #Find the maximum error possible by taking the tangential beam 
            distanceError = distances[startingIndex] - self.cylinderRadius - self.shortRangeAccuracy
            #Calculate acceptable deviation
            angleDeviation = abs(self.unobservableLandmarkArea*2*atan2(self.cylinderRadius,distanceError)*180/pi)
          else:
            #Find the maximum error possible by taking the tangential beam 
            distanceError = distances[startingIndex] - self.cylinderRadius - self.longRangeAccuracy*distances[startingIndex]
            #Calculate acceptable deviation
            angleDeviation = abs(self.unobservableLandmarkArea*2*atan2(self.cylinderRadius,distanceError)*180/pi)


          # Compute least square fit on the model of a straight line. 
          samples=[(angles[i],distances[i]) for i in xrange(startingIndex,stoppingIndex)]
          xSamples,ySamples=self._pole_to_cart(samples) 
          xSamples=array(xSamples)
          ySamples=array(ySamples)  
          parameters,covariance=curve_fit(self._line,xSamples,ySamples)
          fittingError = norm(diag(covariance))

          #Check for different condition:
          #    1) Check if there are enough points 
          #    2) Check if it's angular width is as expected
          #    3) Check if it is not a line, i.e. line fitting threshold is large
          
          if abs(expectedWidth-angularWidth) < angleDeviation \
              and len(samples) > self.minSamples \
              and fittingError > self.lineFittingThreshold:

            landmarks.append([beamAngle,beamDistance],1)

        else: 
          previousThresholdFound = True
          startingIndex = i 

    return landmarks
    
class ABSTRACT_DETECTOR(object):
  #TODO Add all the variables we will need for get_landmarks
  def __init__(self):
    self.landmarkType = "Abstract"
  #TODO Check if the current LIDAR data is enough to constitute a landmark.
  # If it is, return an abstract landmark
  def get_landmarks(self,angles,distances):
          
class IMAGE_LINES(object):
  """Class to find vertical lines in on-board camera images""" 

  def __init__(self,kernalSize,thresholdingBlockSize,thresholdingConstant,
               edgeThreshold1,edgeThreshold2,edgeApertureSize,angleRange,
               distanceRange,lineThreshold,fieldOfView,angularError):
    
    #parameters for median blur 
    self.kernalSize = kernalSize
    #Aperture linear size; it must be odd and greater than 1, 
    #for example: 3, 5, 7 ...

    #adaptiveThreshold parameters
    #Size of a pixel neighborhood that is used to calculate a threshold value for the pixel: 3, 5, 7, and so on.
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

  def get_landmarks(self,image):
    """Hough transform based lines extractor"""
    landmarks=[]
    #Convert image to gray
    gray = cvtColor(image,COLOR_BGR2GRAY)
    #deNoise image
    gray = medianBlur(gray,self.kernalSize)
    #Convert into a binary image
    th2 = adaptiveThreshold(gray,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,
                            self.thresholdingBlockSize,self.thresholdingConstant)
    #Find edges in image
    edges = Canny(th2,self.edgeThreshold1,self.edgeThreshold2,
                  apertureSize = self.edgeApertureSize)
    #Find lines in image
    lines = HoughLines (edges,self.distanceRange,
                        self.angleRange,self.lineThreshold)
    if lines is not None:
      for line in lines:      
        for rho,theta in line:
          if theta < 0.5 or theta > math.pi-0.5: #~20 degrees
            angle = (rho*self.fieldOfView/image.shape[1])-(self.fieldOfView/2)
            #landmarks.append((radian(angles),self.angularError))
            landmarks.append((rho, theta))
            pass
    return landmarks
