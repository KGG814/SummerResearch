import cv2
from threading import Thread
from time import sleep

class webcamImageGetter:
	def __init__(self):
		self.currentFrame = None
		self.CAMERA_WIDTH = 640
		self.CAMERA_HEIGHT = 480
		self.CAMERA_NUM = -1

		self.delayTime = 0 		# Delay time for continuos updation in seconds
		self.capture = cv2.VideoCapture(self.CAMERA_NUM) 
		#OpenCV by default gets a half resolution image so we manually set the correct resolution
		self.capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,self.CAMERA_WIDTH)
		self.capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,self.CAMERA_HEIGHT)

	#Starts updating the images in a thread
	def start(self):
		tCamera=Thread(target=self.updateFrame, args=())
		tCamera.daemon=True
		tCamera.start()

	#Continually updates the frame
	def updateFrame(self):
		while(True):
			sleep(self.delayTime)
			ret, self.currentFrame = self.capture.read()
			while (self.currentFrame == None): #Continually grab frames until we get a good one
				ret, frame = self.capture.read()

	def getFrame(self):
		return self.currentFrame

	def release(self):
		self.capture.release()