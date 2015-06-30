from matplotlib import pyplot as plt
import time
from numpy import *


class VISUALIZATION(object):
	"""Class for methods of visualizing SLAM process"""

	def __init__(self,length,width):

		plt.ion()
		plt.figure(figsize=(width, length))
		self.ax = plt.axes(xlim=(-1,width),ylim=(-1,length))
		self.plotObj, = plt.plot([],[])
		self.scatterObj = plt.scatter([],[],c='k',s=20)
		plt.show()


	def updatePosition(self,robot):
		"""Update the position of the robot"""
		self.plotObj.set_xdata(append(self.plotObj.get_xdata(), robot.position[0]))
		self.plotObj.set_ydata(append(self.plotObj.get_ydata(), robot.position[1]))
		plt.draw()

	def updateLandmark(self,landmark):
		"""Update the position of the cylinders"""

		self.scatterObj.se_offset(append(self.scatterObj.get_xdata(), landmark.position[0]))
		self.scatterObj.set_ydata(append(self.scatterObj.get_ydata(), landmark.position[1]))
		plt.draw()

		