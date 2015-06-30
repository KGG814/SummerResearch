from matplotlib.pyplot import *
from matplotlib import patches
from Algorithms import *
from Robots import *
from Landmarks import *
from Log_Manager import *
from numpy import *
from scipy import arange

class VISUALIZER(object):

	def __init__(self,xRange,yRange):
		ion()
		self.xRange = xRange
		self.yRange = yRange
		self.figureObj = figure()#figsize=(2*(xRange[1]-xRange[0]),yRange[1]-yRange[0]))
		self.plotObj   = self.figureObj.add_subplot(121)
		self.xTiles = arange(xRange[0],xRange[1],0.4572)
		self.yTiles = arange(yRange[0],yRange[1],0.4572)
		# self.plotObj.set_xticks(self.xTiles, minor=True)
		# self.plotObj.set_yticks(self.yTiles, minor=True)
		# self.plotObj.grid(True,which='minor')

		self.scanObj = self.figureObj.add_subplot(122,polar=True)
		self.scanObj.set_rmax(3.)
		self.scanObj.grid(True)

		# self.imgObj = self.figureObj.add_subplot(223)
		# self.imgObj = self.set_xticks([])
		# self.imgObj = self.set_yticks([])
		# show()

		self.completeMapx=[]
		self.completeMapy=[]
		self.robotPathx=[]
		self.robotPathy=[]


	def plot_existing_data(self,pathName,marker):
		ExistingDataHolder = LOG_FILE()
		ExistingDataHolder.read(pathName)
	#Plotting existing paths
		for position in ExistingDataHolder.existingPositions:
			self.plotObj.plot( position[0], position[1], marker)
	#Plotting existing cylinders
		# self.plotObj.plot([0.9144],[1.3716],marker='o',ms=15,mfc='k')
		# self.plotObj.plot([0.9144],[2.7432],marker='o',ms=15,mfc='k')
		# self.plotObj.plot([0.9144],[4.572],marker='o',ms=15,mfc='k')
		# self.plotObj.plot([0.4572],[1.3716],marker='o',ms=15,mfc='k')
		# self.plotObj.plot([1.3716],[2.7432],marker='o',ms=15,mfc='k')
		# self.plotObj.plot([0.4572],[4.572],marker='o',ms=15,mfc='k')
	#Plotting existing walls
		# xWalls=0.4572*array([-1,-1,-1.5,-1.5,-1,-1,5,5,6,6,5,5,-1])
		# yWalls=0.4572*array([-2,8,8,1,1,15,15,12,12,8,8,-2,-2])
		# self.plotObj.plot(xWalls,yWalls,'-k')
	
	def _get_line_points(self,point):
		[x0,y0] = point
		theta=atan2(y0,x0)
		a = np.cos(theta)
		b = np.sin(theta)
		x1 = float(x0 + 1.5*(-b))
		y1 = float(y0 + 1.5*(a))
		x2 = float(x0 - 1.5*(-b))
		y2 = float(y0 - 1.5*(a))
		return [x1,x2],[y1,y2]

	def _get_error_ellipse(self,covariance):
		"""Return the position covariance (which is the upper 2x2 submatrix)
			as a triple: (main_axis_angle, stddev_1, stddev_2), where
			main_axis_angle is the angle (pointing direction) of the main axis,
			along which the standard deviation is stddev_1, and stddev_2 is the
			standard deviation along the other (orthogonal) axis."""
		eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
		angle = atan2(eigenvects[1,0], eigenvects[0,0])
		return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))   

	def plot_map(self,itemList):
		self.plotObj.clear()
		# xlim(self.xRange)
		# ylim(self.yRange)
		# self.plotObj.set_xticks(self.xTiles, minor=True)
		# self.plotObj.set_yticks(self.yTiles, minor=True)
		# self.plotObj.grid(True,which='minor')
		# self.plot_existing_data("./Data/run5/OverHeadPath.txt",'kx')
		self.plot_existing_data("./Data/run10_th/prediction.txt",'gx')

		for items in itemList:
			if items.__class__.__name__ == "ROBOT":
				paths = items.Path.get_path()
				self.plotObj.plot(paths[0],paths[1],'rx')
				self.plotObj.arrow(items.position[0],items.position[1],0.1*cos(items.position[2]),0.1*sin(items.position[2]),head_width=0.05,color='r')
				(angle,width,height) = self._get_error_ellipse(items.covariance)
				e1 = patches.Ellipse((items.position[0],items.position[1]), width, height,angle=angle, linewidth=2, fill=False)
				self.plotObj.add_patch(e1)

			if items.__class__.__name__=="CYLINDER":
				self.plotObj.scatter(items.position[0],items.position[1],marker='o',c='b',s=20)
				(angle,width,height) = self._get_error_ellipse(items.covariance)
				e1 = patches.Ellipse((items.position[0],items.position[1]), width, height,angle=angle, linewidth=2, fill=False)
				self.plotObj.add_patch(e1)

			if items.__class__.__name__=="WALL":
				self.plotObj.scatter(items.position[0],items.position[1],marker='o',c='b',s=20)
				xPoints,yPoints=self._get_line_points(items.position)
				self.plotObj.plot(xPoints,yPoints,'b')
				self.plotObj.plot([0,items.position[0]],[0,items.position[1]],'b--')
				(angle,width,height) = self._get_error_ellipse(items.covariance)
				e1 = patches.Ellipse((items.position[0],items.position[1]), width, height,angle=angle, linewidth=2, fill=False)
				self.plotObj.add_patch(e1)

		draw()
		# print "waiting"
		# raw_input()
		pause(0.000000001)


	def plot_association(self,new,existing):
		if existing.__class__.__name__=="CYLINDER":
			self.plotObj.scatter(new.position[0],new.position[1],marker='o',c='g',s=20)
			if existing:
				self.plotObj.scatter(existing.position[0],existing.position[1],marker='o',c='r',s=20)
				self.plotObj.plot([new.position[0],existing.position[0]],[new.position[1],existing.position[1]])

		if existing.__class__.__name__=="WALL":
			#print "wall"
			self.plotObj.scatter(new.position[0],new.position[1],marker='o',c='g',s=20)
			xPoints,yPoints=self._get_line_points(new.position)
			self.plotObj.plot(xPoints,yPoints,'g')
			self.plotObj.plot([0,existing.position[0]],[0,existing.position[1]],'g--')
			if existing:
				self.plotObj.scatter(existing.position[0],existing.position[1],marker='o',c='r',s=20)
				xPoints,yPoints=self._get_line_points(existing.position)
				self.plotObj.plot(xPoints,yPoints,'r')
				self.plotObj.plot([0,existing.position[0]],[0,existing.position[1]],'r--')
				self.plotObj.plot([new.position[0],existing.position[0]],[new.position[1],existing.position[1]],'g')

		if existing.__class__.__name__=="CORNER":
			#print "corner"
			self.plotObj.scatter(new.position[0],new.position[1],marker='o',c='g',s=20)
			if existing:
				self.plotObj.scatter(existing.position[0],existing.position[1],marker='o',c='r',s=20)
				self.plotObj.plot([new.position[0],existing.position[0]],[new.position[1],existing.position[1]])

			
		
		draw()
		# print "waiting"
		
		pause(0.000000001)

	def _cart_to_pole(self,xPoints,yPoints):
		rPoints=[]
		tPoints=[]
		for (x,y) in zip(xPoints,yPoints):
			tPoints.append(atan2(y,x))
			rPoints.append(hypot(x,y))
		return tPoints,rPoints


	def plot_scan(self,angles,distances,landmarks):
		self.scanObj.clear()
		self.scanObj.grid(True)
		self.scanObj.scatter(angles,distances,marker='x',c='g')
		# print "plotting landmarks:"
		# print len(landmarks)
		# for (position,certainty) in landmarks:
		# 	self.scanObj.scatter(position[0],position[1],marker='o',c='r',s=20)
		# 	xPoints,yPoints=self._get_line_points(position)
		# 	tPoints,rPoints=self._cart_to_pole(xPoints,yPoints)
		# 	self.scanObj.plot(tPoints,rPoints,'b')
		self.scanObj.set_rmax(3)
		# raw_input()
		draw()

	def plot_image(self,image,landmarks):
		self.imgObj.clear()
		print "showing image"
		print len(landmarks)




	def _scan_to_world(self,tPoints,rPoints,robotPosition):
		xPoints=[]
		yPoints=[]
		for (t,r) in zip(tPoints,rPoints):
			x, y = r*cos(t-pi/2), r*sin(t-pi/2)
			dx = cos(robotPosition[2])
			dy = sin(robotPosition[2])
			xPoints.append(x * dx - y * dy + robotPosition[0])
			yPoints.append(x * dy + y * dx + robotPosition[1])
		return xPoints,yPoints

	def add_to_complete_map(self,robotObj):
		angles,distances = get_data("Lidar")
		xList,yList=self._scan_to_world(angles,distances,robotObj.position)
		self.completeMapx = self.completeMapx+xList
		self.completeMapy = self.completeMapy+yList
		self.robotPathx = self.robotPathx+robotObj.position[0]
		self.robotPathy = self.robotPathx+robotObj.position[1]

	def plot_final(self, finalMap) :
		if finalMap.__class__.__name__=="CORNER" :
			self.finalplotObj.scatter(finalMap.position[0],finalMap.position[1],marker='o',c='g',s=20)

		if finalMap.__class__.__name__=="WALL":
			#print "wall"
			self.finalplotObj.scatter(finalMap.position[0],finalMap.position[1],marker='o',c='g',s=20)
			xPoints,yPoints=self._get_line_points(finalMap.position)
			self.finalplotObj.plot(xPoints,yPoints,'g')
			self.finalplotObj.plot([0,finalMap.position[0]],[0,finalMap.position[1]],'g--')


	def plot_complete_map(self, finalMap):
		self.finalFigureObj = figure()#figsize=(2*(xRange[1]-xRange[0]),yRange[1]-yRange[0]))
		self.finalplotObj   = self.finalFigureObj.add_subplot(111)
		# self.plotObj.set_xticks(self.xTiles, minor=True)
		# self.plotObj.set_yticks(self.yTiles, minor=True)
		# self.plotObj.grid(True,which='minor')
		for landmark in finalMap:
			self.plot_final(landmark)
		show()




