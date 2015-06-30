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
def get_lines(angles,distances):
	
	distances=[d*100 for d in distances]
	points=_pole_to_cart(angles,distances)

	# Create new blank 1000x1000 white image
	image = zeros((height, width, 3), uint8)

	for (xo,yo) in points:
		#ax2.plot(xo/100,yo/100,'rx')
		lid_pnt=array([xo,yo,1])
		img_pnt=floor(dot(trans_matrix,lid_pnt))
		image[img_pnt[0],img_pnt[1]]=(255,255,255)

	image[800,100]=(0,0,255)
	circle(image,(800,100),10,(0,0,255))
	lines = HoughLines(image[:,:,0],2,pi/18,50)

	return image,lines

width, height = 1000, 1000
trans_matrix=array([[0,-1,500],[1,0,500],[0,0,1]])
inv_trans_matrix=linalg.inv(trans_matrix)# array([[0,1,500],[-1,0,500],[0,0,1]])

plt.ion()
fig1=plt.figure(1)
ax1 = fig1.add_subplot(121,polar=True)
#ax2=fig1.add_subplot(132)


if __name__ == '__main__':
	data=LOG_FILE()
	data.read("./Data/run10_th/SlamDataNew.txt")
	for t in data.timeStamps[0:270]:
	#t=data.timeStamps[50]
		ax1.clear()
		angles,distances = data.scanAngles[t],data.scanDistances[t]
		for i in xrange(len(angles)):
			if not 0.2 < distances[i] < 5:
				angles[i] = -1
				distances[i] = -1
		angles    = filter(lambda a: a != -1, angles)
		distances = filter(lambda d: d != -1, distances)
		ax1.scatter(angles,distances,marker='x',c='g')


		image,lines, = get_lines(angles,distances)
		try:
			print "timestamp: ",data.timeStamps.index(t)
			print "number of lines: ", len(lines[0])
			for rho,theta in lines[0]:
				# print "R,alpha:"
				# print rho,theta
				a = cos(theta)
				b = sin(theta)
				x0 = a*rho
				y0 = b*rho
				x1 = int(x0 - 1000*(-b))
				y1 = int(y0 - 1000*(a))
				x2 = int(x0 + 1000*(-b))
				y2 = int(y0 + 1000*(a))
				line(image,(x1,y1),(x2,y2),(255,0,0),2)


				img_pnt0=array([y1,x1,1])
				img_pnt1=array([y2,x2,1])
				#ax2.plot([img_pnt0[0],img_pnt1[0]],[img_pnt0[1],img_pnt1[1]],'bo')
				print img_pnt0,img_pnt1
				lid_pnt0=dot(inv_trans_matrix,img_pnt0)/100
				lid_pnt1=dot(inv_trans_matrix,img_pnt1)/100
				print lid_pnt0,lid_pnt1

				#ax2.plot([lid_pnt0[0],lid_pnt1[0]],[lid_pnt0[1],lid_pnt1[1]],'go')
				
				t,r=_cart_to_pole([lid_pnt0[0],lid_pnt1[0]],[lid_pnt0[1],lid_pnt1[1]])
				ax1.scatter(t,r,marker='o',c='r',s=50)


				dx=lid_pnt1[0]-lid_pnt0[0]
				dy=lid_pnt1[1]-lid_pnt0[1]
				K=((dx*lid_pnt0[1])-(dy*lid_pnt0[0]))/(hypot(dx,dy)**2)
				xL=-K*dy
				yL=K*dx


				t1,r1 = _cart_to_pole([xL],[yL])
				ax1.scatter(t1,r1,marker='o',c='b',s=50)

		except:
			print "no lines"
			pass
		
		#plt.title('Cleaned Image'), plt.xticks([]), plt.yticks([])
		plt.subplot(122),plt.imshow(image)
		plt.title('Image with Lines'), plt.xticks([]), plt.yticks([])
		plt.draw()
		# raw_input()
		plt.pause(0.00001)
		plt.cla()

	raw_input()

