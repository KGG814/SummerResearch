from numpy import *
import Log_Manager

class ENCODER(object):
	"""Class containing functions related to a pair of Encoders
		Contains motion model for such a model as well as jacobians for the same
		Has to access encoder data and should contain functions that can handle it.
		Must contain an Update method to apply encoder data to any coordinate given"""

	def __init__(self,wheelDia,wheelSeperation,motionFactor,turnFactor):
		self.wheelDia = wheelDia				#Diameter of Wheels (m)
		self.wheelSeperation = wheelSeperation	#Width of the robot (m)
		self.motionFactor = motionFactor 		#Error due to encoder reading  (m /m)
		self.turnFactor = turnFactor			#Additional Error due to wheel slippage during turns  (m /m)

	def update(self,initialPosition,initialCovariance):
		"""Update Method for the robot. Get's latest Encoder data and uses to calculate new position and error
			position' = g(oldPosition)
			covariance' = G * covariance * GT + R
			where R = V * (covariance in control space) * VT
				  G = dg/dPosition
				  V = dg/dControl
				Covariance in control space depends on move distance.
			Returns new Position and Covariance """

		control = Log_Manager.get_data("Encoder")
		right, left = self._convert_to_m(control)

		newState = self._g(initialPosition,right,left)

		G = self._dg_dstate(initialPosition,right,left)
		V = self._dg_dcontrol(initialPosition,right,left)
		left_var  = (self.motionFactor * left)**2+ (self.turnFactor * (left-right))**2
		right_var = (self.motionFactor * right)**2+(self.turnFactor * (left-right))**2
		control_covariance = diag([left_var, right_var])
		R = dot(V, dot(control_covariance, V.T))

		newCovariance = dot(G, dot(initialCovariance, G.T)) + R

		return newState,newCovariance

	def _g(self,state,r,l):
		x, y, theta = state
		w = self.wheelSeperation
		if (r != l):
			alpha = (r - l) / w
			rad = l/alpha
			g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
			g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
			g3 = (theta + alpha + pi) % (2*pi) - pi
		else:
			g1 = x + l * cos(theta)
			g2 = y + l * sin(theta)
			g3 = theta

		return array([g1, g2, g3])

	def _dg_dstate(self,state,r,l):
		theta = state[2]
		w = self.wheelSeperation
		if (r != l):
			alpha = (r-l)/w
			theta_ = theta + alpha
			rpw2 = l/alpha + w/2.0
			m = array([[1.0, 0.0, rpw2*(cos(theta_) - cos(theta))],
						[0.0, 1.0, rpw2*(sin(theta_) - sin(theta))],
						[0.0, 0.0, 1.0]])
		else:
			m = array([[1.0, 0.0, -l*sin(theta)],
						[0.0, 1.0,  l*cos(theta)],
						[0.0, 0.0,  1.0]])
		return m

	def _dg_dcontrol(self,state,r,l):
		theta = state[2]
		w = self.wheelSeperation
		if r != l:
			rml = r - l
			rml2 = rml * rml
			theta_ = theta + rml/w
			dg1dl = w*r/rml2*(sin(theta_)-sin(theta))  - (r+l)/(2*rml)*cos(theta_)
			dg2dl = w*r/rml2*(-cos(theta_)+cos(theta)) - (r+l)/(2*rml)*sin(theta_)
			dg1dr = (-w*l)/rml2*(sin(theta_)-sin(theta)) + (r+l)/(2*rml)*cos(theta_)
			dg2dr = (-w*l)/rml2*(-cos(theta_)+cos(theta)) + (r+l)/(2*rml)*sin(theta_)

		else:
			dg1dl = 0.5*(cos(theta) + l/w*sin(theta))
			dg2dl = 0.5*(sin(theta) - l/w*cos(theta))
			dg1dr = 0.5*(-l/w*sin(theta) + cos(theta))
			dg2dr = 0.5*(l/w*cos(theta) + sin(theta))

		dg3dl = -1.0/w
		dg3dr = 1.0/w
		m = array([[dg1dl, dg1dr], [dg2dl, dg2dr], [dg3dl, dg3dr]])

		return m

	def _convert_to_m(self,control):
		r,l=control
		scaling=(self.wheelDia*pi)/360
		return r*scaling, l*scaling