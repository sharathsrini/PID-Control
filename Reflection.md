## PID Control
I would like to begin with my love towards Control systems. The course content were excellent that help me revise what I learnt in college.I would like to explain each and every concept that i learnt in detail. The object of this project was to learn to implement a PID controller i.e Propotional-Integral-Derative control theory, on a car to control the steering and the spped value. The application of the controller was done in C++ as it is very robust on on-board processors rather than python or any other language.  


### Learning From the Project
If the controller we use keeps adding the error times a constant to the current output value is an Integrator. We are clearly integrating the error. A proportional controller would be setting the output to P times the error. It also matters what the output controls, e.g. whether it's torque, or position, or velocity for a motor control system. (something proportional in velocity is integral in torque)

The reason for a steady state error with P only is that as our system approaches the set-point the error signal gets smaller and smaller. Our control is Kp times that error signal and eventually the error will be small enough that Kp times the error won't be enough to force it all the way to zero.

An Integrator "saves the day" by accumulating the error over time and therefore even the tiniest error will eventually accumulate to something large enough to force the controller to correct for it. This could be seen in the simulator that the car oscillates on its path, this is because of the windup error . The integral sum starts accumulating when the controller is first put in automatic and continues to change as long as controller error exists.Once we cross over to a “no physical meaning” computation, the controller has lost the ability to regulate the process.

Here is a situation that neither proportional nor integral control fails. It is the  rapid changes to the system that comes from an external source. Keeping the system steady when outside influences are making it change abruptly is the job of derivative control. In calculus, derivative is an operation that measures the rate of change of a curve, such as the error curve . By taking action based on the rate the error is changing, the output drive can respond rapidly to disturbances to the system.Although ∆t might be a small or a large value, so long as it's always the same interval, the derivative constant, Kd can be scaled to compensate for the fact that we made ∆t equal to 1. The rate that the errors are sampled also has to be faster than the error signal's tendency to change.

### A Python Class of PID Controller:
```
class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.Derivator=Derivator
		self.Integrator=Integrator
		self.Integrator_max=Integrator_max
		self.Integrator_min=Integrator_min

		self.set_point=0.0
		self.error=0.0

	def feedback(self,current_value):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = self.set_point - current_value

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error

		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max
		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min

		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value

		return PID

	def set_Point(self,set_point):
		"""
		Initilize the setpoint of PID
		"""
		self.set_point = set_point
		self.Integrator=0
		self.Derivator=0

	def set_Integrator(self, Integrator):
		self.Integrator = Integrator

	def set_Derivator(self, Derivator):
		self.Derivator = Derivator

	def set_Kp(self,P):
		self.Kp=P

	def set_Ki(self,I):
		self.Ki=I

	def set_Kd(self,D):
		self.Kd=D

	def get_Point(self):
		return self.set_point

	def get_Error(self):
		return self.error

	def get-Integrator(self):
		return self.Integrator

	def get-Derivator(self):
		return self.Derivator
```
