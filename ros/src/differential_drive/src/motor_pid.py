#!/usr/bin/env python
import sys
import rospy
from   std_msgs.msg  import Float64
from   beaglebone.io import PWM, GPIO

class MotorPid:
	initialized = False
	def __init__(self,debug=False):
		self.debug = debug

		rospy.init_node("motor_pid")
		self.nodeName = rospy.get_name()

		# ROS Parameter
		self.Kp         = rospy.get_param('~Kp',100)
		self.Ki         = rospy.get_param('~Ki', 50)
		self.Kd         = rospy.get_param('~Kd',0.1)
		self.rate       = rospy.get_param('~rate',20)
		self.timeout    = rospy.Duration(rospy.get_param("~timeout", 0.2))
		self.pinNameBwd = rospy.get_param('~pinNameFwd','')
		self.pinNameFwd = rospy.get_param('~pinNameBwd','')
		self.pinNameCmd = rospy.get_param('~pinNameCmd','')
		self.minPwm     = rospy.get_param('~minPwm',40)
		if self.pinNameBwd=='' or self.pinNameFwd=='' or self.pinNameCmd=='':
			rospy.logfatal("Parameters pinNameFwd, pinNameBwd and pinNameCmd are required.")
			return
		self.timeoutTime = None

		# BeagleBone pin setup
		GPIO.setup(self.pinNameFwd,GPIO.OUT,initial=0)
		GPIO.setup(self.pinNameBwd,GPIO.OUT,initial=0)
		self.pwmCmd = PWM(self.pinNameCmd,frequency=200,dutyCycle=0,enable=1)

		# ROS publisher/subscribers
		rospy.Subscriber("wheel_vel",Float64,self.wheelVelUpdate)
		rospy.Subscriber("cmd_vel"  ,Float64,self.cmdVelUpdate)

		self.initialized = True
		rospy.loginfo("%s started%s"%(self.nodeName,' in debug mode' if self.debug else ''))

	def reset(self):
		self.cmd_vel     = 0
		self.wheel_vel   = 0
		self.last_error  = 0
		self.total_error = 0
		self.last_motor  = 0
		self.last_time   = rospy.Time.now()

	def spin(self):
		if not self.initialized: return
		self.reset()
		rosRate = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			if self.timeoutTime is not None:
				if rospy.Time.now() > self.timeoutTime:
					self.timeoutTime = None
					if self.debug:
						rospy.loginfo("%s : cmd_vel timeout"%self.nodeName)
					self.cmd_vel = 0
				self.spinOnce()
			rosRate.sleep()

	def spinOnce(self):
		if self.cmd_vel == 0:
			self.reset()
			self.stop()
			return

		# PID
		cur_time = rospy.Time.now()
		dt = (cur_time - self.last_time).to_sec()

		err  = self.cmd_vel - self.wheel_vel
		pErr = err
		dErr = (err-self.last_error)/dt
		iErr = self.total_error + err*dt

		motor = self.last_motor + self.Kp*pErr + self.Ki*iErr + self.Kd*dErr

		self.last_time   = cur_time
		self.last_error  = err

		# Flatenning
		if motor > 100:
			motor = 100
		elif motor < -100:
			motor = -100
		elif motor<0 and self.cmd_vel>0 or motor>0 and self.cmd_vel<0:
			motor = 0
		else:
			# Only update total error for valid motor commands
			self.total_error = iErr
		if self.debug:
			rospy.loginfo("%s : cmd_vel %f wheel_vel %f"%(self.nodeName,self.cmd_vel,self.wheel_vel))
			rospy.loginfo("%s : P %d I %d D %d -> %d "%(self.nodeName,self.Kp*pErr,self.Ki*iErr,self.Kd*dErr,motor))
		
		# Writing
		if motor == 0:
			self.stop()
		else:
			if motor*self.last_motor<=0:
				# Motor command has changed direction
				self.writeDirection(motor)
			pwm = abs(int(motor))
			if pwm < self.minPwm:
				pwm = self.minPwm
			self.pwmCmd.setDutyCycle(pwm)
		self.last_motor = motor
		
	def writeDirection(self,direction):
		if self.debug:
			rospy.loginfo("%s : going %s"%(self.nodeName,'Fwd' if direction>0 else 'Bwd'))
		if direction>0:
			GPIO.output(self.pinNameFwd,1)
			GPIO.output(self.pinNameBwd,0)
		else:
			GPIO.output(self.pinNameFwd,0)
			GPIO.output(self.pinNameBwd,1)
	
	def stop(self):
		GPIO.output(self.pinNameFwd,0)
		GPIO.output(self.pinNameBwd,0)
		self.pwmCmd.setDutyCycle(0)
	
	def wheelVelUpdate(self,msg):
		self.wheel_vel = msg.data

	def cmdVelUpdate(self,msg):
		self.cmd_vel = msg.data
		self.last_time   = rospy.Time.now()
		self.timeoutTime = rospy.Time.now()+self.timeout

if __name__ == '__main__':
	motorPid = MotorPid(debug="--debug" in sys.argv)
	motorPid.spin()
