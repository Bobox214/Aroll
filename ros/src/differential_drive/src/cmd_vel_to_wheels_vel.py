#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist 

class TwistToMotors:
	initialized = False
	def __init__(self):

		rospy.init_node("twist_to_motors")
		self.nodeName = rospy.get_name()
		
		# ROS Parameter
		self.rate       = rospy.get_param("~rate", 50)
		self.timeout    = rospy.Duration(rospy.get_param("~timeout", 0.2))
		self.base_width = rospy.get_param("~base_width", 0)
		if self.base_width==0:
			rospy.logfatal("Parameter base_width is required")
			return
		self.timeoutTime = None
		
		# ROS publisher/subscribers
		self.pub_lmotor = rospy.Publisher('left_wheel_cmd_vel' , Float64,queue_size=1)
		self.pub_rmotor = rospy.Publisher('right_wheel_cmd_vel', Float64,queue_size=1)
		rospy.Subscriber('cmd_vel', Twist, self.cmdVelUpdate)
		
		self.initialized = True
		rospy.loginfo("%s started" % self.nodeName)
		
	def spin(self):
		if not self.initialized: return
		rosRate = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			if self.timeoutTime is not None:
				if rospy.Time.now() > self.timeoutTime:
					self.timeoutTime = None
					self.dx = 0
					self.dr = 0
				self.spinOnce()
			rosRate.sleep()
		
	def spinOnce(self):
		# dx = (l + r) / 2
		# dr = (r - l) / w
		
		right = 1.0 * self.dx + self.dr * self.base_width / 2 
		left  = 1.0 * self.dx - self.dr * self.base_width / 2
				
		rospy.logdebug("%s : pub l:%f r:%f",self.nodeName,left,right)
		
		self.pub_lmotor.publish(left)
		self.pub_rmotor.publish(right)
			

	def cmdVelUpdate(self,msg):
		self.timeoutTime = rospy.Time.now()+self.timeout
		self.dx = msg.linear.x
		self.dr = msg.angular.z
	
if __name__ == '__main__':
	twistToMotors = TwistToMotors()
	twistToMotors.spin()
