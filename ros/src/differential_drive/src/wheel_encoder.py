#!/usr/bin/env python
import math
import rospy
from   std_msgs.msg  import Float64
from   beaglebone.io import GPIO

class WheelEncoder:
	initialized = False
	def __init__(self):
		rospy.init_node('wheel_encoder')
		self.nodeName = rospy.get_name()
		
		# ROS parameters
		self.rate     = rospy.get_param('~rate',10)
		tpr           = rospy.get_param('~ticks_per_rotation',0)
		radius        = float(rospy.get_param('~wheel_radius',0))
		self.pinNameA = rospy.get_param('~pinNameA','')
		self.pinNameB = rospy.get_param('~pinNameB','')
		if self.pinNameA=='' or self.pinNameB=='' or tpr==0 or radius==0:
			rospy.logfatal("Parameters pinNameA, pinNameB, wheel_radius and ticks_per_rotation are required")
			return
		self.m_per_tick = 2*math.pi*radius/tpr

		# BeagleBone pin setup
		self.cur_ticks  = 0
		self.last_ticks = 0
		GPIO.setup(self.pinNameA,GPIO.IN)
		GPIO.setup(self.pinNameB,GPIO.IN)
		GPIO.add_event_detect(self.pinNameA,GPIO.BOTH,callback=self._tickUpdate)
		GPIO.add_event_detect(self.pinNameB,GPIO.BOTH,callback=self._tickUpdate)
		# ROS publishers
		self.pub_vel = rospy.Publisher('wheel_vel',Float64,queue_size=1)
		self.initialized = True
		rospy.loginfo("Node %s started"%self.nodeName)
	
	def spin(self):
		if not self.initialized: return
		rosRate = rospy.Rate(self.rate)
		self.last_time = rospy.Time.now()
		while not rospy.is_shutdown():
			self.cur_time = rospy.Time.now()
			self.spinOnce()
			rosRate.sleep()
	
	def spinOnce(self):
		ticks_per_s =1.0*(self.cur_ticks-self.last_ticks)/(self.cur_time-self.last_time).to_sec()
		vel = ticks_per_s*self.m_per_tick 
		#rospy.loginfo("%s:vel %f"%(self.nodeName,vel))
		self.pub_vel.publish(vel)
		self.last_time  = self.cur_time
		self.last_ticks = self.cur_ticks

	def _tickUpdate(self,channel):
		self.cur_ticks += 1
		
if __name__ == '__main__':
    wheelEncder = WheelEncoder()
    wheelEncder.spin()
