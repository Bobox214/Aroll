#!/usr/bin/env python
import sys
import numpy
import rospy,tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist 
from std_msgs.msg      import Int32

class GoToGoal:
	initialized = False
	def __init__(self):
		rospy.init_node("go_to_goal",log_level=rospy.DEBUG)
		self.nodeName = rospy.get_name()

		self.twist = Twist()
		self.pose  = Pose()
		self.goal  = None

		# ROS Parameter
		self.Kp     = rospy.get_param('~Kp'   ,  1 )
		self.rate   = rospy.get_param("~rate" , 20 )
		self.v_max  = rospy.get_param("~v_max", 0.2)
		self.w_max  = rospy.get_param("~w_max", 1  )

		# ROS topics
		rospy.Subscriber("pose",Pose,self.poseUpdate)
		rospy.Subscriber("cmd_goal",Int32,self.cmdGoalUpdate)
		self.pub_twist = rospy.Publisher("cmd_vel",Twist,queue_size=1)

		self.initialized = True
		rospy.loginfo("%s started",self.nodeName)

	def spin(self):
		if not self.initialized: return

		rosRate = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			if self.goal is not None:
				self.twistUpdate()
				self.pub_twist.publish(self.twist)
			rosRate.sleep()
	
	def poseUpdate(self,msg):
		self.pose = msg

	def twistUpdate(self):
		if self.goal is None:
			w = 0
			v = 0
		else:
			errX = self.goal.position.x-self.pose.position.x
			errY = self.goal.position.y-self.pose.position.y
			if errX*errX<0.01 and errY*errY<0.01:
				self.goal = None
				w = 0
				v = 0
				errYaw = 0
			else:
				euler = tf.transformations.euler_from_quaternion([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w])
				goalYaw = numpy.arctan2(errY,errX)
				errYaw  = goalYaw-euler[2]
				errYaw  = numpy.arctan2(numpy.sin(errYaw),numpy.cos(errYaw))

				w = -self.Kp*errYaw
				if abs(w)>self.w_max:
					w = self.w_max*(abs(w)/w)
				if abs(w)<0.75:
					v = self.v_max / ( abs(w) + 1 )**0.5
				else:
					v = 0
			rospy.logdebug("%s pX:%f pY:%f eX:%f eY:%f eYaw:%f -> w:%f v:%f"
			,	self.nodeName,self.pose.position.x,self.pose.position.y,errX,errY,errYaw,w,v
			)

		self.twist.linear.x = v
		self.twist.angular.z = w
	
	def cmdGoalUpdate(self,msg):
		if self.goal is not None:
			rospy.loginfo("%s Cmd goal not used, already existing",self.nodeName)
			return
		self.goal = Pose()
		self.goal.position.x = float(msg.data%100)
		self.goal.position.y = float(msg.data/100)
		rospy.loginfo("%s Cmg goal set : x:%f y:%f",self.nodeName,self.goal.position.x,self.goal.position.y)


if __name__ == '__main__':
	goToGoal = GoToGoal()
	goToGoal.spin()
