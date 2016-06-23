#!/usr/bin/env python
import sys,math,numpy
import rospy,tf
from   std_msgs.msg       import Float64
from   geometry_msgs.msg  import Pose

class BaseLocalizer:
	initialized = False
	def __init__(self,debug=False):
		self.debug = debug

		rospy.init_node("base_localizer")
		self.nodeName = rospy.get_name()

		# ROS Parameter
		self.rate       = rospy.get_param("~rate", 20)
		self.base_width = rospy.get_param("~base_width", 0)
		if self.base_width==0:
			rospy.logfatal("Parameter base_width is required")
			return

		# ROS publisher/subscribers
		rospy.Subscriber("left_wheel_dst" ,Float64,self.leftWheelDstUpdate)
		rospy.Subscriber("right_wheel_dst",Float64,self.rightWheelDstUpdate)
		self.pub_pose = rospy.Publisher("pose",Pose,queue_size=1)

		self.reset()
		self.initialized = True
		rospy.loginfo("%s started%s"%(self.nodeName,' in debug mode' if self.debug else ''))

	def reset(self):
		self.dl        = 0
		self.dr        = 0
		self.pose      = Pose()
		self.yaw       = 0
		self.last_time = rospy.Time.now()

	def setPose(self,pose):
		euler = tf.transformations.euler_from_quaternion(pose.orientation)
		self.dl        = 0
		self.dr        = 0
		self.pose      = pose
		self.yaw       = euler[2]
		self.last_time = rospy.Time.now()
		
	def spin(self):
		if not self.initialized: return
		self.reset()
		rosRate = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.spinOnce()
			rosRate.sleep()
	
	def spinOnce(self):
		
		cur_time = rospy.Time.now()
		dt = (cur_time - self.last_time).to_sec()

		if self.dr!=0 or self.dl!= 0:
			df = (self.dr+self.dl)/2

			dx = df*math.cos(self.yaw)
			dy = df*math.sin(self.yaw)
			dyaw = numpy.arctan2(self.dr-self.dl,self.base_width)

			if self.debug:
				rospy.loginfo("%s : dr:%f dl:%f -> dx:%f dy:%f dyaw:%f"%(self.nodeName,self.dr,self.dl,dx,dy,dyaw))
				
			self.pose.position.x += dx
			self.pose.position.y += dy
			self.yaw += dyaw

			quaternion = tf.transformations.quaternion_from_euler(0,0,self.yaw)
			self.pose.orientation.x = quaternion[0]
			self.pose.orientation.y = quaternion[1]
			self.pose.orientation.z = quaternion[2]
			self.pose.orientation.w = quaternion[3]
			
			self.dl = 0
			self.dr = 0
			self.pub_pose.publish(self.pose)

	def leftWheelDstUpdate(self,msg):
		self.dl += msg.data

	def rightWheelDstUpdate(self,msg):
		self.dr += msg.data

if __name__ == '__main__':
	baseLocalizer = BaseLocalizer(debug="--debug" in sys.argv)
	baseLocalizer.spin()
