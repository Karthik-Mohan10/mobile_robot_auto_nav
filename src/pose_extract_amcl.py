#!/usr/bin/env  python

import rospy
import roslib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(data):

	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z

	x_1 = data.pose.pose.orientation.x
	y_1 = data.pose.pose.orientation.y
	z_1 = data.pose.pose.orientation.z
	w_1 = data.pose.pose.orientation.w

        f.write("%f %f\n" % (x, y))
	#f.write("%f %f %f %f %f %f\n" % (x, y, z, x_1, y_1, z_1))

	print "Data Written"

if __name__ == '__main__':

	f= open("$(find robot_auto_nav)\pose_trajectory_amcl.txt","w")


	rospy.init_node('pose_extract_amcl')
	pose_value=rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,callback)
	rospy.spin()
