#!/usr/bin/env  python

import rospy
import roslib
from nav_msgs.msg import Odometry

def callback(data):

	x_o = data.pose.pose.position.x
	y_o = data.pose.pose.position.y
	z_o = data.pose.pose.position.z

	x_1 = data.pose.pose.orientation.x
	y_1 = data.pose.pose.orientation.y
	z_1 = data.pose.pose.orientation.z
	w_1 = data.pose.pose.orientation.w

	x_li = data.twist.twist.linear.x
	y_li = data.twist.twist.linear.x
	z_li = data.twist.twist.linear.x

	x_an = data.twist.twist.angular.x
	y_an = data.twist.twist.angular.y
	z_an = data.twist.twist.angular.z

        

        f.write("%f %f\n" % (x_o, y_o))
	#f.write("%f %f %f %f %f %f %f %f %f %f %f %f %f\n" % (x_o, y_o, z_o, x_1, y_1, z_1, w_1, x_li, y_li, z_li, x_an, y_an, z_an))

	print "Data Writing completed"


if __name__ == '__main__':

	f= open("pose_odom.txt","w+")

	rospy.init_node('pose_extract_odom')
	odom_value=rospy.Subscriber("/odom",Odometry,callback)
	rospy.spin()
