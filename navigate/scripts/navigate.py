#!/usr/bin/env python
import click
import rospy
from geometry_msgs.msg import Twist 

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rospy.init_node('turtle')

while not rospy.is_shutdown():
	twist = Twist()
	key_pressed = click.getchar()
	up = rospy.get_param("up")
	down = rospy.get_param("down")
	left = rospy.get_param("left")
	right = rospy.get_param("right")

	if key_pressed == up:
		twist.linear.x = rospy.get_param("up_value")
	elif key_pressed == left:
		twist.angular.z = rospy.get_param("left_value")
	elif key_pressed == down:
		twist.linear.x = rospy.get_param("down_value")
	elif key_pressed == right:
		twist.angular.z = rospy.get_param("right_value")

	pub.publish(twist)
