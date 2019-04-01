#!/usr/bin/env python
import click
import rospy
from geometry_msgs.msg import Twist 

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rospy.init_node('turtle')

while not rospy.is_shutdown():
	twist = Twist() #this expresses velocity in free space broken into its linear and angular parts.
	key_pressed = click.getchar()
	up = rospy.get_param("up")
	down = rospy.get_param("down")
	left = rospy.get_param("left")
	right = rospy.get_param("right")

	if key_pressed == up:
		twist.linear.x = 1.5
	elif key_pressed == left:
		twist.angular.z = 3.5
	elif key_pressed == down:
		twist.linear.x = -0.6
	elif key_pressed == right:
		twist.angular.z = -4.20

	pub.publish(twist)
