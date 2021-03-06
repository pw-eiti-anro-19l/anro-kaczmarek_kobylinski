#!/usr/bin/env python
 
import rospy
from lab2.srv import Jint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import os
import json
from sensor_msgs.msg import *
from tf.transformations import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
 

freq = 50
 
xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)

def handle_interpolation(value):
	if value.time <= 0 or not -100 <= value.j1 <= 100 or not -100 <= value.j2 <= 100 or not -100 <= value.j3 <= 100:
		return False
 
	end_pos = [value.j1, value.j2, value.j3]
 	start_pos = [ 0, 0, 0] 
 	pos_change = [0,0,0]
	step=[(end_pos[0]-start_pos[0])/(freq*value.time),(end_pos[1]-start_pos[1])/(freq*value.time),(end_pos[2]-start_pos[2])/(freq*value.time)]
	for k in range(0, int(freq*value.time)+1):
		for i in range(0, 3):
			pos_change[i]=pos_change[i]+step[i]
		start_pos=[pos_change[0],pos_change[1],pos_change[2]]
  
		robot_pose = PoseStamped()
		robot_pose.header.frame_id = "base_link"
        	robot_pose.header.stamp = rospy.Time.now()
        	robot_pose.pose.position.x = pos_change[0]
        	robot_pose.pose.position.y = pos_change[1]
        	robot_pose.pose.position.z = pos_change[2]
   
		rate = rospy.Rate(50) # 50hz
        	pub.publish(robot_pose)
        	rate.sleep()
 
	current_time = 0
	return (str(value.j1)+" "+str(value.j2)+" "+str(value.j3))

 
 
if __name__ == "__main__":
    rospy.init_node('int_srv')
    pub = rospy.Publisher('oint',PoseStamped, queue_size=10)
    s = rospy.Service('oint_control_srv', Jint, handle_interpolation)
    rospy.spin()