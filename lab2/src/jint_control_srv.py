#!/usr/bin/env python

import rospy
from lab2.srv import Jint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math


freq = 50


def handle_interpolation(value):
    if value.time <= 0 or not -100 <= value.j1 <= 100 or not -100 <= value.j2 <= 100 or not -100 <= value.j3 <= 100:
        return False

    start_pos = [0, 0, 0]
    end_pos = [value.j1, value.j2, value.j3]
    pos_change = [0,0,0]
    step=[(end_pos[0]-start_pos[0])/(freq*value.time),(end_pos[1]-start_pos[1])/(freq*value.time),(end_pos[2]-start_pos[2])/(freq*value.time)]
    for k in range(0, int(freq*value.time)+1):
	for i in range(0, 3):
	    pos_change[i]=pos_change[i]+step[i]
	start_pos=[pos_change[0],pos_change[1],pos_change[2]]
	rate = rospy.Rate(50) # 50hz
        pose_str = JointState()
        pose_str.header.stamp = rospy.Time.now()
        pose_str.name = ['base_to_link1', 'join2_to_link2', 'link2_to_join3']
        pose_str.position = [pos_change[0], pos_change[1], pos_change[2]]
        pub.publish(pose_str)
        rate.sleep()


    current_time = 0
    return (str(value.j1)+" "+str(value.j2)+" "+str(value.j3))


if __name__ == "__main__":
    rospy.init_node('int_srv')
    pub = rospy.Publisher('joint_states',JointState,queue_size=10)
    s = rospy.Service('jint_control_srv', Jint, handle_interpolation)
    rospy.spin()