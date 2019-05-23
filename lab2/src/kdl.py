#! /usr/bin/python
import rospy
import json
import PyKDL as kdl
import os
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import *
from visualization_msgs.msg import Marker

kdlChain = kdl.Chain()   
frame = kdl.Frame()

with open(os.path.dirname(os.path.realpath(__file__)) + '/../dh.json', 'r') as file:
    params = json.loads(file.read())


def callback(data):
    kdlChain = kdl.Chain()   
    frame = kdl.Frame()

    with open(os.path.dirname(os.path.realpath(__file__)) + '/../dh.json', 'r') as file:
        params = json.loads(file.read())
	matrices = {}
        for key in sorted(params.keys()):
	    a, d, al, th = params[key]
	    if key == '1':
		al, a, d, th = float(al), float(a), float(d), float(th)
		kdlChain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.TransZ), frame.DH(a, al, d, th-1.57)))
		#kdlChain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.TransZ), frame.DH(0, 0, 2, 1.57-1.57)))
	    if key == '2':
		al, a, d, th = float(al), float(a), float(d), float(th)
		kdlChain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), frame.DH(a, al, d, th)))
		#kdlChain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), frame.DH(4, 0, 0, 1.57)))
	    if key == '3':
		al, a, d, th = float(al), float(a), float(d), float(th)
		kdlChain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), frame.DH(a, al, d, th)))
		#kdlChain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), frame.DH(2, 0,0, 1.57)))
            

    #kdlChain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), frame.DH(a, al, d, th)))
    jointPos = kdl.JntArray(kdlChain.getNrOfJoints())
    jointPos[0] = data.position[0] 
    jointPos[1] = data.position[1]
    jointPos[2] = data.position[2]
    
    forvKin = kdl.ChainFkSolverPos_recursive(kdlChain)
    eeFrame = kdl.Frame() 
    forvKin.JntToCart(jointPos, eeFrame)


    quaternion = eeFrame.M.GetQuaternion()

    
    robot_pose = PoseStamped()
    robot_pose.header.frame_id = 'base_link'
    robot_pose.header.stamp = rospy.Time.now()


    robot_pose.pose.position.x = eeFrame.p[0]
    robot_pose.pose.position.y = eeFrame.p[1]
    robot_pose.pose.position.z = eeFrame.p[2]

    robot_pose.pose.orientation.x = quaternion[0]
    robot_pose.pose.orientation.y = quaternion[1]
    robot_pose.pose.orientation.z = quaternion[2]
    robot_pose.pose.orientation.w = quaternion[3]

    publisher.publish(robot_pose)


def kdl_listener():
    rospy.init_node('KDL_DKIN', anonymous = False)
    # publisher = rospy.Publisher('n_k_axes', PoseStamped, queue_size=10)

    rospy.Subscriber("joint_states", JointState , callback)

    rospy.spin()

if __name__ == '__main__':
    json_file = {}
    t_list = {}
    publisher = rospy.Publisher('kdl', PoseStamped, queue_size=10)

    
    
    # laczenie z modelem
    try:
	    kdl_listener()        
    except rospy.ROSInterruptException:
	pass
