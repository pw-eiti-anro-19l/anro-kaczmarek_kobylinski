#! /usr/bin/python
import rospy
from numpy import * 
import json
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import *
from math import sin, cos 

# defining axes
xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)


def callback(data):
    mainMatrix = translation_matrix((0, 0, 0))

    # reading dh table parameters
    with open('/home/kkaczma5/catkin_ws/src/anro-kaczmarek_kobylinski/kdlandnonkdl/dh.json') as file:
	    params = json.loads(file.read())
	    matrices = {}
	    for key in params.keys():
		a, d, al, th = params[key]
		al, a, d, th = float(al), float(a), float(d), float(th)
		if key=='1':
			a1=a
			d1=d
			th1=th
		if key=='2':
			a2=a
			th2=th 
		if key=='3':
			a3=a
			th3=th
    data0=data.position[0]
    data1=data.position[1]
    data2=data.position[2]
    mainMatrix=array([[ cos(th3 - 157/100)*(cos(data1 + th1)*cos(data2 + th2) - sin(data1 + th1)*sin(data2 + th2)) - sin(th3 - 157/100)*(cos(data1 + th1)*sin(data2 + th2) + cos(data2 + th2)*sin(data1 + th1)), - cos(th3 - 157/100)*(cos(data1 + th1)*sin(data2 + th2) + cos(data2 + th2)*sin(data1 + th1)) - sin(th3 - 157/100)*(cos(data1 + th1)*cos(data2 + th2) - sin(data1 + th1)*sin(data2 + th2)), 0, a2*cos(data1 + th1) + a3*(cos(data1 + th1)*cos(data2 + th2) - sin(data1 + th1)*sin(data2 + th2))],
[ cos(th3 - 157/100)*(cos(data1 + th1)*sin(data2 + th2) + cos(data2 + th2)*sin(data1 + th1)) + sin(th3 - 157/100)*(cos(data1 + th1)*cos(data2 + th2) - sin(data1 + th1)*sin(data2 + th2)),   cos(th3 - 157/100)*(cos(data1 + th1)*cos(data2 + th2) - sin(data1 + th1)*sin(data2 + th2)) - sin(th3 - 157/100)*(cos(data1 + th1)*sin(data2 + th2) + cos(data2 + th2)*sin(data1 + th1)), 0, a2*sin(data1 + th1) + a3*(cos(data1 + th1)*sin(data2 + th2) + cos(data2 + th2)*sin(data1 + th1))],
[                                                                                                                                                                                       0,                                                                                                                                                                                         0, 1,                                                                                       d1 + data0],
[                                                                                                                                                                                       0,                                                                                                                                                                                         0, 0,                                                                                                1]])
    # counting the parameters from the mainMatrix
    x , y , z = translation_from_matrix(mainMatrix)
    
    poseR = PoseStamped()
    poseR.header.frame_id = "base_link"
    poseR.header.stamp = rospy.Time.now()
    poseR.pose.position.x = x
    poseR.pose.position.y = y
    poseR.pose.position.z = z
    
    xq, yq, zq, wq = quaternion_from_matrix(mainMatrix)

    poseR.pose.orientation.x = xq
    poseR.pose.orientation.y = yq
    poseR.pose.orientation.z = zq
    poseR.pose.orientation.w = wq

    # publishing the position via proper topic
    publisher.publish(poseR)


def nonkdl_listener():

    rospy.init_node('NONKDL_DKIN', anonymous = False)

    rospy.Subscriber("joint_states", JointState , callback)

    rospy.spin()

if __name__ == '__main__':

    publisher = rospy.Publisher('nonkdl2', PoseStamped, queue_size=10)

    try:
	    nonkdl_listener()        
    except rospy.ROSInterruptException:
	pass
