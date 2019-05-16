#! /usr/bin/python
import rospy
import json
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import *

# defining axes
xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)

# length of the manipulator's tip
end_len=0
# matrice for the translation of tip
tz = translation_matrix((0, 0, 0))
rz = rotation_matrix(0, zaxis)
tx = translation_matrix((end_len, 0, 0))
rx = rotation_matrix(0, xaxis)
end_matrice = concatenate_matrices(tx, rx, tz, rz)


def callback(data):
    global tz
    global rz
    global rx
    global tx
    mainMatrix = translation_matrix((0, 0, 0))

    # reading dh table parameters
    with open('/home/kaczmi/catkin_ws/src/anro-kaczmarek_kobylinski/dh2urdf/dh.json') as file:
        params = json.loads(file.read())
	matrices = {}
        for key in params.keys():
            a, d, al, th = params[key]
            al, a, d, th = float(al), float(a), float(d), float(th)

            # applying changes published by joint_state_publisher
            if key == '1':
                tz = translation_matrix((0, 0, d+data.position[0]))
                rz = rotation_matrix(th+data.position[1], zaxis)
            
	    if key == '2':
                tz = translation_matrix((0, 0, d))
                rz = rotation_matrix(th+data.position[2], zaxis)
            if key == '3' :
                tz = translation_matrix((0, 0, d))
                rz = rotation_matrix(th-1.57, zaxis)

	    # creating matrix
            tx = translation_matrix((a, 0, 0))
            rx = rotation_matrix(al, xaxis)
            matrices[key] = concatenate_matrices(tx, rx, tz, rz)
    
    # multiplication of matrices
    for key in sorted(params.keys()):
	mainMatrix = concatenate_matrices(mainMatrix,matrices[key])

    mainMatrix = concatenate_matrices(mainMatrix,end_matrice)
     
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

    publisher = rospy.Publisher('nonkdl', PoseStamped, queue_size=10)

    try:
	    nonkdl_listener()        
    except rospy.ROSInterruptException:
	pass
