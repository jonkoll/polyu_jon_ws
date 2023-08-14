#!/usr/bin/env python
import rospy
import tf2_ros
import tf
import time
from math import sin, cos
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf.transformations

def generateGoal():
    gTt = TransformStamped()
    gTt.header.seq = 1
    gTt.header.stamp = rospy.Time.now()
    gTt.header.frame_id = 'new_nav_goal'
    gTt.child_frame_id = 'basic_shapes'
    gTt.transform.translation = [0, 0, 0.6]
    gTt.transform.rotation = quaternion_from_euler(3.14/4, 0, 3.14/4)
    return gTt


def transformChain(aTb, bTc, aTc_old):
    aTc = TransformStamped()
    aTc.header.seq = aTc_old.header.seq + 1
    aTc.header.stamp = rospy.Time.now()
    aTc.header.frame_id = aTc_old.header.frame_id
    aTc.child_frame_id = aTc_old.child_frame_id
    
    aTb_quat = aTb.transform.rotation           #array
    aTb_tran = aTb.transform.translation        #array

    bTc_quat_tmp = bTc.transform.rotation       #quaternion object
    bTc_tran_tmp = bTc.transform.translation    #some kind of dictionary
    bTc_quat = [bTc_quat_tmp.x, bTc_quat_tmp.y, bTc_quat_tmp.z, bTc_quat_tmp.w]
    bTc_tran = [bTc_tran_tmp.x, bTc_tran_tmp.y, bTc_tran_tmp.z]

    aTb_mat  = tf.transformations.translation_matrix(aTb_tran).dot(tf.transformations.quaternion_matrix(aTb_quat))
    bTc_mat  = tf.transformations.translation_matrix(bTc_tran).dot(tf.transformations.quaternion_matrix(bTc_quat))
    aTc_mat  = aTb_mat.dot(bTc_mat)

    aTc_tran = tf.transformations.translation_from_matrix(aTc_mat)
    aTc_quat = tf.transformations.quaternion_from_matrix(aTc_mat)

    aTc.transform.rotation    = aTc_quat
    aTc.transform.translation = aTc_tran
    return(aTc)

if __name__ == '__main__':
    print('main')
    rospy.init_node('agv_calc_move')
    tfBuffer = tf2_ros.Buffer()
    listener_tTm = tf2_ros.TransformListener(tfBuffer)       # transform from map to camera
    gTt = generateGoal()
    gTm = TransformStamped()
    gTm.header.seq = 0
    gTm.header.frame_id = 'basic_shapes'
    gTm.child_frame_id  = 'camera'
    rate = rospy.Rate(1)
    new_goal_broadcaster = tf.TransformBroadcaster()

    print('setup done')

    while not rospy.is_shutdown():
        try:
            tTm = tfBuffer.lookup_transform('basic_shapes', 'camera', rospy.Time())
            target_visible = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            target_visible = False
            continue
        if target_visible:
            gTm = transformChain(gTt, tTm, gTm)
            new_goal_broadcaster.sendTransform(gTm.transform.translation, gTm.transform.rotation, rospy.Time.now(), "new_nav_goal", "camera")
           
        rate.sleep()

        

