#!/usr/bin/env python
import rospy
import tf2_ros
import tf
import time
import move_base
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import TransformStamped, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf.transformations
import os

def generateMBAG(gTm, gTm_quat_corrected, gTm_tran_corrected):
    gTm_MBAG = MoveBaseActionGoal()
    gTm_MBAG.header = gTm.header
    gTm_MBAG.goal_id.stamp = rospy.Time.now()
    gTm_MBAG.goal_id.id = ''
    gTm_MBAG.goal.target_pose.header = gTm.header
    gTm_MBAG.goal.target_pose.header.frame_id = 'map'
    gTm_MBAG.goal.target_pose.pose.position.x = gTm_tran_corrected[0]
    gTm_MBAG.goal.target_pose.pose.position.y = gTm_tran_corrected[1]
    gTm_MBAG.goal.target_pose.pose.position.z = 0 # gTm_tran_corrected[2]
    gTm_MBAG.goal.target_pose.pose.orientation.x = gTm_quat_corrected[0]
    gTm_MBAG.goal.target_pose.pose.orientation.y = gTm_quat_corrected[1]
    gTm_MBAG.goal.target_pose.pose.orientation.z = gTm_quat_corrected[2]
    gTm_MBAG.goal.target_pose.pose.orientation.w = gTm_quat_corrected[3]
    return gTm_MBAG

#def removeZaxis(gTm):


if __name__ == "__main__":
    rospy.init_node("agv_publish_nav_goal")
    tfBuffer = tf2_ros.Buffer()
    listener_gTm = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(2)
    pub_nav_goal = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
    print('starting loop')
    while not rospy.is_shutdown():
        try:
            gTm = tfBuffer.lookup_transform('new_nav_goal', 'map', rospy.Time())
            new_goal = True
            print('we have one')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            new_goal = False
            continue
    
        if new_goal:
            print(gTm)

            gTm_quat = gTm.transform.rotation    
            gTm_tran = gTm.transform.translation
            gTm_quat = [gTm_quat.x, gTm_quat.y, gTm_quat.z, gTm_quat.w]
            gTm_tran = [gTm_tran.x, gTm_tran.y, gTm_tran.z]        
            gTm_mat  = tf.transformations.translation_matrix(gTm_tran).dot(tf.transformations.quaternion_matrix(gTm_quat))

            gTm_decomposed = tf.transformations.decompose_matrix(gTm_mat)
            withoutZ    = gTm_decomposed[2]
            withoutZ[0] = 0.0
            withoutZ[1] = 0.0
            withoutZ    = ([withoutZ[0], withoutZ[1], withoutZ[2]])    #tuple containing a list
            
            gTm_decomposed_corrected  = (gTm_decomposed[0], gTm_decomposed[1], withoutZ, gTm_decomposed[3], gTm_decomposed[4])            
            gTm_mat_corrected = tf.transformations.compose_matrix(*gTm_decomposed_corrected)
            gTm_quat_corrected = tf.transformations.quaternion_from_matrix(gTm_mat_corrected)
            gTm_tran_corrected = tf.transformations.translation_from_matrix(gTm_mat_corrected)
            
            gTm_MBAG = generateMBAG(gTm, gTm_quat_corrected, gTm_tran_corrected)
            print(gTm_MBAG)
            print(euler_from_quaternion([gTm_MBAG.goal.target_pose.pose.orientation.x, gTm_MBAG.goal.target_pose.pose.orientation.y, gTm_MBAG.goal.target_pose.pose.orientation.z, gTm_MBAG.goal.target_pose.pose.orientation.w]))
            print("goal found, killing other threads")
            os.system("rosnode kill opencv_camera")
            os.system("rosnode kill agv_detect_marker")
            os.system("rosnode kill agv_calc_move")
            print("publishing goal")
            pub_nav_goal.publish(gTm_MBAG) 
            print("commiting")
            os.system("rosnode kill agv_publish_nav_goal")

        rate.sleep()




