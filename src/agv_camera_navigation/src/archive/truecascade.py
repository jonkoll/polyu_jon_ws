#!/usr/bin/env python
import rospy
import tf2_ros
import time
from math import sin, cos
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

def traError(TF):
    ex = TF.transform.translation.x
    ey = TF.transform.translation.y
    ez = TF.transform.translation.z
    e_tra = [ex, ey, ez]
    return(e_tra)

def rotError(TF):
    eq1 = TF.transform.rotation.x
    eq2 = TF.transform.rotation.y
    eq3 = TF.transform.rotation.z
    eq4 = TF.transform.rotation.w
    e_quat = [eq1, eq2, eq3, eq4]
    e_rot = euler_from_quaternion(e_quat)
    return(e_rot)


def calc_new_nav_goal(TF_target, TF_current_nav_goal):
    e_tra = traError(TF_target)
    e_rot = rotError(TF_target)

    TF_new_nav_goal = PoseStamped()
    TF_new_nav_goal.header.seq = TF_current_nav_goal.header.seq + 1
    TF_new_nav_goal.header.stamp = rospy.Time()
    TF_new_nav_goal.pose.orientation = TF_current_nav_goal.transform.rotation
    TF_new_nav_goal.pose.position = TF_current_nav_goal.transform.translation

    TF_new_nav_goal.pose.position.x = TF_new_nav_goal.pose.position.x + e_tra[2]
    print('asdf')
    return(TF_new_nav_goal)

if __name__ == '__main__':
    rospy.init_node('truecascade')
    tfBuffer = tf2_ros.Buffer()
    listener_current_nav_goal = tf2_ros.TransformListener(tfBuffer)
    publisher_new_nav_goal = rospy.Publisher('current_goal', PoseStamped, queue_size = 1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            TF_target = tfBuffer.lookup_transform("joint6_flange", "basic_shapes", rospy.Time())
            target_visible = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("no target")
            target_visible = False
            continue

        try:
            TF_current_nav_goal = tfBuffer.lookup_transform("map", "base_link", rospy.Time())
            has_current_goal = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("no goal")
            has_current_goal = False
            continue 

        if (target_visible & has_current_goal):
            TF_new_nav_goal = calc_new_nav_goal(TF_target, TF_current_nav_goal)
            publisher_new_nav_goal.publish(TF_new_nav_goal)
        else:
            time.sleep(1)
        rate.sleep()




