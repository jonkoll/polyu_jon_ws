#!/usr/bin/env python
import rospy
import tf2_ros
import time
from math import sin, cos
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

def rad2deg(rad):
    deg = []
    for a in rad:
        deg.append(a*57.3)
    return deg

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


def move(traError, rotError, pub):
    vehicle_z = traError[2]
    vehicle_y = traError[1] + 0.04  #calibration to centre of target
    vehicle_rot = rotError[2]
    if rotError[0] < 0: vehicle_rot = -vehicle_rot

    print('e_tra: ', traError)
    print('e_rot: ', rotError)
    print('z:', vehicle_z, 'y: ', vehicle_y, 'a: ', vehicle_rot)

    goal_z = 0.6
    goal_y = 0
    goal_rot = 0
    tra_tol = 0.02
    rot_tol = 0.2
    speed = 0.3
    done = True
    msg = Twist()

    if vehicle_z > (goal_z + tra_tol):
        msg.linear.x = speed
        done = False
        print('forwards')
    elif vehicle_z < (goal_z - tra_tol):
        msg.linear.x = -speed
        done = False
        print('backwards')
    
    if vehicle_y < (goal_y - tra_tol):
        msg.linear.y = speed
        done = False
        print('left')
    elif vehicle_y > (goal_y + tra_tol):
        msg.linear.y = -speed
        done = False
        print('right')

    if vehicle_rot > (goal_rot + rot_tol):
        msg.angular.z = -speed*0.7
        done = False
        print('clockwise')
    elif vehicle_rot < -(goal_rot + rot_tol):
        msg.angular.z = +speed*0.7
        done = False
        print('counterclockwise')
    
    if not done:
        pub.publish(msg)
        time.sleep(0.3)
    else:
        print('within tolerances')



if __name__ == '__main__':
    rospy.init_node('basic')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10)                                
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=2)
    i = 0
    print("let's go")
    while not rospy.is_shutdown():
        if (i % 2 != 0):
            try:
                TF = tfBuffer.lookup_transform("joint6_flange", "basic_shapes", rospy.Time())
                e_tra = traError(TF)
                e_rot = rotError(TF)
                move(e_tra, e_rot, pub)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            rate.sleep()
        else:
            msg = Twist()
            pub.publish(msg)
            time.sleep(3)
            print('resting')
        i = i + 1
        rate.sleep()
    
    print("Translational error: ", [round(n, 4) for n in e_tra])
    print("Rotational error:    ", [round(n, 4) for n in e_rot])
    print("I am happy here :)")