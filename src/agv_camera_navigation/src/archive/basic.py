#!/usr/bin/env python
import rospy
import tf2_ros
import time
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

def move(e_tra, e_rot, pub, happycounter):
    goal_z = 0.5
    tol_z = 0.01
    goal_x = 0
    tol_x = 0.02
    speed = 0.2
    e_rot = rad2deg(e_rot)
    done = True
    unstable_move = False
    msg = Twist()
    #translational movement
    
    if e_tra[0] > (goal_x + tol_x):  #target too far left
        msg.linear.y = -speed
        print('moving right', e_tra[0])
        done = False
        unstable_move = True

    elif e_tra[0] < -(goal_x + tol_x): #target too far right
        msg.linear.y = speed
        print('moving left', e_tra[0])
        done = False
        unstable_move = True

    
    if e_tra[2] > (goal_z + tol_z):   #target too far away
        msg.linear.x = speed*2
        print('moving forwards', e_tra[2])
        done = False

    elif e_tra[2] < -(goal_z + tol_z):  #target too close
        msg.linear.x = -speed*2
        print('moving backwards', e_tra[2])
        done = False  
    
    
    #rotational movemen
    if e_rot[2] > 10:                    #degrees
        msg.angular.z = 0.2
        print('counterclockwise', e_rot[1])
        done = False
        unstable_move = True

    
    elif e_rot[2] < -10:
        msg.angular.z = -0.2
        print('against unit circle', e_rot[1])
        done = False
        unstable_move = True

    
    if not done:
        pub.publish(msg)
        if not unstable_move:
            time.sleep(0.8)
        else:
            time.sleep(0.8)
        happycounter = 0
    else:
        happycounter = happycounter + 1
    #msg = Twist()
    #pub.publish(msg)
    return happycounter
    #time.sleep(0.5)
    
    

if __name__ == '__main__':
    rospy.init_node('basic')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(1)                                #slow to wait for camera
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=2)
    i = 0
    happycounter = 0

    time.sleep(2)
    print("let's go")
    while not rospy.is_shutdown():
        if (i % 2 != 0):
            try:
                TF = tfBuffer.lookup_transform("joint6_flange", "basic_shapes", rospy.Time())
                e_tra = traError(TF)
                e_rot = rotError(TF)

                happycounter = move(e_tra, e_rot, pub, happycounter)
                if happycounter > 10:
                    break

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                print('exeption')
                continue
        else:
            msg = Twist()
            pub.publish(msg)
            time.sleep(2)        #let camera catch up
            print("letting camera catch up")
        i = i + 1
        rate.sleep()
    
    print("Translational error: ", [round(n, 4) for n in e_tra])
    print("Rotational error:    ", [round(n, 4) for n in e_rot])
    print("I am happy here :)")