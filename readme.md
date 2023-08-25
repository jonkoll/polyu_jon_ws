# AGV_CAMERA_NAVIGATION
The package works alongside the navigation_demo package provided by the manufacturer to let the myAGV omnidirectional vehicle navigate using camera-based positioning. agv_detect_marker.py runs alongside opencv_camera from the mycobot_280 (another Elephant Robotics product) package. It detects ARUCO codes and calculates the relative position and rotation of the code and the vehicle. The code frame is made available to other nodes using TF2, and can be seen in the RVIZ interface. 

In the launchfile, a goal frame relative to this target frame is calculated. Once the target becomes visible the goal frame is calculated, and agv_publish_nav_goal builds a MoveBaseActionGoal which is then published on the /movebase/goal topic. This topic is read by navigation_demo, which then attempts to move the vehicle to this goal. 

# DEMO MANUAL
To demo the package:

0. Print out an ARUCO-code and place it in a suitable location. Decide on a goal position and orientation relative to this target. Alter agv_camera_navigation.launch in accordance. 
1. Use the navigation_demo package to build a map of the area in which the vehicle will operate (`roslaunch navigation_demo slam.launch`)
2. Run navigation_demo, and navigate the vehicle to somewhere from which the target can be seen (`roslaunch navigation_demo navigation.launch`)
3. Run agv_camera_navigation.launch to start the camera and calculate the goal. It will become visible in RVIZ. (`roslaunch agv_camera_navigation agv_camera_navigation.launch`)
4. Run agv_publish_nav_goal.py to publish the goal (`rosrun agv_camera_navigation agv_publish_nav_goal.py`)
The robot will now move to the goal. 

# USING THE PACKAGE
Some things you should be aware of when using the package:
* All lengths are given in meters, and all rotations in radians
* The ARUCO code should be 10cm in width. If using codes of a different size line 65 in agv_detect_marker.py should be changed to reflect this.  
* The transform between the target frame and the goal frame is set in the launchfile, and uses an intermediate transform to base_goal for convenience. If the transform from base_goal to new_nav_goal is left as the default identity matrix, the goal will be located 50cm directly in front of the target, with the robot facing it. 

# LIMITATIONS, WEAKNESSES AND BUGS
The most important limitations are the ones given by the hardware and the navigation_demo package. Since the latter is used as an inner control loop the precision of the navigation is bounded by it's precision. In addition, the vehicle struggles with making precise, short sideways motion and rotation. This is precisely the kind of movement that it is often required to do, which is a fundamental weakness of this design. Using the transform from the vehicle to the goal to determine how it should move without using the navigation_demo layer inbetween would in theory alleviate this, and since the vehicle is omnidirectional and position and rotation can be solved for independently it is mathematically simple. In reality it would of course cause all kinds of other issues, and likely be a much less elegant and flexible solution. 

There are no known bugs in the software, but it has not been subject to rigorous testing. 

# SUGGESTIONS FOR FUTURE EXPANSIONS
The myAGV vehicle is marketed as being able to navigate using LIDAR-based SLAM, fine-tune its position and orientation using computer vision, allowing a robot arm attached to it to then pick up an object. The navigation_demo package does the first and this package the second, but the software for the third step is still missing. 

Tweaking this package for a setup with an object located somewhere near a target, making the vehicle move close to it, should be reasonably simple. My impression is that based on the provided code having the arm then pick up an object should not be too difficult. The interactions between the arm and the vehicle (assuming no problems in making them communicate at all) should be easily manageable with ROS.
