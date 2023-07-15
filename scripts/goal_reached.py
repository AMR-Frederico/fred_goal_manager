#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose2D

current_pose = Pose2D()
goal_pose = Pose2D()


ROBOT_IN_GOAL_TOLERANCE = 0.2

goal_acknowledge = False
is_path_received = False

ACKNOWLEDGE_TIMEOUT = rospy.Duration(0.5)  # Set the duration for path timeout in seconds

pub_calculate_spline = rospy.Publisher("spline_generator/cmd/generate_spline", Bool, queue_size=1)
pub_goal_reached = rospy.Publisher("goal_manager/goal/reached", Bool, queue_size=1)
pub_goal_acknowledge = rospy.Publisher("goal_manager/goal/acknowledge", Bool, queue_size=1)

def goal_reached():
    global goal_pose
    global current_pose

    dx = goal_pose.x - current_pose.x
    dy = goal_pose.y - current_pose.y

    error_linear = math.hypot(dx, dy)
    in_goal = error_linear < ROBOT_IN_GOAL_TOLERANCE

    pub_goal_reached.publish(in_goal)


def setpoint_callback(msg):
    global goal_pose
    
    goal_pose.x = msg.pose.position.x
    goal_pose.y = msg.pose.position.y 

def spline_callback(spline_msg):
    global is_path_received
    
    is_path_received = True
    pub_goal_acknowledge.publish(is_path_received)  # Publish the goal acknowledge variable

def odom_callback(odom_msg):
    global current_pose

    current_pose.x = odom_msg.pose.pose.position.x
    current_pose.y = odom_msg.pose.pose.position.y

def path_timeout_callback(event):
    global is_path_received 
    
    if not is_path_received:
        is_path_received = False
        pub_goal_acknowledge.publish(is_path_received)  # Publish False when path times out

if __name__ == '__main__':
    try:
        rospy.init_node('goal_reached_monitor', anonymous=True)
        rate = rospy.Rate(100)

        rospy.Subscriber("/odom", Odometry, odom_callback)
        rospy.Subscriber("/goal_manager/goal/current", PoseStamped, setpoint_callback)
        rospy.Subscriber("/spline_generator/out/path", Path, spline_callback)

        pub_calculate_spline.publish(True)


        while not rospy.is_shutdown():
            goal_reached()
            rate.sleep()
            rospy.Timer(ACKNOWLEDGE_TIMEOUT, path_timeout_callback)  # Start the path timeout timer

            print(is_path_received)

    except rospy.ROSInterruptException:
        pass
