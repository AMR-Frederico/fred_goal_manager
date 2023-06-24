#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Int16,Float32,Float64,Int32,Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseStamped,Pose2D

current_pose = Pose2D()
goal_pose = Pose2D()


ROBOT_IN_GOAL_TOLERANCE = 1

def goal_reached():
    global goal_pose
    global current_pose
    # in_goal = False
    
    dx = goal_pose.x - current_pose.x
    dy = goal_pose.y - current_pose.y

    # calculo do modulo do erro linear
    error_linear = math.hypot(dx, dy)
    in_goal = error_linear < ROBOT_IN_GOAL_TOLERANCE
    
    # print(f"odom: {current_pose.x}, {current_pose.y}| goal_pose: {goal_pose.x}, {goal_pose.y}, error : {error_linear},goal:{in_goal }")
    
    pub_goal_reached.publish(in_goal)


def setpoint_callback(msg): 
    global goal_pose
    goal_pose.x = msg.pose.position.x
    goal_pose.y = msg.pose.position.y 

   


def odom_callback(odom_msg):
    global current_pose

   
    current_pose.x = odom_msg.pose.pose.position.x
    current_pose.y = odom_msg.pose.pose.position.y



if __name__ == '__main__':
    try:
        rospy.init_node('goal_reached_monitor', anonymous=True)
        rate = rospy.Rate(100)
        

        rospy.Subscriber("/odom", Odometry, odom_callback)
        rospy.Subscriber("/goal_manager/goal/current", PoseStamped, setpoint_callback)
        
        pub_goal_reached = rospy.Publisher("goal_manager/goal/reached",Bool, queue_size = 1)
        
        while not rospy.is_shutdown():
            goal_reached()
            rate.sleep()

    
    except rospy.ROSInterruptException:
        pass