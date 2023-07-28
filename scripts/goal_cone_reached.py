#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose2D, PoseArray,Pose
import tf


class ConeReached:
    def __init__(self):
        

        self.current_pose = Pose2D()
        self.goal_pose = Pose2D()
        self.Cones_array = PoseArray()
        # self.conesArray = [   
        #   [10.12, 6.86],    # 2° MARCO
        #   [00.42, 15.27],    # 3° MARCO
        #   [10.84, 0.31],    # 4° MARCO
        #    [50,50]
        #]
        self.conesArray = [   
         [1, 0],    # 2° MARCO
         [2, 0],    # 3° MARCO
         [3, 0],    # 4° MARCO
         [1, 0],
         [50,50]
        ]
        self.first_goal_reached = False
        self.init_ctrl_points()
        self.index = 0
        self.in_goal = False
        self.last_in_goal = False 

        self.ROBOT_IN_GOAL_TOLERANCE = 0.25


        self.pub_goal_reached = rospy.Publisher("goal_manager/cone/reached", Bool, queue_size=10)
        
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/odom/reset", Bool, self.reset)
        rospy.loginfo("CONE_REACHED: init")
        
    


    def init_ctrl_points(self):

       
        self.Cones_array.header.frame_id = "map"
        self.Cones_array.header.stamp = rospy.Time.now()

        for sp in self.conesArray:

            p = Pose()

            p.position.x = sp[0]
            p.position.y = sp[1]
            p.position.z = 0

            quat = tf.transformations.quaternion_from_euler(
                0, 0, -math.radians(0))

            p.orientation.x = quat[0]
            p.orientation.y = quat[1]
            p.orientation.z = quat[2]
            p.orientation.w = quat[3]



            self.Cones_array.poses.append(p)

        
        

    def reset(self,msg):
        if(msg.data):
            self.index = 0 


    def goal_reached(self):
        dx = self.Cones_array.poses[self.index].position.x - self.current_pose.x
        dy = self.Cones_array.poses[self.index].position.y - self.current_pose.y


        error_linear = math.hypot(dx, dy)
        self.in_goal = error_linear < self.ROBOT_IN_GOAL_TOLERANCE
        rospy.loginfo(f'GOAL cone: {self.in_goal} goal X = {self.Cones_array.poses[self.index].position.x}  | Y = { self.Cones_array.poses[self.index].position.y} | i : {self.index} | {self.current_pose.x} : {self.current_pose.y}' )

        # print(in_goal)
        # print(error_linear)
        
        
            


        # self.pub_goal_reached.publish(in_goal)

        if(self.in_goal > self.last_in_goal):
            self.index = self.index + 1
        
        self.pub_goal_reached.publish(self.in_goal)
        self.last_in_goal = self.in_goal

      

    def setpoint_callback(self, msg):
        self.goal_pose.x = msg.pose.position.x
        self.goal_pose.y = msg.pose.position.y 


    def odom_callback(self, odom_msg):
        self.current_pose.x = odom_msg.pose.pose.position.x
        self.current_pose.y = odom_msg.pose.pose.position.y
        
        dx = self.Cones_array.poses[self.index].position.x - self.current_pose.x
        dy = self.Cones_array.poses[self.index].position.y - self.current_pose.y

        print(f"-------------------------{dx}: {dy}")
        error_linear = math.hypot(dx, dy)
        self.in_goal = error_linear < self.ROBOT_IN_GOAL_TOLERANCE
        # rospy.loginfo(f'GOAL cone: {self.in_goal} goal X = {self.Cones_array.poses[self.index].position.x}  | Y = { self.Cones_array.poses[self.index].position.y} | i : {self.index} | {self.current_pose.x} : { odom_msg.pose.pose.position.x}' )


            



if __name__ == "__main__":

    rospy.init_node('cone_manager')
    rate = rospy.Rate(0.5)
    
    cone = ConeReached()
    

    while not rospy.is_shutdown():
           
            
            rate.sleep()
