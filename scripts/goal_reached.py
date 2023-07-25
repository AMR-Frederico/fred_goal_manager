

import rospy
import math
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose2D



class GoalManager:
    def __init__(self):
        

        self.current_pose = Pose2D()
        self.goal_pose = Pose2D()

        self.ROBOT_IN_GOAL_TOLERANCE = 0.2

        self.pub_goal_reached = rospy.Publisher("goal_manager/goal/reached", Bool, queue_size=1)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/goal_manager/goal/current", PoseStamped, self.setpoint_callback)


    def goal_reached(self):
        dx = self.goal_pose.x - self.current_pose.x
        dy = self.goal_pose.y - self.current_pose.y

        error_linear = math.hypot(dx, dy)
        in_goal = error_linear < self.ROBOT_IN_GOAL_TOLERANCE
        
        if(in_goal):
             rospy.loginfo('GOAL REACHED: arrieved at current goal')
        else:
             rospy.loginfo('GOAL REACHED: NOT arrieved at current goal')
            


        self.pub_goal_reached.publish(in_goal)

    def setpoint_callback(self, msg):
        self.goal_pose.x = msg.pose.position.x
        self.goal_pose.y = msg.pose.position.y 


    def odom_callback(self, odom_msg):
        self.current_pose.x = odom_msg.pose.pose.position.x
        self.current_pose.y = odom_msg.pose.pose.position.y

        self.goal_reached()


    # def run(self):
    #     rate = rospy.Rate(50)
        
    #     while not rospy.is_shutdown():
    #         self.goal_reached()
    #         rate.sleep()

