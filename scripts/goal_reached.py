

import rospy
import math
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose2D



class GoalReached:
    def __init__(self):
        

        self.current_pose = Pose2D()
        self.goal_pose = Pose2D()

        self.first_goal_reached = False

        self.ROBOT_IN_GOAL_TOLERANCE = 0.25

        self.pub_goal_reached = rospy.Publisher("goal_manager/goal/reached", Bool, queue_size=10)
        

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/goal_manager/goal/current", PoseStamped, self.setpoint_callback)
        rospy.Subscriber('/navigation/on', Bool, self.navigation_on_callback)

        rospy.loginfo("GOAL_REACHED: init")

    def navigation_on_callback(self, msg):
        pass
        # if msg.data:
        #     self.first_goal_reached = True


    def goal_reached(self):
        dx = self.goal_pose.x - self.current_pose.x
        dy = self.goal_pose.y - self.current_pose.y

        rospy.loginfo(f'GOAL REACHED: goal X = {self.goal_pose.x}  | Y = {self.goal_pose.y}' )

        error_linear = math.hypot(dx, dy)
        in_goal = error_linear < self.ROBOT_IN_GOAL_TOLERANCE

        # print(in_goal)
        # print(error_linear)
        
        if(in_goal):
             rospy.loginfo('GOAL REACHED: arrieved at current goal')
        # else:
        #      rospy.loginfo(f'GOAL REACHED: NOT arrieved at current goal   - error = {error_linear}')
            


        self.pub_goal_reached.publish(in_goal or self.first_goal_reached)
        # self.pub_goal_reached.publish(in_goal)


        # print(in_goal)
        self.first_goal_reached = False

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

