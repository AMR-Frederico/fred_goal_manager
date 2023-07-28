# path_monitor.py

import rospy
import math
import tf
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose2D

class PathMonitor:

    def __init__(self):

        # config
        self.TRANSFORM_TOLERANCE = 0.2 # m

        rospy.Subscriber("/odom", Odometry, self._get_position)


        # intern vars
        self.tf_listener = tf.TransformListener()    
        self.pub_goal_reached = rospy.Publisher("goal_manager/goal/reached", Bool, queue_size=1)



    def _get_position(self, odom_msg):

        # now = rospy.Time(0)
        # self.tf_listener.waitForTransform("/map", "/base_link", now)
        # (trans, rot) = self.tf_listener.lookupTransform("/base_link", "/map", now)

        trans = odom_msg.pose.pose.position
        rot = odom_msg.pose.pose.orientation

        return (trans, rot)



    def _check_if_in_target_pose(self, target_trans):

        (trans, rot) = self._get_position()

        # check if in target pose
        if trans[0] - target_trans[0] < self.TRANSFORM_TOLERANCE:

            print(f"GOAL REACHED: DIST X OK -> {trans[0] - target_trans[0]}")
            return True
        
        if trans[1] - target_trans[1] < self.TRANSFORM_TOLERANCE:
            
            print(f"GOAL REACHED: DIST Y OK -> {trans[1] - target_trans[1]}")

            return True
        
        return False
        print(f"DIST NOK    DX = {trans[0] - target_trans[0]}  |  DY = {trans[1] - target_trans[1]}")


    def routine(self):

        in_pose = self._check_if_in_target_pose()

        msg = Bool(in_pose)

        self.pub_goal_reached(msg)