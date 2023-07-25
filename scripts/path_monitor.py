
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


        # intern vars
        self.tf_listener = tf.TransformListener()    
        self.pub_goal_reached = rospy.Publisher("goal_manager/goal/reached", Bool, queue_size=1)



    def _get_position(self):

        now = rospy.Time(0)
        self.tf_listener.waitForTransform("/map", "/base_link", now)
        (trans, rot) = self.tf_listener.lookupTransform("/base_link", "/map", now)

        return (trans, rot)



    def _check_if_in_target_pose(self, target_trans):

        (trans, rot) = self._get_position()

        # check if in target pose
        if trans[0] - target_trans[0] < self.TRANSFORM_TOLERANCE:

            return True
        
        if trans[1] - target_trans[1] < self.TRANSFORM_TOLERANCE:
            
            return True
        
        return False



    def routine(self):

        in_pose = self._check_if_in_target_pose()

        msg = Bool(in_pose)

        self.pub_goal_reached(msg)