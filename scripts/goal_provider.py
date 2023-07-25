import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Path

class GoalProvider:
    def __init__(self):
        # rospy.init_node('goal_provider')
        self.rate = rospy.Rate(50)
        self.trajectory = []
        self.reset_goals = False
        self.is_goal_reached = False
        self.previous_goal_reached = False
        self.index = 0
        self.current_goal = PoseStamped()

        # Publishers
        self.goal_current_pub = rospy.Publisher('/goal_manager/goal/current', PoseStamped, queue_size=10)
        self.mission_completed_pub = rospy.Publisher('/goal_manager/goal/mission_completed', Bool, queue_size=1)

        # Subscribers
        rospy.Subscriber('/fred_spline_generator/service/out/path', Path, self.path_callback)
        rospy.Subscriber('/goal_manager/goal/reset', Bool, self.reset_goals_callback)
        rospy.Subscriber('/goal_manager/goal/reached', Bool, self.goal_reached_callback)

    def path_callback(self, path_msg):
        self.trajectory = path_msg.poses
        rospy.loginfo("GOAL PROVIDER: path received")

    def reset_goals_callback(self, reset_msg):
        self.reset_goals = reset_msg.data
        if self.reset_goals:
            self.index = 0
        rospy.loginfo('GOAL PROVIDER: reset goals')

    def goal_reached_callback(self, reached_msg):
        self.is_goal_reached = reached_msg.data

        # print(f"reached callback msg: {self.is_goal_reached }")

    def main(self):
        mission_completed = False

        if len(self.trajectory) == 0:
            rospy.loginfo("GOAL PROVIDER: path has not been received")
            return

        if self.is_goal_reached and not self.previous_goal_reached:
            if self.index < len(self.trajectory) - 1:
                self.current_goal.header.stamp = rospy.Time.now()
                self.current_goal.header.frame_id = 'map'
                self.current_goal.pose = self.trajectory[self.index].pose

                self.goal_current_pub.publish(self.current_goal)

                self.index += 1

                rospy.loginfo('GOAL PROVIDER: sending new goal')
            else:
                mission_completed = True
                self.mission_completed_pub.publish(mission_completed)

                rospy.loginfo('GOAL PROVIDER: mission completed')

        self.previous_goal_reached = self.is_goal_reached

    def run(self):
        while not rospy.is_shutdown():
            self.main()
            self.rate.sleep()
    
if __name__ == '__main__':
    goal_provider = GoalProvider()
    goal_provider.run()
