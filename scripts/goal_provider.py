#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Bool

class GoalManager:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("goal_manager")

        # Subscribe to goals and goal reached topics
        rospy.Subscriber("/goal_manager/goals", PoseArray, self.goals_callback)
        rospy.Subscriber("/goal_manager/goal/reached", Bool, self.goal_reached_callback)

        # Publisher for current goal topic
        self.current_goal_pub = rospy.Publisher("/goal_manager/goal/current", PoseStamped, queue_size=10)

        # Initialize class variables
        self.goals = []
        self.current_goal_index = 0
        self.goal_reached = False

    def goals_callback(self, data):
        # Callback function to handle new goals received on "/goal_manager/goals" topic
        self.goals = data.poses
        self.publish_current_goal()

    def goal_reached_callback(self, data):
        # Callback function to handle goal reached messages received on "/goal_manager/goal/reached" topic
        rospy.loginfo("Goal reached: %s", data.data)

        if data.data and not self.goal_reached:
            self.goal_reached = True

            # Check if there are more goals to go to
            if self.current_goal_index < len(self.goals) - 1:
                # Go to the next goal
                self.current_goal_index += 1
                self.publish_current_goal()
                rospy.loginfo("Changed current goal to index %d", self.current_goal_index)
            else:
                rospy.loginfo("Reached the last goal.")

                # Reset the index to the first goal
                self.current_goal_index = 0
                self.publish_current_goal()

        # Reset goal_reached to False if it has been True and the new message is False
        if not data.data and self.goal_reached:
            self.goal_reached = False

    def publish_current_goal(self):
        # Create a new PoseStamped message and fill in its fields with the current goal
        current_goal = PoseStamped()
        current_goal.header.stamp = rospy.Time.now()
        current_goal.header.frame_id = "odom"
        current_goal.pose = self.goals[self.current_goal_index]

        # Publish the message
        self.current_goal_pub.publish(current_goal)

    def run(self):
        # Start ROS node and spin
        rospy.spin()

if __name__ == '__main__':
    try:
        gm = GoalManager()
        gm.run()
    except rospy.ROSInterruptException:
        pass
