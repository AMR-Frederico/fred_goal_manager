#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool

class GoalManager:
    def __init__(self):
       
        # Subscribe to goals and goal reached topics
        rospy.Subscriber("/spline_generator/out/path", Path, self.goals_callback)
        rospy.Subscriber("/goal_manager/goal/spline/reached", Bool, self.goal_reached_callback)
        rospy.Subscriber("/goal_manager/goal/reset", Bool, self.reset_goals_callback)
        # Publisher for current goal topic
        self.current_goal_pub = rospy.Publisher("/goal_manager/goal/current", PoseStamped, queue_size=10)
        self.mission_completed_pub = rospy.Publisher("/goal_manager/goal/mission_completed", Bool, queue_size=10)
        # Initialize class variables
        self.path = [1]
        self.current_goal_index = 1
        self.goal_reached_flag = False
    

    def reset_goals_callback(self,msg):
        reset = msg.data 
        if(reset):
             self.current_goal_index = 0

    def goals_callback(self, data):
        # Callback function to handle new goals received on "/goal_manager/goals" topic
        self.path = data.poses
        # print(self.path)
        print("received goals")
        self.publish_current_goal()

    def goal_reached_callback(self, msg):
        # muda pro proximo objetivo 
        # Callback function to handle goal reached messages received on "/goal_manager/goal/reached" topic
        # rospy.loginfo("Goal reached: %s", data.data)
        
        if self.path is None:
            return
        
        print(msg.data)
        print(self.goal_reached_flag)
        print('-------------')
        if (msg.data and not self.goal_reached_flag) or (msg.data and self.goal_reached_flag):
            self.goal_reached_flag = True

           
            # Check if there are more goals to go to
            if self.current_goal_index < len(self.path) - 1:
                # Go to the next goal
                self.current_goal_index += 1
                self.publish_current_goal()
                # rospy.loginfo("Changed current goal to index %d", self.current_goal_index)
                self.mission_completed_pub.publish(False)
            else:
                # rospy.loginfo("Reached the last goal.")
                self.mission_completed_pub.publish(True)
                # Reset the index to the first goal
                self.current_goal_index = 0

        else: 
            print("aaaa")
        
        # Reset goal_reached to False if it has been True and the new message is False
        if not msg.data and self.goal_reached_flag:
            self.goal_reached_flag = False
            print("b")

    def publish_current_goal(self):
            print("new goal")
            # Create a new PoseStamped message and fill in its fields with the current goal
            current_goal = PoseStamped()
            current_goal.header.stamp = rospy.Time.now()
            current_goal.header.frame_id = "map"
            # print(self.goals)
            current_goal.pose = self.path[self.current_goal_index].pose
            print(current_goal.pose)
            # Publish the message
            self.current_goal_pub.publish(current_goal)
        

    def run(self):
        # Start ROS node and spin
       
        rospy.spin()

init_spline = rospy.Publisher("/spline_generator/cmd/generate_spline", Bool, queue_size=10)
msg = Bool()

if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node("goal_manager")
        
        msg.data = True
        init_spline.publish(msg)
        gm = GoalManager()
        gm.run()
    except rospy.ROSInterruptException:
        pass
