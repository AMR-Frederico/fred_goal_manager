#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path,Odometry
from tf.transformations import euler_from_quaternion

class GoalPathPublisher:
    def __init__(self):
        rospy.init_node('goal_path_publisher')
        self.rate = rospy.Rate(10)  # Defina a taxa de atualização desejada (10 Hz neste exemplo)
        self.path_sub = rospy.Subscriber('/fred_spline_generator/service/out/path', Path, self.path_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_goal_pub = rospy.Publisher('/goal_manager/goal/current', PoseStamped, queue_size=1)

        self.path_received = False
        self.current_path = Path()

        self.current_goal_index = 0
        self.tolerance = 0.2  # Defina a tolerância aceitável em metros
        self.stopped_at_goal_time = None
        self.goal_updated = False

        # Variáveis para verificar se o robô chegou ao objetivo
        self.is_goal_reached = False
        self.previous_goal_reached = False

    def path_callback(self, path_msg):
        if not self.path_received:
            self.path_received = True
            self.current_path = path_msg
            self.publish_current_goal()

    def odom_callback(self, odom_msg):

        current_pose = odom_msg.pose
        print(f"{current_pose.pose.position.x}: {current_pose.pose.position.y}")
        try:
          current_goal = self.current_path.poses[self.current_goal_index]
        except:
         pass 

        if not self.path_received:
            return

        # Verificar a distância entre a posição atual do robô e o objetivo atual
        dx = current_pose.pose.position.x - current_goal.pose.position.x
        dy = current_pose.pose.position.y - current_goal.pose.position.y
        distance_to_goal = (dx ** 2 + dy ** 2) ** 0.5

        self.is_goal_reached = distance_to_goal < 0.5
         
        if self.current_goal_index < len(self.current_path.poses):
            self.publish_current_goal()
        else:
            rospy.loginfo("Reached the last goal.")
            self.path_received = False
            self.current_goal_index = 0

        

    
     
        if self.is_goal_reached and not self.previous_goal_reached:
            
            if self.current_goal_index < len(self.current_path.poses):
                self.current_goal_index += 1
                self.publish_current_goal()

        self.previous_goal_reached = self.is_goal_reached

    def publish_current_goal(self):
        if self.current_path and self.current_goal_index < len(self.current_path.poses):
            current_goal = self.current_path.poses[self.current_goal_index]
            current_goal.header.stamp = rospy.Time.now()
            current_goal.header.frame_id = 'odom'
            self.current_goal_pub.publish(current_goal)
            rospy.loginfo(f"Published goal {self.current_goal_index + 1} of {len(self.current_path.poses)}")

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        goal_path_publisher = GoalPathPublisher()
        goal_path_publisher.run()
    except rospy.ROSInterruptException:
        pass
