#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

goal_number = 0 
current_goal = Pose2D()


def publish_new_goal():
    global current_goal_pub
    current_goal_pub.publish(current_goal)
    goal_number = goal_number + 1 

def current_goal_callback(msg):
    global current_goal
    current_goal = msg


def callback_goal_reached(msg):
    global last_goal_reached

    # Verifica se houve uma mudança de False para True
    if not last_goal_reached and msg.data:
        publish_new_goal()

    # Armazena o último valor recebido
    last_goal_reached = msg.data


if __name__ == '__main__':
    rospy.init_node('goal_provider_node')

    last_goal_reached = False

    rospy.Subscriber('goal_manager/goal/reached', Bool, callback_goal_reached)
    rospy.Subscriber('goal_manager/goal/' + str(goal_number),Pose2D,current_goal_callback)
    current_goal_pub = rospy.Publisher('goal_manager/goal/current', Pose2D, queue_size=10)

    rospy.spin()
