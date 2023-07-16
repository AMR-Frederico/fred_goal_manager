import rospy 

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Path

# ----- pulishers 
goal_current_pub = rospy.Publisher('/goal_manager/goal/current', PoseStamped, queue_size=10)
mission_completed_pub = rospy.Publisher('/goal_manager/goal/mission_completed', Bool, queue_size=1)

trajectory = []

reset_goals = False

is_goal_reached = False
previous_goal_reached = False

index = 0

current_goal = PoseStamped()

def path_callback(path_msg):
    global trajectory
    
    trajectory = path_msg.poses
    rospy.loginfo("GOAL PROVIDER: path received")

def reset_goals_callback(reset_msg):
    global reset_goals
    global index

    reset_goals = reset_msg.data
    
    if (reset_goals): 
        index = 0
    
    rospy.loginfo('GOAL PROVIDER: reset goals')


def goal_reached_callback(reached_msg): 
    global is_goal_reached

    is_goal_reached = reached_msg.data


def main():
    global is_goal_reached, previous_goal_reached
    global index
    global current_goal, trajectory

    mission_completed = False

    if len(trajectory) == 0:
        rospy.loginfo("GOAL PROVIDER: path has not been received")
        return
    
    if (is_goal_reached and not previous_goal_reached): 
        
        if (index < len(trajectory) - 1): 
            current_goal.header.stamp = rospy.Time.now()
            current_goal.header.frame_id = 'map'
            current_goal.pose = trajectory[index].pose  

            goal_current_pub.publish(current_goal)
        
            index += 1

            rospy.loginfo('GOAL PROVIDER: sending new goal')

        else: 
            mission_completed = True
            mission_completed_pub.publish(mission_completed)

            rospy.loginfo('GOAL PROVIDER: mission completed')
    
    previous_goal_reached = is_goal_reached

if __name__ == '__main__':
    rospy.init_node('goal_provider')
    rate = rospy.Rate(100)

    rospy.Subscriber('/fred_spline_generator/service/out/path', Path, path_callback)
    rospy.Subscriber('/goal_manager/goal/reset', Bool, reset_goals_callback)
    rospy.Subscriber('/goal_manager/goal/spline/reached', Bool, goal_reached_callback)

    while not rospy.is_shutdown():
        main()  
        rate.sleep()

