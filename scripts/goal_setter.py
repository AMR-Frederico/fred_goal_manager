import rospy
import math
from geometry_msgs.msg import PoseArray, Pose

goals = [[0,0,0],[15.2,-1.91,0], [9,-12.7,0]]#, [10.27,-7.22,0], [5.42, -5.4, 0]]#,[0,0,0]]
frame_id = 'odom'

def goals_publisher(goal):
    # Inicializa o nó do ROS
    rospy.init_node('goal_setter_node')
    
    # publica o valor de n_goals do ROS param
    n_goals = len(goal)
    rospy.set_param('~n_goals',n_goals)

    # Cria o publicador para o tópico
    pub = rospy.Publisher('goal_manager/goals', PoseArray, queue_size=10)

    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        # Cria uma mensagem PoseArray vazia
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = frame_id

        # Adiciona as poses na mensagem
        for i in range(len(goal)):
            x, y, theta = goal[i]
            pose_msg = Pose()
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.orientation.w = math.cos(math.radians(theta/2))
            pose_msg.orientation.z = math.sin(math.radians(theta/2))
            pose_array_msg.poses.append(pose_msg)
        
        # Publica a mensagem
        pub.publish(pose_array_msg)
        
        rate.sleep()

def calculate_theta(goals):
    angles = []
    for i in range(len(goals)-1):
        x1, y1, theta1 = goals[i]
        x2, y2, theta2 = goals[i+1]
        dx = x2 - x1
        dy = y2 - y1
        angle = math.degrees(math.atan2(dy, dx))
        goals[i][2] = angle
    # Calcula o ângulo relativo para o último ponto
    goals[-1][2] = goals[-2][2]

    return goals
    
if __name__ == '__main__':
    try:
        goals_theta = calculate_theta(goals)
        goals_publisher(goals_theta)

    except rospy.ROSInterruptException:
        pass
