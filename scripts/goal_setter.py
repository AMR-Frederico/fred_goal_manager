import rospy
import math
from geometry_msgs.msg import PoseArray, Pose

GHOST = 1 
LED = 0

# goals = [[2.18, 0.02,GHOST],[4.29, 0.36,LED],[5.8, 1.1,GHOST],[6.58, 2.1,LED]]

# goals = [
#          [2.11,0.23,  GHOST],
#          [3.50,0.49,  GHOST],
#          [5.51,1.02,  GHOST],
#          [7.09,1.45,  GHOST],
#          [9.06,1.54,  GHOST],
#          [11.04,1.36,  GHOST],
#          [13.62, 0.54, GHOST],
#          [15.74,-0.75,  GHOST],
#          [16.7,-1.91, LED],
#          [15.58, -4.19, GHOST],                                                                                                 
#          [14.5,-6.0,  GHOST],
#          [13.14, -8.5, GHOST],
#          [12.0,-10.0, GHOST],
#          [9.97, -11.84, GHOST],
#          [9.00,-12.7, LED],
#          [10.27,-7.22,GHOST], 
#          [5.42, -5.4, LED]]#,[0,0,0]]

goals = [
        # [1.88, 0.94, GHOST],
        [3.69, 1.31, GHOST],
        # [5.70, 1.52, GHOST],
        [7.46, 1.58, GHOST],
        # [9.06, 1.54, GHOST],
        [10.78, 1.39, GHOST],
        [12.64, 1.06, GHOST],
        # [14.65, 0.37, GHOST],
        [15.93, -0.5, GHOST],
        [16.7, -1.91, LED],
        [16.65, -3.06, GHOST],
        [16.18, -4.43, GHOST],
        [15.65, -5.45, GHOST], 
        [15.01, -6.44, GHOST], 
        [14.22, -7.52, GHOST], 
        [13.26, -8.69, GHOST],
        [12.37, -9.68, GHOST],
        [11.29, -10.78, GHOST],
        [10.21, -11.79, GHOST],
        [9.00, -12.7, LED]
]

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
            pose_msg.orientation.w = 1 #math.cos(math.radians(theta/2)) # 1
            # pose_msg.orientation.z = math.sin(math.radians(theta/2)) # comenta
            # print(theta)
            pose_msg.orientation.z = theta
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
        # goals_theta = calculate_theta(goals)
        # goals_publisher(goals_theta)
        goals_publisher(goals)

    except rospy.ROSInterruptException:
        pass
