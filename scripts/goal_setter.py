# /user/bin/env python3

#read a hardcode list
#calculate angles 
#publish each goal 


import rospy
from geometry_msgs.msg import Pose2D
import math



goals = [[0,0,0],[3,3,0],[-3,3,0],[3,-3,0],[-3,-3,0]]

def goals_publisher(goal):
    # Inicializa o nó do ROS
    rospy.init_node('goal_setter_node')
    
    # Cria os publicadores para cada tópico
    pub_list = []
    for i in range(len(goals)):
        pub = rospy.Publisher('goal_manager/goal/' + str(i), Pose2D, queue_size=10)
        pub_list.append(pub)

    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        # Publica as coordenadas em cada tópico
        for i in range(len(goal)):
            x, y,theta = goal[i]
            pose_msg = Pose2D()
            pose_msg.x = x
            pose_msg.y = y
            pose_msg.theta = theta
            pub_list[i].publish(pose_msg)
        
        rate.sleep()

def calculate_theta(goals):
    angles = []
    for i in range(len(goals)-1):
        x1, y1,theta1 = goals[i]
        x2, y2,theta2 = goals[i+1]
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
   
   
  
    
   

