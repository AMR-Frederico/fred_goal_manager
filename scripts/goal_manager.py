#!/usr/bin/env python3

import rospy
from goal_reached import GoalManager
from goal_provider import GoalProvider

if __name__ == '__main__':
        
    rospy.init_node('goal_reached_monitor', anonymous=True)    

    try:
        goal_provider = GoalProvider()
        gm = GoalManager()
        
        goal_provider.run()
        gm.run()
        
    except rospy.ROSInterruptException:
        pass
