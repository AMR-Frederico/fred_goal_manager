#!/usr/bin/env python3

import rospy
from goal_reached import GoalManager
from goal_provider import GoalProvider

if __name__ == '__main__':
    try:
        goal_provider = GoalProvider()
        gm = GoalManager()
        
        gm.run()
        goal_provider.run()
        
    except rospy.ROSInterruptException:
        pass
