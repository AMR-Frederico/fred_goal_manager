#!/usr/bin/env python3

import rospy
from goal_reached import GoalManager
from goal_provider import GoalProvider
from path_monitor import PathMonitor

if __name__ == '__main__':
        
    rospy.init_node('goal_manager_node', anonymous=True)    

    try:
        goal_provider = GoalProvider()
        # gm = GoalManager()
        path = PathMonitor()

        goal_provider.run()
        # gm.run()
        while not rospy.is_shutdown():
            goal_provider.main()
            # gm.goal_reached()
            path.routine()

            goal_provider.rate.sleep()

    except rospy.ROSInterruptException:
        pass