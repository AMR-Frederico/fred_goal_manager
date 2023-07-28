#!/usr/bin/env python3

import rospy
from goal_reached import GoalReached
from goal_provider import GoalProvider
from goal_cone_reached import ConeReached
# from path_monitor import PathMonitor

if __name__ == '__main__':
        
    rospy.init_node('goal_manager_node', anonymous=True)    

    try:
        goal_provider = GoalProvider()
        gm = GoalReached()
        cr = ConeReached()
        # path = PathMonitor()
        # print("init path")

        # goal_provider.run()
        # gm.run()
        rospy.loginfo("GOAL_MANAGER: init")
        while not rospy.is_shutdown():
            goal_provider.main()
            gm.goal_reached()
            cr.goal_reached()
            # path.routine()
            # print("routine VAI FRED")


            goal_provider.rate.sleep()

    except rospy.ROSInterruptException:
        pass