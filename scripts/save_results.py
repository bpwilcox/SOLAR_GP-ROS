#!/usr/bin/env python


import rospy
from test_results import TestResults

def collect_results():

    rospy.init_node('results_node')
    arm = rospy.get_param('~arm', 'right')
    test_topic = 'robot/limb/' + arm + '/endpoint_state'
    cur_topic = 'teleop_pose_next'
    GP_topic = 'solarGP'
    duration = 0.2
    Results = TestResults(test_topic, cur_topic, GP_topic, duration, savefile = 'testbag3.bag')
    Results.run()

    rospy.spin()

if __name__ == '__main__':
    try:
        collect_results()
    except rospy.ROSInterruptException:
        pass