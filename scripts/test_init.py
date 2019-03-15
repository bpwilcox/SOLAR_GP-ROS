#!/usr/bin/env python

import rospy
import rosbag
from bwrobot.msg import LocalGP, Errors, Params
from bwrobot.srv import SaveToFile, SetNeutral
from trainer import Solar_Trainer
from test_results import TestResults
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Bool
from teleop_utils.srv import GetTeleop
import time
import numpy as np

class TestStarter():
    def __init__(self, Trainer, DataCollector):
        self.Trainer = Trainer
        self.DataCollector = DataCollector
        rospy.wait_for_service('restart_bag')
        self.restart = rospy.ServiceProxy('restart_bag', Empty)  
        rospy.wait_for_service('set_neutral')
        self.set_neutral = rospy.ServiceProxy('set_neutral', SetNeutral)              
        rospy.Subscriber('trajectory_finished', Bool, self.done_callback, queue_size=10)

    def done_callback(self, msg):
        if msg.data:
            self.stop_trainer()

    def init_trainer(self):
        self.Trainer.initialize()
        self.restart(EmptyRequest())
        rospy.loginfo("Model Initialized")

    def run_trainer(self):
        rospy.loginfo("Running Trainer")
        self.Trainer.stop = False
        self.Trainer.run()

    def run_data_collector(self):
        self.DataCollector.run()

    def stop_trainer(self):
        if not self.Trainer.stop:
            rospy.loginfo("Stopping Trainer")
            self.Trainer.stop = True

    def start_test(self):
        self.init_trainer()
        self.run_data_collector()
        self.run_trainer()

    def run_again(self):
        self.set_neutral()
        self.run_trainer()
        self.run_data_collector()
        self.restart(EmptyRequest())

    def end_test(self):
        rospy.loginfo("Ending Test")
        self.DataCollector.save_data()


def start():
    rospy.init_node('test_node')

    "Model Parameters"
    njit = rospy.get_param('~njit', 25)
    deg = rospy.get_param('~degree', 3)
    num_inducing = rospy.get_param('~num_inducing', 50)
    w_gen = rospy.get_param('~wgen', 0.98)
    param = Params()
    param.njit = njit
    param.inducing = num_inducing
    param.degrees = deg
    param.wgen = w_gen

    "Test Data Collection setup"
    arm = rospy.get_param('~arm', 'left')
    test_topic = 'robot/limb/' + arm + '/endpoint_state'
    cur_topic = 'teleop_pose_next'
    solar_topic = 'solarGP'
    duration = 0.2

    Trainer = Solar_Trainer(njit, deg, num_inducing, w_gen, False)
    DataCollector = TestResults(test_topic, cur_topic, solar_topic, duration, params = param)
    Test = TestStarter(Trainer, DataCollector)
    directory = '/home/bpwilcox/catkin_ws/src/SOLAR_GP-ROS/bags/tests/'

    num_inducing = range(15,50,5)
    w_gen = np.linspace(0.8, 0.985, 5)

    test_num = 1
    redos = 0
    for inducing in num_inducing:
        for thresh in w_gen:
            Test.Trainer.num_inducing = inducing
            Test.Trainer.wgen = thresh
            rospy.loginfo("Starting Test # %s out of %s", test_num, (redos+1)* len(num_inducing)*len(w_gen))
            Test.DataCollector.filename = directory + 'test_'+ time.strftime("%Y%m%d-%H%M%S") + '.bag'
            Test.start_test()
            Test.end_test()
            test_num +=1
            for times in range (0, redos):
                rospy.loginfo("Starting Test # %s out of %s", test_num, len(num_inducing)*len(w_gen))
                Test.DataCollector.filename = directory + 'test_redo_'+ time.strftime("%Y%m%d-%H%M%S") + '.bag'            
                Test.run_again()
                test_num +=1       

    # for test_num in range(0, 3):

    #     rospy.loginfo("Starting Test # %s", test_num)
    #     Test.DataCollector.filename = directory + 'test_'+ time.strftime("%Y%m%d-%H%M%S") + '.bag'
    #     Test.start_test()
    #     Test.end_test()


if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass


