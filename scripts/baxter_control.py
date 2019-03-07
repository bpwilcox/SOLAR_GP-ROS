#!/usr/bin/env python

import rospy
import numpy as np
from bwrobot.msg import Arrays
import baxter_interface
from sensor_msgs.msg import JointState
from math import fabs
import baxter_dataflow
from baxter_interface import settings
from teleop_utils.srv import GetTeleop, GetTeleopResponse, SetPose
from bwrobot.srv import Jitter, SetNeutral, SetNeutralRequest

class ActRobot():
    def __init__(self, limb, joint_names):
        self.limb = limb
        self.joint_names = joint_names
        # self.Y = list()
        rospy.Subscriber('prediction', Arrays, self.callback)
    
    def callback(self, msg):
        # self.Y = msg.array
        self.limb.move_to_joint_positions(dict(zip(self.joint_names, msg.array)))


class ControlJoints():
    def __init__(self, limb,joint_names, topic):
        self.limb = limb
        # self.gripper = gripper
        self.joint_names = joint_names
        self.joint_angles = dict()
        self.nextY = list()
        self.currentY = list()
        rospy.Subscriber('prediction', Arrays, self.pred_callback)
        rospy.Subscriber(topic, JointState, self.curr_callback, queue_size = 10)
        self.k = 25
        self.limit = 0.1

        # self.gripper_state = 1 # Assume gripper starts open

    # def toggle_gripper(self):
    #     if self.gripper_state:
    #         self.gripper.close()
    #         self.gripper_state = 0
    #     else:
    #         self.gripper.open()
    #         self.gripper_state = 1

    def set_neutral(self):
        self.limb.move_to_neutral()
        self.nextY = self.currentY[:]

    def pred_callback(self, msg):
        self.nextY = msg.array

    def curr_callback(self, msg):   
        for idx, name in enumerate(msg.name):
            if name in self.joint_names:
                self.joint_angles[name] = msg.position[idx]
        
        Y = []
        for joint in self.joint_names:
            Y.append(self.joint_angles[joint])
        self.currentY = Y
    
    def get_joint_command(self):
        v = []
        for nY, cY in zip(self.nextY, self.currentY):
            v.append(np.sign(nY-cY)*min(fabs(self.k*(nY-cY)), self.limit))
        return v

    def filtered_cmd(self):
        # First Order Filter - 0.2 Hz Cutoff
        cmd = self.joint_angles
        for idx, joint in enumerate(self.joint_names):
            cmd[joint] = 0.012488 * self.nextY[idx] + 0.98751 * cmd[joint]
        return cmd

    def move_joints(self,timeout=15.0,
                                    threshold=settings.JOINT_ANGLE_TOLERANCE,
                                    test=None):
        """
        (Blocking) Commands the limb to the provided positions.

        Waits until the reported joint state matches that specified.

        This function uses a low-pass filter to smooth the movement.

        @type positions: dict({str:float})
        @param positions: joint_name:angle command
        @type timeout: float
        @param timeout: seconds to wait for move to finish [15]
        @type threshold: float
        @param threshold: position threshold in radians across each joint when
        move is considered successful [0.008726646]
        @param test: optional function returning True if motion must be aborted
        """
        cmd = self.joint_angles

        def filtered_cmd():
            # First Order Filter - 0.2 Hz Cutoff
            cmd = self.joint_angles
            for idx, joint in enumerate(self.joint_names):
                # cmd[joint] = 0.012488 * self.nextY[idx] + 0.98751 * cmd[joint]
                cmd[joint] = 0.3 * self.nextY[idx] + 0.7 * cmd[joint]
            return cmd

        def genf(joint, angle):
            def joint_diff():
                return abs(angle - self.joint_angles[joint])
            return joint_diff

        diffs = [genf(j, a) for j, a in zip(self.joint_names, self.nextY)]

        self.limb.set_joint_positions(filtered_cmd())
        # baxter_dataflow.wait_for(
        #     test=lambda: callable(test) and test() == True or \
        #                 (all(diff() < threshold for diff in diffs)),
        #     timeout=timeout,
        #     timeout_msg=("%s limb failed to reach commanded joint positions"),
        #     rate=100,
        #     raise_on_error=False,
        #     body=lambda: self.limb.set_joint_positions(filtered_cmd())
        #     )

def actuate():

    rospy.init_node('actuator_node')
    limb = baxter_interface.Limb('right')
    gripper = baxter_interface.Gripper('right')
    joint_names = rospy.get_param('~joints', ['right_s0', 'right_s1', 'right_e1', 'right_w1'])
    y_topic = rospy.get_param('~y_topic', 'robot/joint_states')
    # MoveRobot = ActRobot(limb, joint_names)
    MoveRobot = ControlJoints(limb,joint_names, y_topic)
    # limb.set_command_timeout(1)
    limb.set_joint_position_speed(0.3)

    rospy.wait_for_service('get_xbox_teleop')
    get_teleop = rospy.ServiceProxy('get_xbox_teleop', GetTeleop)
    teleop_state = GetTeleopResponse()

    rospy.wait_for_service('jitter')
    jitter_pos = rospy.ServiceProxy('jitter', Jitter)

    rospy.wait_for_message('prediction',Arrays)

    while not rospy.is_shutdown():
        # print('current: ', MoveRobot.currentY)
        # print('target: ', MoveRobot.nextY)
        # print('velocity: ', MoveRobot.get_joint_command())
        # limb.set_joint_velocities(dict(zip(joint_names, MoveRobot.get_joint_command())))
        # limb.set_joint_positions(dict(zip(joint_names, MoveRobot.nextY)))
        # limb.set_joint_positions(MoveRobot.filtered_cmd())
        teleop_state = get_teleop()
        if teleop_state.button6:
            jitter_pos(5,3)
        if teleop_state.button7:
            MoveRobot.set_neutral()
        if teleop_state.button4:
            gripper.close()
        elif teleop_state.button5:
            gripper.open()
        MoveRobot.move_joints()


        # rospy.Rate(10).sleep()
        
if __name__ == '__main__':
    try:
        actuate()
    except rospy.ROSInterruptException:
        pass
