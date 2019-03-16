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
    def __init__(self, limb,joint_names, topic, filter_coef):
        self.limb = limb
        self.coef = filter_coef
        # self.gripper = gripper
        self.joint_names = joint_names
        self.joint_angles = dict()
        self.nextY = list()
        self.currentY = list()
        rospy.Subscriber('prediction', Arrays, self.pred_callback)
        rospy.Subscriber(topic, JointState, self.curr_callback, queue_size = 10)
        self.k = 25
        self.limit = 0.1
        self.neutral_service = rospy.Service('set_neutral',SetNeutral, self.neutral_callback)
        self.jit_service= rospy.Service('jitter', Jitter, self.jitter_callback)

    def jitter(self, njit, deg):

        Y = np.array(self.currentY).reshape(1,len(self.joint_names))   
        pert = deg * 0.0174533 * np.random.uniform(-1.,1.,(njit, np.size(Y,1)))
        YI = Y + pert

        for y in YI:        
            self.limb.move_to_joint_positions(dict(zip(self.joint_names,y.tolist())))

    def set_neutral(self):
        self.limb.move_to_neutral()
        self.nextY = self.currentY[:]

    def neutral_callback(self, req):
        self.set_neutral()
        return True

    def jitter_callback(self, req):
        self.jitter(req.num_jit, req.degree)
        return True
        
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
                cmd[joint] = self.coef * self.nextY[idx] + (1-self.coef) * cmd[joint]
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

    arm = rospy.get_param('~arm', 'right')
    filter_coef = rospy.get_param('~filter_coef', 0.1)

    limb = baxter_interface.Limb(arm)
    gripper = baxter_interface.Gripper(arm)
    joints = rospy.get_param('~joints', ['s0', 's1', 'e1', 'w1'])
    joint_names = [arm + '_' + joint for joint in joints]    
    y_topic = rospy.get_param('~y_topic', 'robot/joint_states')
    MoveRobot = ControlJoints(limb,joint_names, y_topic, filter_coef)

    rospy.wait_for_service('get_teleop')
    get_teleop = rospy.ServiceProxy('get_teleop', GetTeleop)
    teleop_state = GetTeleopResponse()

    # rospy.wait_for_service('jitter')
    # jitter_pos = rospy.ServiceProxy('jitter', Jitter)

    rospy.wait_for_message('prediction',Arrays)
    print("Ready")
    while not rospy.is_shutdown():
        teleop_state = get_teleop()
        if teleop_state.button6:
            MoveRobot.jitter(5,3)
        if teleop_state.button7:
            MoveRobot.set_neutral()
        if teleop_state.button4:
            gripper.close()
        elif teleop_state.button5:
            gripper.open()
        if teleop_state.button8:
            continue       
        MoveRobot.move_joints()


        # rospy.Rate(10).sleep()
        
if __name__ == '__main__':
    try:
        actuate()
    except rospy.ROSInterruptException:
        pass
