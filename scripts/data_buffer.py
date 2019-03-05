import rospy
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from bwrobot.srv import Jitter
from copy import deepcopy, copy
from baxter_core_msgs.msg import EndpointState

class DataBuffer():
    def __init__(self, x_topic, y_topic, joint_names, duration, max_buffer_size = float('inf')):
        rospy.Subscriber(x_topic, EndpointState, self.x_callback, queue_size = 10)
        rospy.Subscriber(y_topic, JointState, self.y_callback, queue_size = 10)
        self.x_state = None
        self.y_state = None
        self.joint_angles = dict()
        self.joint_names = joint_names
        self.max_buffer_size = max_buffer_size
        self.Xexp = list()
        self.Yexp = list()
        rospy.Timer(rospy.Duration(duration), self.timer_callback)

    def x_callback(self, data):
        self.x_state = data

    def y_callback(self, data):
        self.y_state = data
    
    def timer_callback(self, event):
        if self.x_state and self.y_state != None:
            self.Xexp.append(self.parse_x(self.x_state))
            self.Yexp.append(self.parse_y(self.y_state))
            if len(self.Xexp) > self.max_buffer_size:
                self.Xexp.pop(0)
                self.Yexp.pop(0)

    def parse_x(self, msg):
        X = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]).reshape(1,3)
        return X

    def parse_y(self, msg):

        for idx, name in enumerate(msg.name):
            if name in self.joint_names:
                self.joint_angles[name] = msg.position[idx]
        
        Y = []
        for joint in self.joint_names:
            Y.append(self.joint_angles[joint])
        
        Y = np.array(Y).reshape(1,len(self.joint_names))
        return Y

    def clear(self):
        del self.Xexp[:]
        del self.Yexp[:]
