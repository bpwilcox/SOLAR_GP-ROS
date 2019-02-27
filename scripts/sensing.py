#!/usr/bin/env python

import rospy
# import math
import tf
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy
from bwrobot.srv import *
from bwrobot.msg import *

def poser():
   
    rospy.init_node('sensor_node')
	
    listener = tf.TransformListener()  # This will listen to the tf data later

    pub = rospy.Publisher('experience', Point, queue_size=10)
    rospy.init_node('sensor_node')
	
    R = rospy.get_param('~tf_ee_pub_rate')
    base_frame = rospy.get_param('~tf_base_frame', 'base')
    final_frame = rospy.get_param('~tf_final_frame', 'endEffector')
    rate = rospy.Rate(R)
 
    while not rospy.is_shutdown():


        try:
            now = rospy.Time.now()
#            (trans,rot) = listener.waitForTransform('base', 'endEffector', now, rospy.Duration(10.0))    
#            (trans, rot) = listener.lookupTransform('base', 'endEffector', now)
#            (trans, rot) = listener.lookupTransform('base', 'endEffector', rospy.Time(0))
            (trans, rot) = listener.lookupTransform(base_frame, final_frame, rospy.Time(0))  
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        newpoint = Point()
        newpoint.x = trans[0]
        newpoint.y = trans[1]
        newpoint.z = trans[2]
        pub.publish(newpoint)

        rate.sleep()


if __name__ == '__main__':
    try:
        poser()
    except rospy.ROSInterruptException:
        pass

