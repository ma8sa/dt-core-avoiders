#!/usr/bin/env python3
# @create-shortcut-for-this

import sys

import duckietown_code_utils as dtu

import rospy
from std_msgs.msg import String

#!/usr/bin/env python
# license removed for brevity


def avoid_obstacle_static():

    #taken from rospy tutorial and easy_node

    pub = rospy.Publisher('chatter', String, queue_size=10)

    rospy.init_node('avoid_obstacle_static', anonymous=True)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    dtu.wrap_script_entry_point(avoid_obstacle_static)