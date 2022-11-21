#!/usr/bin/env python3
# @create-shortcut-for-this

import sys

import duckietown_code_utils as dtu

import rospy
from std_msgs.msg import String

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import (
    Twist2DStamped,
    LanePose,
    WheelsCmdStamped,
    BoolStamped,
    FSMState,
    StopLineReading,
)
#!/usr/bin/env python
# license removed for brevity


class Avoider(DTROS):

    def __init__(self):
        ''' put all the init stuff like states, lane coef , planned path status vairable here '''
        self.stuff = 0

        hello_str = "hello world 1" 
        rospy.loginfo(hello_str)
        super(Avoider, self).__init__(node_name="avoider", node_type=NodeType.PERCEPTION)

        #rospy.init_node("avoider")

        hello_str = "hello world 2%s" % rospy.get_time()
        rospy.loginfo(hello_str)

        self.pub_motor = rospy.Publisher(
            "~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )
        hello_str = "hello world 3%s" % rospy.get_time()
        rospy.loginfo(hello_str)
        #self.pub_motor = rospy.Publisher() # define the publisher here, in avoider node case it will the motor control and status message
        #self.pub_status = rospy.Publisher() # define the publisher here, in avoider node case it will the motor control and status message

        #obstacle_sub = message_filters.Subscriber('image', Image)
        #lane_sub = message_filters.Subscriber('camera_info', CameraInfo)
    
        #ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
        #ts.registerCallback(self.callback)

        self.callback(0,0)


    def callback(self,obstacles,lane):

            ''' do planning stuff here '''
            path = self.path_plan(obstacles,lane)

            rospy.loginfo( " in the node printing yeaaaah")


            ''' in a loop, while follow that by publishing motor state '''
            while(1):
                car_control_msg = Twist2DStamped()
                car_control_msg.header.stamp = rospy.Time.now()
                car_control_msg.header.seq = 0

        # Add commands to car message
                car_control_msg.v = 2.0
                car_control_msg.omega = 0
            #self.pub_motor.pub()
                self.pub_motor.publish(car_control_msg)

                hello_str = "hello world 6%s" % rospy.get_time()
                rospy.loginfo(hello_str)
            

    def path_plan(self,obstacle,lane):
            return 0

    



        

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
    A = Avoider()
    rospy.spin()
    #obs_msg = None
    #lane_msg = None
    #dtu.wrap_script_entry_point(A.callback(obs_msg,lane_msg))
