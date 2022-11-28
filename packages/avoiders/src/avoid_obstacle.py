#!/usr/bin/env python3
# @create-shortcut-for-this

import sys

import duckietown_code_utils as dtu

import rospy
import message_filters
from std_msgs.msg import String

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose, Point, Vector3, TransformStamped, Transform

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped

from tf2_ros import TransformBroadcaster

from tf import transformations as tr
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

        super(Avoider, self).__init__(node_name="lane_controller_node", node_type=NodeType.PERCEPTION)

        #rospy.init_node("avoider")

        self.state = 0 #-1 failed, 0 in work , 1 finished
        rate = rospy.Rate(0.5)


        self.pub_motor = rospy.Publisher(
            "~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        self.left_encoder_last = None
        self.right_encoder_last = None
        self.encoders_timestamp_last = None
        self.encoders_timestamp_last_local = None
        self.timestamp = None

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.q = [0.0, 0.0, 0.0, 1.0]
        self.tv = 0.0
        self.rv = 0.0

        self.ticks_per_meter = 656.0
        self.debug = False
        self.wierd = True
        self.wheelbase = 0.1
        #ROS
        #TODO : 1) get the subscirbers working 
        #TODO : 1.1) Test this using fake data
        
        #Theory
        #TODO : 2) given an obstacle plan an trajectory
        #TODO : 3) execute the trajectory

        #self.pub_motor = rospy.Publisher() # define the publisher here, in avoider node case it will the motor control and status message
        #self.pub_status = rospy.Publisher() # define the publisher here, in avoider node case it will the motor control and status message

        #obstacle_sub = message_filters.Subscriber('image', Image)
        #lane_sub = message_filters.Subscriber('camera_info', CameraInfo)
    
        #ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
        #ts.registerCallback(self.callback)
        
        # subscirber
        self.sub_encoder_left = message_filters.Subscriber("/agent/left_wheel_encoder_node/tick", WheelEncoderStamped)
        self.sub_encoder_right = message_filters.Subscriber("/agent/right_wheel_encoder_node/tick", WheelEncoderStamped)

        self.ts_encoders = message_filters.ApproximateTimeSynchronizer(
            [self.sub_encoder_left, self.sub_encoder_right], 10, 0.5
        )

        self.ts_encoders.registerCallback(self.cb_ts_encoders)
        #self.callback(0,0)


    def cb_ts_encoders(self, left_encoder, right_encoder):
        timestamp_now = rospy.get_time()

        # Use the average of the two encoder times as the timestamp
        left_encoder_timestamp = left_encoder.header.stamp.to_sec()
        right_encoder_timestamp = right_encoder.header.stamp.to_sec()
        timestamp = (left_encoder_timestamp + right_encoder_timestamp) / 2

        if not self.left_encoder_last:
            self.left_encoder_last = left_encoder
            self.right_encoder_last = right_encoder
            self.encoders_timestamp_last = timestamp
            self.encoders_timestamp_last_local = timestamp_now
            return

        # Skip this message if the time synchronizer gave us an older message
        dtl = left_encoder.header.stamp - self.left_encoder_last.header.stamp
        dtr = right_encoder.header.stamp - self.right_encoder_last.header.stamp
        if dtl.to_sec() < 0 or dtr.to_sec() < 0:
            self.loginfo("Ignoring stale encoder message")
            return

        left_dticks = left_encoder.data - self.left_encoder_last.data
        right_dticks = right_encoder.data - self.right_encoder_last.data

        left_distance = left_dticks * 1.0 / self.ticks_per_meter
        right_distance = right_dticks * 1.0 / self.ticks_per_meter

        # Displacement in body-relative x-direction
        distance = (left_distance + right_distance) / 2

        # Change in heading
        dyaw = (right_distance - left_distance) / self.wheelbase

        dt = timestamp - self.encoders_timestamp_last

        if dt < 1e-6:
            self.logwarn("Time since last encoder message (%f) is too small. Ignoring" % dt)
            return

        self.tv = distance / dt
        self.rv = dyaw / dt

        if self.debug:
            self.loginfo(
                "Left wheel:\t Time = %.4f\t Ticks = %d\t Distance = %.4f m"
                % (left_encoder.header.stamp.to_sec(), left_encoder.data, left_distance)
            )

            self.loginfo(
                "Right wheel:\t Time = %.4f\t Ticks = %d\t Distance = %.4f m"
                % (right_encoder.header.stamp.to_sec(), right_encoder.data, right_distance)
            )

            self.loginfo(
                "TV = %.2f m/s\t RV = %.2f deg/s\t DT = %.4f" % (self.tv, self.rv * 180 / math.pi, dt)
            )

        dist = self.tv * dt
        dyaw = self.rv * dt

        self.yaw = self.angle_clamp(self.yaw + dyaw)
        self.x = self.x + dist * math.cos(self.yaw)
        self.y = self.y + dist * math.sin(self.yaw)
        self.q = tr.quaternion_from_euler(0, 0, self.yaw)
        self.timestamp = timestamp

        self.left_encoder_last = left_encoder
        self.right_encoder_last = right_encoder
        self.encoders_timestamp_last = timestamp
        self.encoders_timestamp_last_local = timestamp_now

        if self.wierd:
            start_time = rospy.Time.now()
            
            message_count = [0,0,1,0,-1,0]
            m_len = len(message_count)

            self.iter_ = 0
            self.state = 0

            while(self.state == 0):

                car_control_msg = Twist2DStamped()
                car_control_msg.header.stamp = rospy.Time.now()
                car_control_msg.header.seq = 0

        # Add commands to car message
                car_control_msg.v = 0.3 
                car_control_msg.omega = message_count[ iter_ % m_len ]
            #self.pub_motor.pub()
                self.pub_motor.publish(car_control_msg)
                print(self.x,self.y,self.z)
                cur_time = rospy.Time.now()
                if cur_time.secs - start_time.secs > 1:
                    start_time = rospy.Time.now()
                    iter_ += 1
                    self.state = 1

    def callback(self,obstacles,lane):

            ''' do planning stuff here '''
            path = self.path_plan(obstacles,lane)

            rospy.loginfo( " in the node printing yeaaaah")


            ''' in a loop, while follow that by publishing motor state '''
            
            start_time = rospy.Time.now()
            
            message_count = [0,0,1,0,-1,0]
            m_len = len(message_count)

            self.iter_ = 0

            while(self.state == 0):

                car_control_msg = Twist2DStamped()
                car_control_msg.header.stamp = rospy.Time.now()
                car_control_msg.header.seq = 0

        # Add commands to car message
                car_control_msg.v = 0.3 
                car_control_msg.omega = message_count[ iter_ % m_len ]
            #self.pub_motor.pub()
                self.pub_motor.publish(car_control_msg)
                print(self.x,self.y,self.z)
                cur_time = rospy.Time.now()
                if cur_time.secs - start_time.secs > 1:
                    start_time = rospy.Time.now()
                    iter_ += 1

            

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
