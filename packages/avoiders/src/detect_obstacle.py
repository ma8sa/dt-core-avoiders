#!/usr/bin/env python3
# @create-shortcut-for-this

import sys
import math
import numpy as np
import duckietown_code_utils as dtu

import rospy
import message_filters
from std_msgs.msg import String

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose, Point, Vector3, TransformStamped, Transform, Polygon

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


class Avoider(DTROS): #comment here 

    def __init__(self):
        ''' put all the init stuff like states, lane coef , planned path status vairable here '''
        self.stuff = 0

        super(Avoider, self).__init__(node_name="detector_node", node_type=NodeType.PERCEPTION)

        #rospy.init_node("detector")
        # declare variables here

        self.state = 0 #-1 failed, 0 in work , 1 finished
        rate = rospy.Rate(0.5)

        self.OBSTACLE_RADIUS = 5 # in centimeters
        self.LANE_WIDTH = 25.5

        self.OBSTACLE_RADIUS = 5 # in centimeters
        self.LANE_WIDTH = 25.5

        self.pose_d = 0
        self.pose_phi = 0
        self.in_lane = False

        self.obstacle = [0,30] # obstacle in robo frame , units:cm

        
        self.sub_lane_reading = rospy.Subscriber(
            "/agent/lane_filter_node/lane_pose", LanePose, self.get_pose, "lane_filter", queue_size=1 #TODO need to fix hardcoded values
        )

        # subscribe to objects
        #TODO

        self.pub_path = rospy.Publisher(
            "~avoidance_path", Polygon, queue_size=1, dt_topic_type=TopicType.CONTROL
        )


        #self.sub_lane_reading = rospy.Subscriber(
        #    "~lane_pose", LanePose, self.get_pose, "lane_filter", queue_size=1
    
        #subscribe to obstacle , make sure you fix units and axis NOTE
        # if we need to avoid , then publish avoidance path

        # TODO 
        # move this
        if self.in_lane: # check if we have lane message or not
            print(' we have valid lane message ')
            if self.get_to_avoid(self.obstacle,[self.pose_d,self.pose_phi]):
                self.plan(self.obstacle,[self.pose_d,self.pose_phi])

    def get_pose(self, input_pose_msg,source):

        self.pose_d =   input_pose_msg.d
        self.pose_phi = input_pose_msg.phi
        self.in_lane =  input_pose_msg.in_lane
        print("got lane ")
        if self.get_to_avoid(self.obstacle,[self.pose_d,self.pose_phi]):
            print(" making path ")
            self.plan(self.obstacle,[self.pose_d,self.pose_phi])


## ALL OBSTACLE_INFO ASSUMES THETA=0


    def rotate(self,origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.
    
        The angle should be given in radians.
        """
        ox, oy = origin
        px, py = point
    
        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return qx, qy

    
    def plan(self,obstacle_info: np.array, lane_info: np.array):
        """
        Inputs: 
            obstacle_info contains the x and y position of the obstacle in robot frame(assume constant diameter?) 
                [x, y]
            lane_info contains distance from yellow lane (center) and orientation w.r.t lane 
                [d, theta]
        Outputs:
            waypoints: list of waypoints [(x_1,  y_1), (x_2, y_2), ..., (x_n, y_n)] in original robot frame
        """
        dist2obstacle = np.sqrt(obstacle_info[0]**2 + obstacle_info[1]**2)
        
        obstacle_info = self.rotate((0,0), np.array(list(obstacle_info)), lane_info[1])
    
        # Assume obstacle_info[1] > 20
        safe_distance = 10
        end_point = (0,  obstacle_info[1] + 20) # want to end up 20 centimeters past the obstacle, back at the same x coordinate
    
        # middle point
        if obstacle_info[0] < 0:
            if safe_distance + (lane_info[0] + obstacle_info[0]) < 20:
                middle_point = (max(obstacle_info[0] + safe_distance, end_point[0]), obstacle_info[1])
            else:
                middle_point = (obstacle_info[0] - safe_distance, obstacle_info[1])
        else:
            middle_point = (obstacle_info[0] - safe_distance, obstacle_info[1])
    
    
        first_third = (middle_point[0], middle_point[1]/2)
        second_third = (middle_point[0], middle_point[1] + (end_point[1] - middle_point[1])/2)
        
        waypoints = np.array([first_third, second_third, end_point])
        waypoints = [self.rotate((0,0), w, -lane_info[1]) for w in waypoints]

        #TODO 
        #insted of returning publish message directly
        # convert the axis as well as the units
        poly = Polygon()
        print("printing paths")
        for p in waypoints:
            point = Point32()
            point.x = (p[1]/100)
            point.y = (p[0]/100)
            point.z = 0
            print(p)

            poly.points.append(point)

        self.pub_path.publish(poly)
            


        
        return 
    
    def get_to_avoid(self,obstacle_info: np.array, lane_info: np.array):
        """
        Inputs:
            same as plan
        Outpus:
            to_avoid: boolean of whether we need to avoid obstacle
        """
        
        obstacle_info = self.rotate((0,0), np.array(list(obstacle_info)), lane_info[1])
        
        obstacle_2_lane_x = obstacle_info[:, 0] + lane_info[0]
        to_avoid = np.logical_not(np.logical_or(obstacle_2_lane_x < -OBSTACLE_RADIUS, 
                             obstacle_2_lane_x > (LANE_WIDTH + OBSTACLE_RADIUS)))
        
        return to_avoid
    



if __name__ == '__main__':
    A = Avoider()
    rospy.spin()
    #obs_msg = None
    #lane_msg = None
    #dtu.wrap_script_entry_point(A.callback(obs_msg,lane_msg))
