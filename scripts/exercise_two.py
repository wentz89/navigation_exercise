#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs as tg
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from paintHelper import LoggerAPI

tfBuffer = None
listener = None
map_data = None
tf_poses = []

def map_cb(msg):
    global map_data
    map_data = msg


def timer_cb(ev):
    global tfBuffer
    global listener
    # TODO
    # get the pose of the base_link in map coordinates
    # use tfBuffer to get the transform
    # use tf2_geometry_msgs to do transform between coordinate-frames
    # See also: https://wiki.ros.org/tf2_ros
    # See also: https://github.com/ros/geometry2/blob/indigo-devel/tf2_geometry_msgs/src/tf2_geometry_msgs/tf2_geometry_msgs.py

    # Hint: base_link in base_link-frame is (0, 0, 0)
    # Hint: dont forget time_stamps in headers
    # Hint: you will only need between 5 - 8 lines of code ;-)
    point_stamped = PointStamped()
    #
    #
    #
    #

    try:
        #
    except Exception as e:
        #

    tf_poses.append(point_in_map.point)

def srv_cb(req):
    global map_data
    global tf_poses
    logger = LoggerAPI()
    # TODO: call the function of the LoggerAPI to draw the picture


def main():
    global tfBuffer
    global listener
    rospy.init_node("tf_observer")
    map_sub = rospy.Subscriber("/map", OccupancyGrid, map_cb)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    timer = rospy.Timer(rospy.Duration(1.0), timer_cb)
    service = rospy.Service("/map_to_img", Empty , srv_cb)

    rospy.spin()

if __name__=="__main__":
    main()
