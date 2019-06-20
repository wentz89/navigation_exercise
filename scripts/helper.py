#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from navigation_exercise.srv import FreeSpace
import tf2_ros
import tf2_geometry_msgs as tg

map_data = None
got_map = False
tfBuffer = None
listener = None

def map_cb(msg):
    global map_data
    global got_map
    map_data = msg
    got_map = True

def free_space_srv_cb(req):

    global map_data
    global got_map
    global tfBuffer

    point = req.point
    point_stamped = PointStamped()

    # TODO: Fill in the infos of point_stamped
    # Dont forget the header
    # call the function tfBuffer.lookup_transform with the right params to 
    # get the transformation t


    try:
        point_in_map = tg.do_transform_point(point_stamped, t)
    except Exception as e:
        print e
        return -1
    while not got_map:
        rospy.sleep(0.1)


    ans = point_is_free(map_data,point_in_map)
    if ans >= 80:
        return 0
    elif ans == -1:
        return -1
    else:
        return 1

def point_is_free(map_data, point_in_map):
    info = map_data.info
    data = map_data.data
    resolution = info.resolution
    width = info.width
    height = info.height
    orig = info.origin.position
    point = point_in_map.point

    # TODO
    # calculate x_ind, y_ind
    # dont forget use origin (orig.x, orig.y) in calculation
    # Pseudo: # X = ....
    # Pseudo: ind = X / resolution
    # ind = ....
    return data[ind]


def main():
    global tfBuffer
    global listener
    rospy.init_node("navigation_helper_node")
    map_sub = rospy.Subscriber("/map", OccupancyGrid, map_cb)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    service = rospy.Service("/check_free_space", FreeSpace, free_space_srv_cb)
    rospy.spin()

if __name__=="__main__":
    main()
