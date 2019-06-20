#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from navigation_exercise.srv import SetAMCLPose

pub = None
seq = 0


def set_amcl_srv_cb(req):
    global pub
    global seq

    point = req.point
    rad = float(req.radius)

    pw = PoseWithCovarianceStamped()

    pw.header.frame_id = 'map'
    pw.header.stamp = rospy.Time.now()
    pw.header.seq = 0
    pw.pose.pose.position.x = point.x
    pw.pose.pose.position.y = point.y
    pw.pose.pose.position.z = point.z
    pw.pose.pose.orientation.x = 0.0
    pw.pose.pose.orientation.y = 0.0
    pw.pose.pose.orientation.z = 0.0
    pw.pose.pose.orientation.w = 1.0

    pw.pose.covariance = [rad/3.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, rad/3.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 3.0]

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(pw)
        pw.header.seq += 1
        if pw.header.seq > 5:
            break
        rate.sleep()
    return []

def main():
    global pub
    global seq
    rospy.init_node("amcl_helper_node")
    seq = 0
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    service = rospy.Service("/set_amcl_pose", SetAMCLPose, set_amcl_srv_cb)
    rospy.spin()

if __name__=="__main__":
    main()
