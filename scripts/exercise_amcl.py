#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from navigation_exercise.srv import SetAMCLPose
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

class KidnappedRobotRescue:

    def __init__(self):

        self.service = rospy.ServiceProxy("/set_amcl_pose", SetAMCLPose)
        self.amcl_pose_sub = rospy.Subscriber(
                    "/amcl_pose",
                    PoseWithCovarianceStamped,
                    self.amcl_pose_cb)
        self.zero_tw = Twist()
        self.start_srv = rospy.Service("/localize_robot", Empty, self.start_cb)
        self.enabled = False
        self.sigma = 10
        self.sigma_theta = 10
        self.retry_count = 0
        self.timer = rospy.Timer(rospy.Duration(1.0/10.0), self.run)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def start_cb(self, req):
        self.enabled = True
        return []

    def amcl_pose_cb(self, msg):
        cov = msg.pose.covariance
        sigma_x = cov[0]
        sigma_y = cov[7]
        sigma_xy = cov[1]
        self.sigma_theta = cov[35]
        self.sigma = math.sqrt(sigma_x**2 + 2*abs(sigma_xy) + sigma_y**2)

    def run(self, timer_event):
        if not self.enabled:
            return
        self.enabled = False
        # TODO: determine or set a radius and middle point
        # Hint: It can be pretty simple ;-)
        self.service(# middle point + radius for amcl
        )

        while self.sigma > 0.05 and self.sigma_theta > 0.01:
            #TODO strategie to localize the robot
            # Hint: Use self.turn and self.drive function

        rospy.loginfo("Sucess?")
        self.sigma = 10
        self.sigma_theta = 10



    def drive(self, direction, dist=0.2, vel=0.2):
        time = dist / vel
        twist = Twist()
        twist.linear.x = direction[0] * vel
        twist.linear.y = direction[1] * vel
        t = rospy.Time.now()
        while ((rospy.Time.now() - t).to_sec() < time):
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.005)
        self.cmd_vel_pub.publish(self.zero_tw)

    def turn(self, degree=3.14, speed=0.5):
        time = abs(degree / speed)
        twist = Twist()
        twist.angular.z = speed
        t = rospy.Time.now()
        while ((rospy.Time.now() - t).to_sec() < time):
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.005)
        self.cmd_vel_pub.publish(self.zero_tw)

def main():

    rospy.init_node("amcl_exercise")
    kr_obj = KidnappedRobotRescue()

    rospy.spin()

if __name__=="__main__":
    main()
