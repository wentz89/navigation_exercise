#!/usr/bin/env python

import rospy
import decimal
from navigation_exercise.srv import FreeSpace
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

def drange(x, y, jump):
    y += 0.1
    if x < y:
        while x <= y:
            yield float(x)
            x += float(decimal.Decimal(jump))
            x = round(x,2)
    else:
        while x >= y:
            yield float(x)
            x += float(decimal.Decimal(jump))
            x = round(x,2)

class Statemachine:

    def __init__(self):
        self.state = "init"
        self.dir_change_count = 0
        self.last_dir = "front"
        self.zero_tw = Twist()
        self.dir = []
        self.rate = rospy.Rate(0.2)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.wait_for_service("/check_free_space")
        try:
            self.srv = rospy.ServiceProxy("/check_free_space", FreeSpace)
        except Exception as e:
            print e
            return None

    def start(self):
        self.timer = rospy.Timer(rospy.Duration(1.0), self.run, oneshot=True)

    def run(self, event):
        while not rospy.is_shutdown():
            # TODO: implement some logic to make it work
            # The main logic should be implemented in state "search"
            # use some variables like: dir_change_count, last_dir, dir, etc.
            # to mak it work
            # the basic logic that is already implement will cause problems
            # cause its pretty dump ;-)
            # Hint: if there are no problems its pretty normal to alternate between
            # states search and drive 
            if self.state == "init":
                self.state = "search"
            elif self.state == "search":
                found = False
                if self.check_front() and not found:
                    found = True
                elif self.check_left() and not found:
                    found = True
                elif self.check_back() and not found:
                    found = True
                elif self.check_right() and not found:
                    found = True

                self.state = "drive"
                if not found:
                    self.state = "failure"

            elif self.state == "drive":
                self.drive(self.dir, 0.3)
                self.state = "search"
            elif self.state == "failure":
                # TODO
                pass


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


    def check_front(self, dist=0.3):
        '''
            * * *
            - - -
            _____
            |   |
        '''
        sp, mp, ep = Point(), Point(), Point()
        sp.y = -0.2
        mp.y = 0.0
        ep.y = 0.2

        for l in list(drange(0.36, 0.36+dist, '0.05')):
            sp.x, mp.x, ep.x = l, l, l
            res1 = self.srv(sp)
            if res1.ans == 0:
                return False

            res1 = self.srv(mp)
            if res1.ans == 0:
                return False

            res1 = self.srv(ep)
            if res1.ans == 0:
                return False
        self.dir = [1, 0]
        return True


    def check_back(self, dist=0.3):
        '''
            |___|
            * * *
            * * *
        '''
        sp, mp, ep = Point(), Point(), Point()
        sp.y = -0.2
        mp.y = 0.0
        ep.y = 0.2

        for l in list(drange(-0.36, -0.36-dist, '-0.05')):
            sp.x, mp.x, ep.x = l, l, l
            res1 = self.srv(sp)
            if res1.ans == 0:
                return False

            res1 = self.srv(mp)
            if res1.ans == 0:
                return False

            res1 = self.srv(ep)
            if res1.ans == 0:
                return False
        self.dir = [-1, 0]
        return True


    def check_left(self, dist=0.3):
        '''
                __
            *  |
               |
            *  |
               |
            *  |__
        '''
        sp, mp, ep = Point(), Point(), Point()
        sp.x = -0.36
        mp.x = 0.0
        ep.x = 0.36

        for l in list(drange(0.2, 0.2+dist, '0.05')):
            sp.y, mp.y, ep.y = l, l, l
            res1 = self.srv(sp)
            if res1.ans == 0:
                return False

            res1 = self.srv(mp)
            if res1.ans == 0:
                return False

            res1 = self.srv(ep)
            if res1.ans == 0:
                return False
        self.dir = [0, 1]
        return True


    def check_right(self, dist=0.3):
        '''
            __
              |  *
              |
              |  *
              |
            __|  *
        '''
        sp, mp, ep = Point(), Point(), Point()
        sp.x = -0.36
        mp.x = 0.0
        ep.x = 0.36

        for l in list(drange(-0.2, -0.2-dist, '-0.05')):
            sp.y, mp.y, ep.y = l, l, l
            res1 = self.srv(sp)
            if res1.ans == 0:
                return False

            res1 = self.srv(mp)
            if res1.ans == 0:
                return False

            res1 = self.srv(ep)
            if res1.ans == 0:
                return False
        self.dir = [0, -1]
        return True

def main():
    rospy.init_node("Exploration")
    sm = Statemachine()
    if not sm:
        return
    sm.start()
    while not rospy.is_shutdown():
        sm.rate.sleep()

if __name__=="__main__":
    main()


