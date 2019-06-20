#!/usr/bin/env python

import rospy
import actionlib
import sys

from std_msgs.msg import String as String
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist as Twist
from exercise_one import drange

class MoveBaseClient:
    def __init__(self):
        self.move_base_goal = None

        self.goal_time = 0.0
        self.succeeded = False
        self.init_actionclient()


    def set_goal(self, goal):
        self.move_base_goal = goal

    def init_actionclient(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('>> SM: waiting for move_base action server')
        self.move_base_client.wait_for_server()
        rospy.loginfo('>> SM: move_base action server available')

    def client_call(self):
            self.move_base_goal.target_pose.header.stamp = rospy.Time.now()
            rospy.loginfo('Sending goal: %s', self.move_base_goal.target_pose.pose)
            self.move_base_client.send_goal(self.move_base_goal)
            rospy.loginfo("Waiting for result")

            if self.move_base_client.wait_for_result(rospy.Duration(60.0)):
                self.result = self.move_base_client.get_result()
                rospy.loginfo("Goal reached")
                self.goal_time = rospy.Time.now()
                self.succeeded = True
            else:
                self.result = self.move_base_client.get_result()
                self.succeeded = True

    def get_info(self):
        return self.result


class StateMachine:
    def __init__(self, rate, mb_cl):
        self.rate = rate
        self.move_base_cl = mb_cl
        self.state = "init"
        self.prev_state = ""
        self.retry_count = 0
        self.params = []

    def run(self):
        while not rospy.is_shutdown():  
            self.rate.sleep()
            if self.state == "init":
                # do stuff you want to do for init
                self.state = "search"
            elif self. state == "search":
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
                if found:
                    self.get_goal(self.dir)
                else:
                    self.state = "failure"

            elif self.state == "drive":
                # TODO
            elif self. state == "failure":
                # TODO error handling
                self.state = "search"

    def get_goal(self):
        # TODO: search new goal you wanne navigate to
        # Use map, laserscan or functions from the other exercises

        goal = MoveBaseGoal() # all values are zeros
        goal.target_pose.header.frame_id = 'map'
        # Example:
        # goal.target_pose.header.frame_id = 'map'
        # goal.target_pose.pose.position.x = float(sys.argv[1])
        # goal.target_pose.pose.position.y = float(sys.argv[2])
        # goal.target_pose.pose.position.z = float(sys.argv[3])
        # goal.target_pose.pose.orientation.x = float(sys.argv[4])
        # goal.target_pose.pose.orientation.y = float(sys.argv[5])
        # goal.target_pose.pose.orientation.z = float(sys.argv[6])
        # goal.target_pose.pose.orientation.w = float(sys.argv[7])
        # Hint: you can send goals in other coordinate frames (it may easier)
        #       for example base_link
        self.goal = goal

    def drive(self):
        self.move_base_cl.set_goal(self.goal)
        self.move_base_cl.client_call() # this function is blocking

## Helper Functions
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

## Helper Functions End

def main():
    rospy.init_node('MoveBaseClient')
    move_base_cl = MoveBaseClient()
    rate = rospy.Rate(10)
    state_machine = StateMachine(rate, move_base_cl)
    state_machine.run()


if __name__ == '__main__':
    main()

