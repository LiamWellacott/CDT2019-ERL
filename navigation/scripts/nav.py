#!/usr/bin/env python

import rospy
import math

from actionlib import SimpleActionClient

from navigation.srv import NavigateTo, NavigateToResponse

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from std_msgs.msg import Bool

from enum import Enum

class State(Enum):
    IDLE = 0
    EXPLORING = 1
    NAVIGATING = 2
    FOLLOWING = 3
    GUIDING = 4

class Navigation:

    def __init__(self):

        # initialise state variables
        self._reset()

        # services
        self.robot_pose = Pose()
        s = rospy.Service('navigate_to', NavigateTo, self.setNavigate)

        # pub/sub
        self.nav_done_pub = rospy.Publisher('nav_done', Bool, queue_size=10)

        rospy.Subscriber('robot_pose', PoseWithCovarianceStamped, self._pose_callback)

        # NavigateTo
        self.move_base_msg = MoveBaseGoal()
        self.move_base_msg.target_pose.header.frame_id = 'map'

        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        rospy.loginfo("Ready to navigate")

    def _pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def _reset(self):
        self.state = State.IDLE
        self.goal_pose = None
        
        self.move_base_request_sent = False

    ### service functions
    def setExplore(self):
        self.state = State.EXPLORING

    def setNavigate(self, msg):
        self._reset()
        self.state = State.NAVIGATING
        self.goal_pose = msg.pose
        return NavigateToResponse(True)

    def setFollow(self):
        self.state = State.FOLLOWING

    def setGuide(self):
        self.state = State.GUIDING

    ### state based logic
    def explore(self):
        return

    def navigate(self):

        if not self.move_base_request_sent:
            # create a request to move base
            self.move_base_msg.target_pose.header.stamp = rospy.Time.now()
            self.move_base_msg.target_pose.pose = self.goal_pose

            self.move_base_client.send_goal(self.move_base_msg)

            rospy.loginfo('Request sent, waiting for planner to move')
            self.move_base_request_sent = True

            # Note: I tried using callbacks in send_goal, and using wait_for_response, neither worked and caused the module to wait forever. 
            # The only active topic in the move_base package seems to be status which can be continously monitored until it reports that the goal has been reached.
            # This is not a good solution, but since the correct mechanisms are broken I don't know what else to do.
            # Annoyingly something is printing "GOAL Reached!" so there should be a notification of success... I just don't know where it is...
            # Workaround for now is to reimplement the distance check here which is not good.

            # self.move_base_client.wait_for_result()

            # rospy.loginfo(self.move_base_client.get_result)

        else:
            # Wait for path to be completed
            # Note hacky solution to above comment
            xdiff = (self.goal_pose.position.x  - self.robot_pose.position.x) ** 2
            ydiff = (self.goal_pose.position.y  - self.robot_pose.position.y) ** 2
            dist = math.sqrt(xdiff + ydiff)
        
            if dist < 0.2: # 0.2 value comes from YAML file for the local planner and is the xy_goal_tollerance     

                # notify system of success/failure
                # failure will only be available if the interface with move_base is fixed
                self.nav_done_pub.publish(True)
                rospy.loginfo("Nav done!")

                # reset nav state machine
                self._reset()

    def follow(self):
        return

    def guide(self):
        return

    def step(self):

        # check collisions TODO

        # execute behaviour
        if self.state == State.EXPLORING:
            self.explore()
        elif self.state == State.NAVIGATING:
            self.navigate()
        elif self.state == State.FOLLOWING:
            self.follow()
        elif self.state == State.GUIDING:
            self.guide()

def main():

    rospy.init_node('navigation', anonymous=True)
    rate = rospy.Rate(1)
    nav = Navigation() 

    while(True):

        nav.step()
        rate.sleep()

if __name__ == "__main__":
    main()


