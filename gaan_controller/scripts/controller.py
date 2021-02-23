#!/usr/bin/env python

import rospy

from navigation.srv import NavigateTo
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Pose2D, Pose

from tf.transformations import quaternion_from_euler

from enum import Enum

class State(Enum):
    IDLE = 0
    SUMMONED = 1
    RECEIVE_INSTRUCTION = 2
    FETCHING = 3
    
class Controller(object):

    def __init__(self):
        self.reset()

        # subscribe to RSBB topics
        rospy.Subscriber('roah_rsbb/tablet/call', Empty, self._setSummoned)
        rospy.Subscriber('roah_rsbb/tablet/position', Pose2D, self._setSummonLocation)

        # prepare to call GAAN services
        self.navigate_to = rospy.ServiceProxy('navigate_to', NavigateTo)

        # subscribe to GANN topics

    def reset(self):
        self.state = State.IDLE

        self.job_done = False # flag to indacate when service has finished (action server would improve)

        self.goal_pose = Pose()

        # TODO reset arm

        # TODO cancel current requests

    def _wait_for(self, topic):
        # Note: making the underlying services an action server would make this not required, instead can use the wait provided by that interface
        # listen to confirmation topic
        sub = rospy.Subscriber(topic, Bool, self._waitCB)

        # loop until received
        rate = rospy.Rate(1)
        while not self.job_done:
            rate.sleep()

        # reset waiting variables
        sub.unregister()
        self.job_done = False

    ### Callbacks
    def _setSummoned(self, msg):
        # note: since we need to wait for the position of the summon, nothing to do here
        return

    def _setSummonLocation(self, msg):
        self.goal_pose.position.x = msg.x
        self.goal_pose.position.y = msg.y

        q = quaternion_from_euler(0.0, 0.0, msg.theta)
        self.goal_pose.orientation.x = q[0]
        self.goal_pose.orientation.y = q[1]
        self.goal_pose.orientation.z = q[2]
        self.goal_pose.orientation.w = q[3]

        self.state = State.SUMMONED

    def _waitCB(self, msg):
        self.job_done = True 

    ### Main functions
    def summoned(self):
        rospy.loginfo("WHO SUMMONS ME")

        # Send navigation request and wait until arrived
        rospy.loginfo(self.goal_pose.orientation)
        self.navigate_to(self.goal_pose)
        self._wait_for('nav_done')

        # Request next instruction
        self.state = State.RECEIVE_INSTRUCTION

    def receiveInstruction(self):
        rospy.loginfo("COMMAND ME ANNIE")

        # TODO conversation logic
        
        return #TODO 

    def fetching(self):
        return #TODO

    def step(self):
        if self.state == State.SUMMONED:
            self.summoned()
        elif self.state == State.RECEIVE_INSTRUCTION:
            self.receiveInstruction()
        elif self.state == State.FETCHING:
            self.fetching()

if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    con = Controller()

    rate = rospy.Rate(1)
    while(True):
        con.step()
        rate.sleep()