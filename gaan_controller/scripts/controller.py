#!/usr/bin/env python

import rospy
from enum import Enum
import copy

from navigation.srv import NavigateTo
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Pose2D, Pose

from tf.transformations import quaternion_from_euler

class State(Enum):
    IDLE = 0
    SUMMONED = 1
    RECEIVE_INSTRUCTION = 2
    FETCHING = 3

once = True # TODO required until NLP services can are available

class Controller(object):

    def __init__(self):
        self.reset()

        # subscribe to RSBB topics
        rospy.Subscriber('roah_rsbb/tablet/call', Empty, self._setSummoned)
        rospy.Subscriber('roah_rsbb/tablet/position', Pose2D, self._setSummonLocation)

        # prepare to call GAAN services
        self.navigate_to = rospy.ServiceProxy('navigate_to', NavigateTo)

        # subscribe to GANN topics

        # initialise semantic map TODO read from a sem map file stored in navigation
        self.sem_map = {}

        self.registerLocation('kitchen_island', -2.4, 1.8, 1.5708)
        self.registerLocation('coffee_table', 1.7, 2.6, 0.0)

    def registerLocation(self, location_tag, x, y, theta):
        self.sem_map[location_tag] = Pose()
        self.sem_map[location_tag].position.x = x
        self.sem_map[location_tag].position.y = y
        q = quaternion_from_euler(0.0, 0.0, theta)
        self.sem_map[location_tag].orientation.z = q[2]
        self.sem_map[location_tag].orientation.w = q[3]

    def reset(self):
        self.state = State.IDLE

        self.job_done = False # flag to indacate when service has finished (action server would improve)

        # fetch variables
        self.fetch_object_location = Pose()
        self.fetch_return_location = Pose()

        # TODO reset arm (or at least check it is ok to move)

        # TODO cancel current requests only relevant for action server things

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

    def navigateAndWait(self, pose):
        self.navigate_to(pose)
        self._wait_for('nav_done')

    ### Callbacks
    def _setSummoned(self, msg):
        # note: since we need to wait for the position of the summon, nothing to do here
        return

    def _setSummonLocation(self, msg):
        # reset state
        self.reset()

        # registered the passed location as where annie is
        self.registerLocation('annie', msg.x, msg.y, msg.theta)

        # move the state machine to trigger movement to annie
        self.state = State.SUMMONED

    def _waitCB(self, msg):
        self.job_done = True 

    ### Main functions
    def summoned(self):
        rospy.loginfo('WHO SUMMONS ME')

        # Send navigation request and wait until arrived
        self.navigateAndWait(self.sem_map['annie'])

        # Request next instruction
        self.state = State.RECEIVE_INSTRUCTION

    def receiveInstruction(self):
        global once

        rospy.loginfo('COMMAND ME ANNIE')

        # TODO call nlp service to ask annie for instruction 
        # for now set the internal state of controller to next phase of scenario one time
        # subsequent calls (such as after the task is complete) will be "requests" for the robot to go idle
        if once:
            # the first time around "nlp" tells us to fetch something
            response_task = 'fetch'
            # 'Get the cracker box from the kitchen island'
            response_object_location = 'kitchen_island'
            # '... and put it on the coffee table'
            response_return_location = 'coffee_table'
            once = False
        else:
            response_task = 'no_task'
            rospy.loginfo("OK, GOING IDLE")
        
        # use response from nlp to determine next action
        if response_task == 'fetch': # TODO format of the response from NLP TBD

            # pose of robot in relation to furniture looked up from the 'semantic map' 
            self.fetch_object_location = self.sem_map[response_object_location]
            self.fetch_return_location = self.sem_map[response_return_location]
            
            self.state = State.FETCHING
    
        elif response_task == 'no_task':

            # annie is satisfied, we can rest and wait for further commands
            self.reset()
    
    def fetching(self):
        
        # Go to object location
        self.navigateAndWait(self.fetch_object_location)

        # object pose estimation TODO
        rospy.loginfo('LOOKING FOR OBJECT ON FURNITURE')

        # announce that the object has been found TODO
        rospy.loginfo('SPEAKING NOW')

        # pick up object TODO
        rospy.loginfo('TIME TO PICK')

        # Go to the fetch destination
        self.navigateAndWait(self.fetch_return_location)
        
        # identify a location to place the object TODO
        rospy.loginfo('FINDING SOMEWHERE TO PUT THE OBJECT ON THE FURNITURE')

        # place the object TODO
        rospy.loginfo('TIME TO PLACE')

        # announce that the object has been placed TODO
        rospy.loginfo('SPEAKING NOW')

        # return to annie
        self.navigateAndWait(self.sem_map['annie'])

        # request further instructions
        self.state = State.RECEIVE_INSTRUCTION

    def step(self):
        if self.state == State.SUMMONED:
            self.summoned()
        elif self.state == State.RECEIVE_INSTRUCTION:
            self.receiveInstruction()
        elif self.state == State.FETCHING:
            self.fetching()

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    con = Controller()

    rate = rospy.Rate(1)
    while(True):
        con.step()
        rate.sleep()