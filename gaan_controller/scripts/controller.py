#!/usr/bin/env python

import rospy
from enum import Enum
import copy

from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Pose2D, Pose

from gaan_msgs.srv import Command, CommandResponse, Speech, Faces, NavigateTo

from tf.transformations import quaternion_from_euler

class State(Enum):
    IDLE = 0
    SUMMONED = 1
    RECEIVE_INSTRUCTION = 2
    BRINGING = 3

class Controller(object):

    def __init__(self):
        rospy.loginfo("Initalizing controller")
        self.reset()

        # subscribe to RSBB topics
        rospy.Subscriber('roah_rsbb/tablet/call', Empty, self._setSummoned)
        rospy.Subscriber('roah_rsbb/tablet/position', Pose2D, self._setSummonLocation)

        nlp_service = rospy.Service('/gaan/nlp/command', Command, self._vocal_commandCB)

        # prepare to call GAAN services
        self.navigate_to = rospy.ServiceProxy('/gaan/navigation/navigate_to', NavigateTo)
        self.face_rec = rospy.ServiceProxy('/gaan/face_rec', Faces)
        self.speak = rospy.ServiceProxy('/gaan/nlp/speech', Speech)

        # subscribe to GANN topics

        # initialise semantic map
        self.sem_map = {}

        furniture = rospy.get_param('/semmap/furniture')
        for item in furniture:
            interaction_pose = furniture[item]['interaction_pose']
            self.registerLocation(item, interaction_pose['x'], interaction_pose['y'], interaction_pose['theta'])

        rospy.loginfo("Ready to control!")

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

        # bringing variables
        self.bringing_object_location = Pose()
        self.bringing_return_location = Pose()

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

    def _vocal_commandCB(self, req):

        # use response from nlp to determine next action
        if req.action == 'BRINGING':

            # pose of robot in relation to furniture looked up from the 'semantic map'
            for arg in req.arguments:

                # in bringing action, source is the location of the object to bring
                if arg.arg_name == 'source':
                    self.bringing_object_location = self.sem_map[arg.role_filler]

                # beneficiary or goal refer to where to bring the object
                # if beneficiary used then a hand over should be performed instead of a furniture place TODO
                elif arg.arg_name == 'beneficiary':
                    self.bringing_return_location = self.sem_map[arg.role_filler]

                # theme refers to the object to be brought
                elif arg.arg_name == 'theme':
                    theme = arg.role_filler # TODO may be an input to vision in future

            # update state of controller to begin task
            self.state = State.BRINGING

        else:

            # non supported argument for action
            # annie is satisfied, we can rest until we receive a new task
            self.reset()

        return CommandResponse("Done")

    ### Main functions
    def summoned(self):
        rospy.loginfo('WHO SUMMONS ME')

        # Send navigation request and wait until arrived
        self.navigateAndWait(self.sem_map['annie'])

        # Request next instruction
        self.state = State.RECEIVE_INSTRUCTION

    def receiveInstruction(self):

        # send a greeting to initiate conversation
        self.speak("Hello, Granny Annie, How can I help you?")

        # remain in this state until a command is received from the user (state update via _vocal_commandCB)
        
    def get_faces(self):
        rospy.wait_for_service('/gaan/face_rec')
        try:
            resp = self.face_rec()

            return resp.names

        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
            return [""]

    def bringing(self):

        # Go to object location
        self.navigateAndWait(self.bringing_object_location)

        # object pose estimation TODO
        rospy.loginfo('LOOKING FOR OBJECT ON FURNITURE')

        # announce that the object has been found
        self.speak('I have found the cracker box')

        # pick up object TODO
        rospy.loginfo('TIME TO PICK')

        # Go to the fetch destination
        self.navigateAndWait(self.bringing_return_location)

        # identify a location to place the object TODO
        rospy.loginfo('FINDING SOMEWHERE TO PUT THE OBJECT ON THE FURNITURE')

        # place the object TODO
        rospy.loginfo('TIME TO PLACE')

        # announce that the object has been placed
        self.speak('Here is the cracker box')

        # return to annie
        self.navigateAndWait(self.sem_map['annie'])

        # request further instructions
        self.state = State.RECEIVE_INSTRUCTION

    def step(self):
        if self.state == State.SUMMONED:
            self.summoned()
        elif self.state == State.RECEIVE_INSTRUCTION:
            self.receiveInstruction()
        elif self.state == State.BRINGING:
            self.bringing()

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    con = Controller()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        con.step()
        rate.sleep()
