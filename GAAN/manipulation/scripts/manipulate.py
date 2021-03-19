#! /usr/bin/env python

import rospy
import time
import numpy as np
import cv2
from cv_bridge import CvBridge
from enum import Enum

from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import PoseStamped, Pose
from actionlib import SimpleActionClient, GoalStatus

from gaan_msgs.srv import Manipulate, ManipulateResponse #add our grasp action pickuppose 

from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name 

class State(Enum):
    IDLE = 0
    HOLDING = 1

class Action(Enum):
    PICK = 'PICK'
    PLACE = 'PLACE'
    PRESENT = 'PRESENT'

class ManipulationServer(object):
    def __init__(self):

        # initialise state variables
        self._reset()

        # manipulation services 
        s = rospy.Service('/gaan/manipulate', Manipulate, self.manipulateHandler)

        # manipulation clients
        self.play_motion_client = SimpleActionClient('/play_motion', PlayMotionAction)
        rospy.loginfo("Waiting for Action Server...")
        self.play_motion_client.wait_for_server()

        self.playmotion_goal = PlayMotionGoal()
        self.playmotion_goal.skip_planning = False
        self.playmotion_goal.priority = 0  # Optional

        self.pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction)
        self.place_as = SimpleActionClient('/place_pose', PickUpPoseAction)
        
        self.pickup_goal = PickUpPoseGoal()
        self.pickup_goal.object_pose.header.frame_id = "base_footprint"

        # end initialisation
        rospy.loginfo("Ready to manipulate!")

    def play_motion(self, motion_name):
        rospy.loginfo("Sending goal")
        self.playmotion_goal.motion_name = motion_name
        self.play_motion_client.send_goal(self.playmotion_goal)

        rospy.loginfo("Waiting for result...")
        action_ok = self.play_motion_client.wait_for_result(rospy.Duration(30.0))

        state = self.play_motion_client.get_state()

        if action_ok:
            rospy.loginfo(motion_name + " succesfully")
        else:
            rospy.logwarn(motion_name + " failed")

    def unfold_arm(self):
        self.play_motion("safe_unfold")

    def tuck_arm(self):
        self.play_motion("home")

    def arm_out(self):
        self.play_motion("arm_out")
    
    def open_hand(self):
        self.play_motion("open_hand")

    def _reset(self):
        self.state = State.IDLE

    def pick(self):

        #Service calls this function after receiving the pose
        #unfold the arm
        self.unfold_arm()

        #Send goal to pick_and_place_server
        self.pick_as.send_goal_and_wait(self.pickup_goal)

        #Read the return code of server
        self.pick_result = self.pick_as.get_result()
        if str(moveit_error_dict[self.pick_result.error_code]) == "SUCCESS":
            rospy.loginfo("Pick Action finished succesfully")

            # robot is now holding an object
            self.state = State.HOLDING
            
        else:
            rospy.logerr("Failed to pick, not trying further")
            #Set state to idle
            self._reset()    

        #Fold the arm back to a neutral position
        self.tuck_arm()


    def place(self):
        #Service calls this function after receiving the pose to place the object
        #unfold the arm
        self.unfold_arm()

        #Send goal to pick_and_place_server
        self.place_as.send_goal_and_wait(self.pickup_goal)

        #Read the return code of server
        self.place_result = self.place_as.get_result()
        if str(moveit_error_dict[self.place_result.error_code]) == "SUCCESS":
            rospy.loginfo("Place action finished succesfully")
            
        else:
            rospy.logerr("Failed to place, not trying further")

        #Fold the arm back to a neutral position
        self.tuck_arm()
        
        #Set state to idle
        self._reset()   

    def present(self):
        #extend the arm out
        self.arm_out()

        # release grip
        self.open_hand()

        #Set state to idle
        self._reset()   

    def manipulateHandler(self,msg):
        
        response = True

        # # check if requested action is valid
        # TODO this threw errors but only from some people...
        # rospy.loginfo(type(Action))
        # actions = [e.value for e in Action]
        # if msg.action not in actions:
        #     response = False
        #     rospy.logerr("Invalid action requested from manipulate, received %s but must be one of: %s" % (msg.action, actions) )  

        # only allow pick requests if the current state is idle (not holding anything)
        if self.state == State.IDLE:
            if msg.action == Action.PICK:
                self.pickup_goal.object_pose.pose = msg.goal_pose
                self.pick()

            else:
                # place or present requested when we don't have an object
                if str(moveit_error_dict[self.pick_result.error_code]) != "SUCCESS":
                    response = False
                    rospy.logerr("%s requested, but we must be holding an object do these actions" % msg.action)

        # only allow place/present if the robot is holding an object
        elif self.state == State.HOLDING:
            if msg.action == Action.PLACE:
                # pickup goal used for both picking and placing in pick_and_place_server
                self.pickup_goal.object_pose.pose = msg.goal_pose
                self.place()

            elif msg.action == Action.PRESENT:
                self.present()

            else:
                if str(moveit_error_dict[self.place_result.error_code]) != "SUCCESS":
                    # pick or invalid action sent
                    response = False 
                    rospy.logerr("%s requested, but robot is already holding an object" % msg.action)
                    
        return ManipulateResponse(response)

if __name__ == '__main__':
    rospy.init_node('manipulation_server')
    ms = ManipulationServer()
    rospy.spin()
  


   

