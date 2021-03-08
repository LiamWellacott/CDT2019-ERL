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

from manipulation.srv import GraspObject, GraspObjectResponse #add our grasp action pickuppose 

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
    PICKING = 1
    HOLDING = 2
    PLACING = 3
    PRESENT = 4

class ManipulationServer(object):
    def __init__(self):

        # initialise state variables
        self._reset()


        ####
        ##Service## 

        s = rospy.Service('manipulate_object', GraspObject, self.setState)

        
        ####
        ##Client## 
        self.play_motion_client = SimpleActionClient('/play_motion', PlayMotionAction)
        rospy.loginfo("Waiting for Action Server...")
        self.play_motion_client.wait_for_server()

        self.goal = PlayMotionGoal()
        self.goal.skip_planning = False
        self.goal.priority = 0  # Optional


        self.pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction)
        self.place_as = SimpleActionClient('/place_pose', PickUpPoseAction)



        
        self.req_goal = PickUpPoseGoal()
        self.req_goal.object_pose.header.frame_id = "base_footprint"


        # # Get the cheezits size
        # self.object_height = 0.06
        # self.object_width = 0.158
        # self.object_depth = 0.21


        rospy.loginfo("Ready to manipulate!")



    def play_motion(self, motion_name):
        rospy.loginfo("Sending goal")
        self.goal.motion_name = motion_name
        self.play_motion_client.send_goal(self.goal)

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
        self.pick_as.send_goal_and_wait(self.req_goal)

        #Read the return code of server
        result = self.pick_as.get_result()
        if str(moveit_error_dict[result.error_code]) == "SUCCESS":
            rospy.loginfo("Pick Action finished succesfully")
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
        self.place_as.send_goal_and_wait(self.req_goal)

        #Read the return code of server
        result = self.place_as.get_result()
        if str(moveit_error_dict[result.error_code]) == "SUCCESS":
            rospy.loginfo("Place action finished succesfully")
            
        else:
            rospy.logerr("Failed to place, not trying further")

        #Fold the arm back to a neutral position
        self.tuck_arm()
        
        #Set state to idle
        self._reset()   

        return

    def setPresent(self):
        self._reset()
        self.state= State.PRESENT
        return GraspObjectResponse(True)

    def present(self):
        #extend the arm out
        self.arm_out()

        self.open_hand()

        #Set state to idle
        self._reset()   

        return

    def setState(self,msg):
        #Copy the pose in to the object
        self.req_goal.object_pose.pose = msg.goal_pose

        #If Idle, Grasping, move the state to the Grasping or Holding state respectively
        if self.state <= 1:
            self.state += 1
            

        #IF the current state is that the robot is holding an object
        elif self.state == State.HOLDING:
            #If the message tells the robot to place an object
            if msg.act == 'place':
                self.state = State.PLACING    
            #If the message tells the robot to hold out an object   
            elif msg.act == 'present':
                self.state = State.PRESENT
            else:
                rospy.logerr("No Instruction Passed: Please include whether to Place or Present the held object.")
        
        rospy.loginfo("setState: %i", self.state)
        return GraspObjectResponse(True)

    def step(self):
        # execute behaviour
        if self.state == State.PICKING:
            self.pick()
        elif self.state == State.PLACING:
            self.place()
        elif self.state == State.PRESENT:
            self.present()



if __name__ == '__main__':
    rospy.init_node('manipulation_server')
    ms = ManipulationServer()
    rate = rospy.Rate(1)
     

    while(True):
        ms.step()
        rate.sleep()
  


   

