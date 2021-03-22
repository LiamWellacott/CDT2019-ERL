#!/usr/bin/env python

import rospy
import math

import std_srvs.srv
import std_msgs.msg
from geometry_msgs.msg import Pose2D, Pose
from gazebo_msgs.msg import ModelStates

# Benchmarks
RESTRICTED_TASK_3 = "restricted_task_3"

class RSBB():

    def __init__(self):

        # get requested benchmark
        self.benchmark = rospy.get_param('~benchmark')

        # topics
        self.topic_end_execute = '/roah_rsbb/end_execute'

        # publishers
        self.call = rospy.Publisher('/roah_rsbb/tablet/call', std_msgs.msg.Empty, queue_size=10)
        self.position = rospy.Publisher('/roah_rsbb/tablet/position', Pose2D, queue_size=10)
        self.score = rospy.Publisher('/fake_rsbb/score', std_msgs.msg.Int8, queue_size=10)

        # subscribe to model states
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_statesCB)

        # run requested benchmark
        if self.benchmark == RESTRICTED_TASK_3:
            self.restricted_task_3()
            

    def model_statesCB(self, msg):

        for i, name in enumerate(msg.name):
            if name == 'cheezit_box':
                cheezit_index = i

        self.box_pose = msg.pose[cheezit_index]

    def dist(self, poseA, poseB):
        xdiff = (poseA.position.x  - poseB.position.x) ** 2
        ydiff = (poseA.position.y  - poseB.position.y) ** 2
        zdiff = (poseA.position.z  - poseB.position.z) ** 2
        return math.sqrt(xdiff + ydiff + zdiff)

    ### BENCHMARK START FUNCTIONS
    def restricted_task_3(self):

        # start the task 
        self.call.publish()
        x = rospy.get_param('~x')
        y = rospy.get_param('~y')
        theta = rospy.get_param('~theta')

        rospy.sleep(1) # all hail the magic sleep of making the next publish work, deny its power at your own peril
        self.position.publish(Pose2D(x, y, theta))

        # listen for end execute for generating score
        rospy.Service(self.topic_end_execute, std_srvs.srv.Empty, self.handleEE_restricted_task_3)

    ### BENCHMARK END FUNCTIONS
    def handleEE_restricted_task_3(self, request):

        score = 0

        # one point if the box is no longer on the table
        expected_pose = Pose()
        expected_pose.position.x = rospy.get_param('~box_initial_x')
        expected_pose.position.y = rospy.get_param('~box_initial_y')
        expected_pose.position.z = rospy.get_param('~box_initial_z')
        if self.dist(self.box_pose, expected_pose) > 1.0:
            # box is far away from initial pose
            score += 1

        # one point if the box is at the correct end position
        expected_pose.position.x = rospy.get_param('~box_final_x')
        expected_pose.position.y = rospy.get_param('~box_final_y')
        expected_pose.position.z = rospy.get_param('~box_final_z')
        if self.dist(self.box_pose, expected_pose) < 1.0:
            # box is close to final pose
            score += 1

        # report score
        rospy.loginfo("score: %d" % score)
        self.score.publish(score)

        return std_srvs.srv.EmptyResponse()

if __name__ == "__main__":
    rospy.init_node('rsbb')
    rsbb = RSBB()
    rospy.spin()
