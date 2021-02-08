#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class face_rec_node(object):
    """docstring for ."""

    def __init__(self):
        super(face_rec_node, self).__init__()
        #self.arg = arg
        self.track = False
        self.track_pub = rospy.Publisher('/tiago/vision/track', String, queue_size=10)
        rospy.init_node('face_rec_node', anonymous=True)
        self.rate = rospy.Rate(10)
        rospy.loginfo("Visual component node started")

    def find_object_srv(self, msg):
        rospy.loginfo("Find object service")

    def recognise_person(self, msg):
        rospy.loginfo("Detect/recognise person")

    def track_person(self, msg):
        rospy.loginfo("Publishes the position of a tracked human")
        position = "hello world"
        while not rospy.is_shutdown():
            self.track_pub.publish(position)
            self.rate.sleep()

    def state_machine(self):
        rospy.loginfo("Main state machine for the node")


def main():
    node = face_rec_node()

if __name__ == '__main__':
    main()
