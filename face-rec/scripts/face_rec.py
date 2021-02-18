#!/usr/bin/env python

import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image
import face_recognition as fr
import numpy as np
import cv2
import os
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

class face_rec_node(object):
    """docstring for ."""

    def __init__(self):
        super(face_rec_node, self).__init__()
        #self.arg = arg
        rospack = rospkg.RosPack()
        self.track = False

        rospy.init_node('face_rec_node', anonymous=True)
        self.rate = rospy.Rate(10)

        # list containing known faces
        self.known_enco = []
        self.known_name = []
        self.process_this_frame = True
        dir = rospack.get_path("face-rec")
        self.init_known(os.path.join(dir, "faces", "known"))

        self.track_pub = rospy.Publisher('/tiago/vision/track', String, queue_size=10)
        self.cam_rec_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.recognise_person)
        self.face_rec_pub = rospy.Publisher("/erl/face_rec/faces", Image, queue_size=10)
        self.cam_track_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.track_person)
        self.bridge = CvBridge()

        rospy.loginfo("Visual component node started")

    def get_name(self, filename):
        # Assumes that the filename is /path/to/file/name.extension
        _, name = os.path.split(filename)
        return os.path.splitext(name)[0]

    def init_known(self, face_dir):
        files = [os.path.join(face_dir, f) for f in os.listdir(face_dir) if os.path.isfile(os.path.join(face_dir, f))]
        for face_file in files:
            img = fr.load_image_file(face_file)
            # want a width of 128 to speed up the process, be sure not to run out of memory.
            scale = float(img.shape[1])/640.
            small_frame = cv2.resize(img, (0, 0), fx=1./scale, fy=1./scale)

            self.known_name.append(self.get_name(face_file))
            face_loc = fr.face_locations(small_frame, model='cnn')
            face_encoding = fr.face_encodings(small_frame, face_loc)[0]
            self.known_enco.append(face_encoding)
        rospy.loginfo("known faces learned")

    def find_object_srv(self, msg):
        pass

    def recognise_person(self, msg):
        if self.process_this_frame:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            frame = np.asarray(cv_img)
            scale=2
            small_frame = cv2.resize(frame, (0, 0), fx=1./float(scale), fy=1./float(scale))
            # convert back to RGB, cv uses BGR conventiion.
            small_frame = small_frame[:, :, ::-1]

            face_loc = fr.face_locations(small_frame, model='cnn')
            rospy.loginfo("Face-rec: {} face detected ".format(len(face_loc)))
            face_encodings = fr.face_encodings(small_frame, face_loc)

            face_names = []
            for enc in  face_encodings:
                matches = fr.compare_faces(self.known_enco, enc)
                name = "Unkown"

                face_distances = fr.face_distance(np.array(self.known_enco), np.array(enc))
                best_match_index = np.argmin(face_distances)
                print(best_match_index)

                if matches[best_match_index]:
                    name = self.known_name[best_match_index]

                face_names.append(name)

            render = self.overlay(frame, face_loc, face_names, scale)

            self.render = self.bridge.cv2_to_imgmsg(render)

        self.process_this_frame = not self.process_this_frame

        self.face_rec_pub.publish(self.render)

    def overlay(self, frame, loc, names, scale):
        # Display the results
        for (top, right, bottom, left), name in zip(loc, names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= scale
            right *= scale
            bottom *= scale
            left *= scale

            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
        return frame

    def track_person(self, msg):
        position = "hello world {}".format(rospy.get_time())
        self.track_pub.publish()
        pass

    def state_machine(self):
        if self.track:
            self.track_person()


    def start(self):
        while not rospy.is_shutdown():
            self.state_machine()
            self.rate.sleep()

def main():
    node = face_rec_node()
    node.start()

if __name__ == '__main__':
    main()
