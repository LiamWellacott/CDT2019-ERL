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
from gaan_msgs.srv import Faces, FacesResponse

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
        self.process_frame = 2
        self.frame=None

        dir = rospack.get_path("gaan_face_rec")
        self.init_known(os.path.join(dir, "faces", "known"))



        self.cam_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.save_last)
        self.face_rec_srv = rospy.Service("/gaan/face_rec", Faces, self.recognise_person)

        #self.track_pub = rospy.Publisher('/tiago/vision/track', String, queue_size=10)
        #self.cam_track_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.track_person)


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
            scale = float(img.shape[1])/480.
            small_frame = cv2.resize(img, (0, 0), fx=1./scale, fy=1./scale)

            self.known_name.append(self.get_name(face_file))
            face_loc = fr.face_locations(small_frame, model='cnn')
            face_encoding = fr.face_encodings(small_frame, face_loc)[0]
            self.known_enco.append(face_encoding)
        rospy.loginfo("known faces learned")

    def find_object_srv(self, msg):
        pass

    def save_last(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.frame = np.asarray(cv_img)

    def recognise_person(self, req):
        if self.frame is None:
            rospy.logerr("Seems like the camera is not running, please make sure the camera images are published on the right topic")
            return [""]
        # to avoid writing the frame while processing it.
        self.cam_sub.unregister()

        frame = self.frame
        #reduce this to speed things up but will reduce the accuracy
        scale=1
        small_frame = cv2.resize(frame, (0, 0), fx=1./float(scale), fy=1./float(scale))
        # convert back to RGB, cv uses BGR conventiion.

        face_loc = fr.face_locations(small_frame, model='cnn')
        rospy.loginfo("gaan_face_rec: {} face detected ".format(len(face_loc)))
        face_encodings = fr.face_encodings(small_frame, face_loc)

        face_names = []
        for enc in  face_encodings:
            matches = fr.compare_faces(self.known_enco, enc)
            name = "Unkown"
            face_distances = fr.face_distance(np.array(self.known_enco), np.array(enc))
            best_match_index = np.argmin(face_distances)


            if matches[best_match_index]:
                name = self.known_name[best_match_index]

            face_names.append(name)

        self.cam_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.save_last)
        return FacesResponse(face_names)

        # Use this bit of code to overlay the detection to the image
        #render = self.overlay(frame, face_loc, face_names, scale)
        #self.render = self.bridge.cv2_to_imgmsg(render, encoding="rgb8")


        #self.process_this_frame %= self.process_frame

        #self.face_rec_pub.publish(self.render)

    def overlay(self, frame, loc, names, scale):
        # Display the results
        for (top, right, bottom, left), name in zip(loc, names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= scale
            right *= scale
            bottom *= scale
            left *= scale

            w =frame.shape[0]
            h =frame.shape[1]
            pad_w = 20
            pad_h = 50
            # Draw a box around the face
            if top < pad_h:
                top_box = 0
            else:
                top_box = top - pad_h

            if bottom + pad_h >= h:
                bottom_box = h-1
            else:
                bottom_box = bottom + pad_h

            if left < pad_w:
                left_box = 0
            else:
                left_box = left - pad_w

            if right + pad_w >= w:
                right_box = w-1
            else:
                right_box = right + pad_w

            if name == "Unkown":
                cv2.rectangle(frame, (left_box, top_box), (right_box, bottom_box), (255, 0, 0), 2)
                cv2.rectangle(frame, (left_box, bottom_box - 35), (right_box, bottom_box), (255, 0, 0), cv2.FILLED)

            else:
                cv2.rectangle(frame, (left_box, top_box), (right_box, bottom_box), (0, 0, 255), 2)
                cv2.rectangle(frame, (left_box, bottom_box - 35), (right_box, bottom_box), (0, 0, 255), cv2.FILLED)

            # Draw a label with a name below the face
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left_box + 6, bottom_box - 6), font, 1.0, (255, 255, 255), 1)

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
