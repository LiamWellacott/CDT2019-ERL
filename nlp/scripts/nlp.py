#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import json
import requests

from gaan_msgs.srv import Speech, SpeechResponse, Command

class NLP:

    def __init__(self):
        
        cmd_pub = rospy.Publisher('/gaan/nlp/command', String, queue_size=10)

        rospy.Subscriber('/gaan/nlp/user_msg', String, self.userMsgCallback)

    def userMsgCallback(self, msg):
        rospy.loginfo(msg.data)

        response = requests.post('http://localhost:5005/webhooks/rest/webhook', json = {'sender': 'Tiago', 'message': msg.data})
        response_obj = eval(response.text)
        for text in response_obj:
            json_text = json.dumps(text)
            json_obj = json.loads(json_text)
            rospy.loginfo(json_obj["text"])

    def speak(self):
        return # TODO

def main():

    rospy.init_node("nlp", anonymous=True)

    nlp = NLP()

    rospy.spin()
    # rate = rospy.Rate(1)

    # while True:

    #     nlp.step()
    

if __name__ == '__main__':
    main()
