#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import json
import requests

from gaan_msgs.srv import Speech, SpeechResponse, Command

from sound_play.libsoundplay import SoundClient

class NLP:

    def __init__(self):
        
        # Used to send user messages interpreted as commands to the controller module
        self.command = rospy.ServiceProxy('/gaan/nlp/command', Command)

        # Raw user messages interpreted by the speech to text system are received on the user_msg topic
        rospy.Subscriber('/gaan/nlp/user_msg', String, self.userMsgCallback)

        # Used by the controller to request a message be spoken via Tiago's TTS service
        rospy.Service('/gaan/nlp/speech', Speech, self.speakHandler)

        # Used for TTS     
        self.sound_client = SoundClient()   


    def userMsgCallback(self, msg):

        # Log interpreted message
        rospy.loginfo(msg.data)

        # Send message to rasa to be interpreted and generate a response
        response = requests.post('http://localhost:5005/webhooks/rest/webhook', json = {'sender': 'Tiago', 'message': msg.data})
        response_obj = eval(response.text)
        for text in response_obj:
            json_text = json.dumps(text)
            json_obj = json.loads(json_text)

            # Log response
            rospy.loginfo(json_obj["text"])

            # Say response
            self.sound_client.say(json_obj["text"])

            # TODO match the response to a predefined message to decide whether to launch a command
            # Note there should be an interpretation layer somewhere in our NLP system which attempts to fit incomming sentences into
            # the competition grammar, rasa should be capable of this apparently, but we didn't get far implementing this.
            # Therefore we have the following hack to simply match the single command we are capabable of supporting.

    def speakHandler(self, req):
        self.sound_client.say(req.text)

        return SpeechResponse(True)

def main():

    rospy.init_node("nlp", anonymous=True)

    nlp = NLP()

    rospy.spin()
    # rate = rospy.Rate(1)

    # while True:

    #     nlp.step()
    

if __name__ == '__main__':
    main()
