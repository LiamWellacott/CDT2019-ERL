#!/usr/bin/env python

import logging

import rospy
from std_msgs.msg import String

from random import randint

from flask import Flask, render_template

from flask_ask import Ask, statement, question, session, convert_errors #Added convert_errors ~Emilyann

from erl_msgs.srv import Speech, SpeechResponse, Command

app = Flask(__name__)

ask = Ask(app, "/")

logging.getLogger("flask_ask").setLevel(logging.DEBUG)

rospy.init_node("nlp", anonymous=True)
speachServ = rospy.Service('speech', handle_speech)
rate = rospy.Rate(10)
resp_pub = rospy.Publisher("/erl/nlp/resp", String, queue_size=10)
cmd_pub = rospy.Publisher("/erl/nlp/command", String, queue_size=10)

def handle_speech(req):
    rospy.loginfo("Saying: " + req)
    return SpeechResponse("Done")

def cmd_service(msg):
    rospy.wait_for_service('voc_cmd')
    try:
        voc_cmd_service = rospy.ServiceProxy('voc_cmd', Command)
        ret = voc_cmd_service(msg)
        return ret
    except rospy.ServiceException as e:
         rospy.loginfo("Service call failed: %s"%e)
         return "vocal command service failed"

@ask.launch
def new_game():
    welcome_msg = render_template('welcome')
    publish(welcome_msg, "None")
    return question(welcome_msg)


@ask.intent("PlaceIntent", convert={"object": str}, default={"place": "bin"})
def placeobj(object, place):
    str=""
    ret = cmd_service(str)
    return statement(ret)


@ask.intent("PickupIntent", convert={"object":str})
def pickup(object):
    str=""
    ret = cmd_service(str)
    return statement(ret)



@ask.intent("PickupIntent")
def response(object):

    msg = render_template('find', object=object)

    session.attributes['object'] = object

    return statement(msg)

@ask.intent("ReadinessCommand")
def introduce():

    msg = render_template('intro')

    return statement(msg)

@ask.intent("FoundItem") ###Still using alexa confirmation - can delete this and use code here instead
def found(object):

    msg = render_template('found', object=object)

    session.attributes['object'] = object

    return statement(msg)

@ask.intent("Handoff")
def handoff(object):

    msg = render_template('handoff', object=object)

    session.attributes['object'] = object

    return statement(msg)


@ask.intent("Amazon.NavigateHomeIntent")
def home():
    publish("Home", "None")
    return statement("home.")

@ask.intent("AMAZON.CancelIntent")
def cancel():
    publish("Cancel", "None")
    return statement("cancel.")

@ask.intent("AMAZON.HelpIntent")
def help():
    publish("help", "None")
    return question("Help you? burk.")

@ask.intent("AMAZON.StopIntent")
def stop():
    publish("Stop", "None")
    return statement("I'm not stopping for you!")

def publish(resp, cmd):
    resp_msg = String(resp)
    cmd_msg = String(cmd)
    resp_pub.publish(resp_msg)
    cmd_pub.publish(cmd_msg)

def main():
    app.run(debug=True, host='0.0.0.0')

if __name__ == '__main__':
    main()
