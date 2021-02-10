#!/usr/bin/env python

import rospy

IDLE = 0
EXPLORING = 1
NAVIGATING = 2
FOLLOWING = 3
GUIDING = 4
state = IDLE

### service functions
def setExplore():
    global state
    state = EXPLORING

def setNavigate():
    global state
    state = NAVIGATING

def setFollow():
    global state
    state = FOLLOWING

def setGuide():
    global state
    state = GUIDING

### state based logic
def explore():
    return

def navigate():
    return

def follow():
    return

def guide():
    return

### main loop
def init():
    return

def main():

    init()

    while(True):

        # check collisions

        # execute behaviour
        if state == EXPLORING:
            explore()
        elif state == NAVIGATING:
            navigate()
        elif state == FOLLOWING:
            follow()
        elif state == GUIDING:
            guide()

        # wait

if __name__ == "__main__":
    main()


