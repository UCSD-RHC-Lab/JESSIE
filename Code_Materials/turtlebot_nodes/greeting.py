#! /usr/bin/env python

"""
Filename: greeting.py
Package: mci_ltl
Description: Give a greeting.
"""

#CHANGES MADE: none

import rospy

from mci_interactions.srv import DialoguePrompt
from std_msgs.msg import Bool, String, Float32
import json
import argparse

speak_and_display = rospy.ServiceProxy('dialogue/speak_and_display', DialoguePrompt)

class ltlStack(object):
    def __init__(self):
        self.request_data = False

    def callback(self, data):
        self.request_data = data.data

if __name__ == '__main__':
    rospy.init_node('greeting')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/greeting', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/greetingComplete', Bool, callback=complete.callback)

    # publish to topics
    done_pub = rospy.Publisher('/mci_ltl/greetingSignalComplete', Bool, queue_size=1)
    rospy.sleep(0.5)


    while not rospy.is_shutdown():
        if ltl.request_data:
            
            rospy.loginfo("About to start greeting")

            dialogue = \
              { "Prompt" : "Hi there! It's great to see you! "+
                           "Let me know when you are ready to start.",
                "Responses" : ["Continue"] }
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
      
            dialogue = \
              { "Prompt" : "Great! Let's get started.",
                "Responses" : [] }
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)
            
            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()