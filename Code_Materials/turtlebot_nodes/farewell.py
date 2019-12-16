#! /usr/bin/env python

"""
Filename: farewell.py
Package: mci_ltl
Description: Say goodbye
"""

#CHANGES MADE: commented out animation

import rospy

from mci_interactions.srv import DialoguePrompt
from std_msgs.msg import Bool, String, Float32
import json
import argparse

class ltlStack(object):
    def __init__(self):
        self.request_data = False

    def callback(self, data):
        self.request_data = data.data

if __name__ == '__main__':
    rospy.init_node('farewell')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/farewell', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/farewellComplete', Bool, callback=complete.callback)


    # publish to topics
    #anim_pub = rospy.Publisher('/command', Command, queue_size=1)
    done_pub = rospy.Publisher('/mci_ltl/farewellSignalComplete', Bool, queue_size=1)
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        if ltl.request_data:

            rospy.loginfo("About to start farewell")

            #publish_animation(anim_pub, "reset_head")
            #rospy.sleep(0.5)

            dialogue = \
              { "Prompt" : "Thank you for spending time with me today! " +
                           "I will see you next time!",
                "Responses" : [] }
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)

           # publish_animation(anim_pub, "bye")
            #rospy.sleep(5)
            #publish_animation(anim_pub, "docking_complete_asleep")

            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()
