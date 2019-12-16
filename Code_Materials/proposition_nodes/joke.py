#! /usr/bin/env python

"""
Filename: joke.py
Package: mci_ltl
Description: Tell a joke
"""

from common import *

class ltlStack(object):
    def __init__(self):
        self.request_data = False

    def callback(self, data):
        self.request_data = data.data

if __name__ == '__main__':
    rospy.init_node('joke')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/joke', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/jokeComplete', Bool, callback=complete.callback)


    # publish to topics
    anim_pub = rospy.Publisher('/command', Command, queue_size=1)
    done_pub = rospy.Publisher('/mci_ltl/jokeSignalComplete', Bool, queue_size=1)
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        if ltl.request_data:

            rospy.loginfo("About to start joke")

            publish_animation(anim_pub, "reset_head")
            rospy.sleep(0.5)

            dialogue = \
              { "Prompt" : "Let me tell you a joke!",
                "Responses" : [] }
            speak_and_display(True, False, True, False, json.dumps(dialogue), 0)

            dialogue = \
              { "Prompt" : "What did the pirate say on his eightieth birthday?",
                "Responses" : [] }
            speak_and_display(True, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            dialogue = \
              { "Prompt" : "Aye matey!", 
                "Responses" : [] }
            speak_and_display(True, False, True, False, json.dumps(dialogue), 0)

            publish_animation(anim_pub, "giggle")
            rospy.sleep(1)
            
            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()
