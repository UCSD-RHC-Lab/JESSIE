#! /usr/bin/env python

"""
Filename: mindfulness_instructions.py
Package: mci_ltl
Description: Give instructions about the mindfulness exercise
"""

from common import *

class ltlStack(object):
    def __init__(self):
        self.request_data = False

    def callback(self, data):
        self.request_data = data.data

if __name__ == '__main__':
    rospy.init_node('mindfulnessInstructions')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/mindfulnessInstructions', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/mindfulnessInstructionsComplete', Bool, callback=complete.callback)

    # publish to topics
    anim_pub = rospy.Publisher('/command', Command, queue_size=1)
    done_pub = rospy.Publisher('/mci_ltl/mindfulnessInstructionsSignalComplete', Bool, queue_size=1)
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        if ltl.request_data:

            rospy.loginfo("About to start mindfulnessInstructions")

            publish_animation(anim_pub, "reset_head")
            rospy.sleep(0.5)

            #### Intro ####
            dialogue = \
              { "Prompt" : "Today we are going to be working on Mindfulness Training.",
                "Responses" : []}
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)

            dialogue = \
              { "Prompt" : "Mindfulness means focusing on the experience of the present moment. " +
                           "It's important for us to do Mindfulness exercises because frequent practice " +
                           "can improve your physical, emotional, and cognitive health.",
                "Responses" : []}
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            #### Instructions ####
            dialogue = \
                { "Prompt" : "You will gently bring your focus to the present moment, non-judgementally, and with" +
                             " an open and curious attitude. Perhaps like a scientist or an objective observer, " +
                             "here to record details about something new and unknown.",
                  "Responses" : []}
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            dialogue = \
                { "Prompt" : "Mindful exercises can be done laying down on the floor, or sitting in a chair or " +
                             "on the floor. Many people practice mindfulness once a day or several times per week, " +
                             "either first thing in the morning, as a break sometime during the day, or at the " +
                             "end of their day. Mindfulness can be done briefly, around 5 to 15 minutes, or it " +
                             "can be an extended exercise, 20 to 45 minutes of longer. Today we will practice a " +
                             "short exercise while sitting in a chair.",
                  "Responses" : []}
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()
