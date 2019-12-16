#! /usr/bin/env python

"""
Filename: mindfulness_exercise.py
Package: mci_ltl
Description: Execute a mindfulness exercise
"""

#CHANGES MADE: commented out animation

import rospy

from mci_interactions.srv import DialoguePrompt
from std_msgs.msg import Bool, String, Float32
import json
import argparse
import os

class ltlStack(object):
    def __init__(self):
        self.request_data = False

    def callback(self, data):
        self.request_data = data.data

def loadInstructions():
    path = os.path.join(os.path.dirname(__file__), "dialogue", "mindfulness_instructions", "MindfulSittingMeditation.txt")
    fp = open(path)
    lines = [line.strip() for line in fp.readlines()]
    fp.close()

    ActivityName = lines[0].split("Name:")[1]

    Instructions = []

    for line in lines[1:]:
        #skip empty lines and comments
        if len(line) == 0 or line.isspace() or line[0] == "#":
            continue

        #otherwise, add the line to the instructions
        Instructions.append(line)

    return Instructions

if __name__ == '__main__':
    rospy.init_node('mindfulnessExercise')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/mindfulnessExercise', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/mindfulnessExerciseComplete', Bool, callback=complete.callback)


    # publish to topics
    #anim_pub = rospy.Publisher('/command', Command, queue_size=1)
    done_pub = rospy.Publisher('/mci_ltl/mindfulnessExerciseSignalComplete', Bool, queue_size=1)
    rospy.sleep(0.5)

    Instructions = loadInstructions()

    while not rospy.is_shutdown():
        if ltl.request_data:

            rospy.loginfo("About to start mindfulnessExercise")

            #publish_animation(anim_pub, "reset_head")
            #rospy.sleep(0.5)

            #### Introduction ####
            dialogue = {"Prompt" : Instructions[0], 
               "Responses" : []}
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            dialogue = {"Prompt" : "I'll close my eyes along with you.",
                       "Responses" : []}
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)

            #publish_animation(anim_pub, "sleep_very_quick")
            #rospy.sleep(1.5)

            #### Exercise ####
            for instruction in Instructions[1:]:
                # If the instruction tells us to wait, sleep for the amount of time in seconds
                if instruction[0:5] == "WAIT:":
                    time = int(instruction.split("WAIT: ")[1])
                    rospy.sleep(time)
                    continue
                # Otherwise, read the instruction aloud
                dialogue = {"Prompt" : instruction, 
                            "Responses" : []}
                speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
                rospy.sleep(2)

            #### Conclusion ####
            publish_animation(anim_pub, "proud_1")
            rospy.sleep(1.5)
            dialogue = {"Prompt" : "You did a fantastic job today! " +
                                   "Great job staying so focused for so long!",
                   "Responses" : ["Continue"]}
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)

            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()
