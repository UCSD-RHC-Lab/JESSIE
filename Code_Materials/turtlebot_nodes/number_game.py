#! /usr/bin/env python

"""
Filename: number_game.py
Package: mci_ltl
Description: Give the number game exercise.
"""

#CHANGES MADE: commented out animation for reset head

from common import *
from random import randint

class ltlStack(object):
    def __init__(self):
        self.request_data = False

    def callback(self, data):
        self.request_data = data.data

if __name__ == '__main__':
    rospy.init_node('numberGame')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/numberGame', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/numberGameComplete', Bool, callback=complete.callback)


    # publish to topics
    #anim_pub = rospy.Publisher('/command', Command, queue_size=1)
    done_pub = rospy.Publisher('/mci_ltl/numberGameSignalComplete', Bool, queue_size=1)
    score_pub = rospy.Publisher('/mci_ltl/scoreCurrent', Float32, queue_size=1)
    rospy.sleep(0.5)

    # Game Parameters
    NumNumsPerList = 15
    NumberUpperBound = 10
    prev_number = randint(0, NumberUpperBound)
    TotalSums = 0
    TotalCorrect = 0

    while not rospy.is_shutdown():
        if ltl.request_data:

            rospy.loginfo("About to start numberGame")

            #publish_animation(anim_pub, "reset_head")
            #### Start Level ####
            dialogue = \
              { "Prompt" : "Let's start the number game.",
                "Responses" : [] }
            resp = speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            dialogue = \
              { "Prompt" : "Let's get through " + str(NumNumsPerList) + " numbers on this list. " +
                           "Here we go.",
                "Responses" : [] }
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            # Send the first number to start off the list
            dialogue = \
              { "Prompt" : str(prev_number),
                "Responses" : []}
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            rospy.sleep(0.5)

            # Now send NumNums per list
            num_correct = 0
            for i in range(0, NumNumsPerList):
                next_number = randint(0, NumberUpperBound)
                answer = prev_number + next_number
                
                dialogue = \
                  { "Prompt" : str(next_number),
                    "Responses" : [] }
                speak_and_display(True, True, True, False, json.dumps(dialogue), 0)

                # Ask for the sum and give 4 seconds to respond.
                dialogue = \
                  { "Prompt" : "What is the sum?",
                    "Responses" : ["$NUMBER_INPUT"] }
                resp = speak_and_display(True, True, False, False, json.dumps(dialogue), 4)
          
                TotalSums += 1
                
                try:
                    response = int(resp.selected_response)
                    if response == answer:
                        num_correct += 1
                        TotalCorrect += 1
                        print "Correct: " + str(response) + " == " + str(answer)
                    else:
                        print "Incorrect: " + str(response) + " != " + str(answer)
                except:
                    pass

                prev_number = next_number

            # After all the words have been done, calculate how well the person did
            CurrentScore = float(num_correct) / NumNumsPerList
            print "Score: %.1f" % CurrentScore

            #### Feedback ####
            dialogue = \
              { "Prompt" : "On that last list you got " +
                           '%d' % (CurrentScore * 100) + " percent correct.",
                "Responses" : [] }
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)

            score_pub.publish(float(TotalCorrect) / TotalSums)
            rospy.sleep(4)
            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()
