#! /usr/bin/env python

"""
Filename: number_game_instructions.py
Package: mci_ltl
Description: Give instructions and an example about the number game

If you are interested in learning more, or if you use this system in your work, 
please cite and refer to [1].

[1] A. Kubota, E. I. C. Peterson, V. Rajendren, H. Kress-Gazit, and L. D. Riek. 
JESSIE: Synthesizing Social Robot Behaviors for Personalized Neurorehabilitation 
and Beyond. In Proceedings of the 2020 ACM/IEEE International Conference on 
Human-Robot Interaction (HRI). IEEE, 2020.
"""

from common import *

class ltlStack(object):
    def __init__(self):
        self.request_data = False

    def callback(self, data):
        self.request_data = data.data

if __name__ == '__main__':
    rospy.init_node('numberGameInstructions')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/numberGameInstructions', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/numberGameInstructionsComplete', Bool, callback=complete.callback)

    # publish to topics
    anim_pub = rospy.Publisher('/command', Command, queue_size=1)
    done_pub = rospy.Publisher('/mci_ltl/numberGameInstructionsSignalComplete', Bool, queue_size=1)
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        if ltl.request_data:

            rospy.loginfo("About to start numberGameInstructions")

            publish_animation(anim_pub, "reset_head")
            rospy.sleep(0.5)

            #### Intro ####
            dialogue = \
              { "Prompt" : "Today we are going to be working on Cognitive Training. " +
                           "Part of Cognitive Training involves playing games that \"work out\" " +
                           "different parts of your brain. It's important to do these exercises " +
                           "daily, just like with normal working out, so that it will help you " +
                           "to be able to remember more.",
                "Responses" : []}
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)

            #### Instructions ####
            dialogue = \
              { "Prompt" : "Let's play a Number Game!",
                "Responses" : [] }
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            dialogue = \
              { "Prompt" : "I'll tell you the instructions first, and then we will go over an example " +
                           "to make sure you understand.",
                "Responses" : []}
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            dialogue = \
              { "Prompt" : "I will read a list of numbers to you. Listen to the first two " +
                           "numbers, add them up, and then enter their sum on the screen.",
                "Responses" : []}
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            dialogue = \
              { "Prompt" : "When you hear the next number, add it to the one you heard before it. " +
                           "Continue to add the next number to the preceding one.",
                "Responses" : []}
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            dialogue = \
              { "Prompt" : "You will have approximately three seconds to respond.",
                "Responses" : []}
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            dialogue = \
              { "Prompt" : "Remember, you are not being asked to calculate a running total, but rather " +
                           "the sum of the last two numbers I read.",
                "Responses" : []}
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            #### Example ####
            publish_animation(anim_pub, "yes")
            rospy.sleep(1.5)

            display = \
              { "Prompt" : "Let's do an example now.",
                "Responses" : ["Continue"]}
            speak_and_display(True, True, True, False, json.dumps(display), 0)
          
            display = {"Prompt" : "5 + 7 = 12", "Responses" : []}
            speak_and_display(True, False, False, False, json.dumps(display), 0)
            dialogue = \
            { "Prompt" : "If the first two numbers are 5 and 7, the correct sum is 12.",
                "Responses" : []}
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)

            display = {"Prompt" : "5 + 7 = 12\n7 + 3 = 10", "Responses" : []}
            speak_and_display(True, False, False, False, json.dumps(display), 0)
            dialogue = \
            { "Prompt" : "If the next number was 3, then the correct answer is 10.",
                "Responses" : []}
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            
            dialogue = \
            { "Prompt" : "Now if the next number is 2, what is the correct answer?" +
                         "\n5 + 7 = 12\n7 + 3 = 10\n3 + 2 = ?",
              "Responses" : [
                "$NUMBER_INPUT"]}
            resp = speak_and_display(True, True, True, False, json.dumps(dialogue), 0)

            # Person answered correctly
            if int(resp.selected_response) == 5:
                dialogue = \
                    { "Prompt" : "That's right! We add 2 to the last number that I read to get 5." +
                           "\n5 + 7 = 12\n7 + 3 = 10\n3 + 2 = 5",
                      "Responses" : ["Continue"]}
                resp = speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            # Person answered incorrectly
            else:
                dialogue = \
                    { "Prompt" : "Oops, that's not quite right. We add 2 to the last number I read to get 5." +
                           "\n5 + 7 = 12\n7 + 3 = 10\n3 + 2 = 5",
                      "Responses" : ["Continue"]}
                resp = speak_and_display(True, True, True, False, json.dumps(dialogue), 0)

            publish_animation(anim_pub, "gotit")
            rospy.sleep(1.5)
      
            dialogue = \
              { "Prompt" : "Great! Remember this is supposed to be a challenging task. If you lose " +
                           "your place, just jump right back in. Listen for two consecutive numbers, add " +
                           "them up, and keep going. It's okay if you don't get all of the numbers correct, " +
                           "especially at first, but we can continue practicing together.",
                "Responses" : [] }
            resp = speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()
