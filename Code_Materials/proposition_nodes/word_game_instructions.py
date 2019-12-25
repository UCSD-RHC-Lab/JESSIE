#! /usr/bin/env python

"""
Filename: word_game_instructions.py
Package: mci_ltl
Description: Give instructions and an example about the word game

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

def checkAnswers(responses, answers):
    # Make sure that each entry is unique
    responses = set(responses)
    answers = set(answers)

    score = { "Correct" : 0, "Responses" : 0}

    for word in responses:
        score["Responses"] += 1
        if word.lower() in answers:
            score["Correct"] += 1

    return score

if __name__ == '__main__':
    rospy.init_node('wordGameInstructions')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/wordGameInstructions', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/wordGameInstructionsComplete', Bool, callback=complete.callback)

    # publish to topics
    anim_pub = rospy.Publisher('/command', Command, queue_size=1)
    done_pub = rospy.Publisher('/mci_ltl/wordGameInstructionsSignalComplete', Bool, queue_size=1)
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        if ltl.request_data:

            rospy.loginfo("About to start wordGameInstructions")

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
            rospy.sleep(1)
            
            #### Instructions ####
            dialogue = \
              { "Prompt" : "Let's play a Word Game today!",
                "Responses" : [] }
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            dialogue = \
              { "Prompt" : "I'll reed you the instructions first, and then we will go over an example " +
                           "to make sure you understand.",
                "Responses" : []}
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            dialogue = \
              { "Prompt" : "I am going to read a list of words to you. Listen to the list " +
                           "and then enter as many as you can remember on the tablet.",
                "Responses" : []}
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            #### Example ####
            display = \
              { "Prompt" : "Let's do an example now.",
                "Responses" : ["Continue"]}
            speak_and_display(True, True, True, False, json.dumps(display), 0)
          
            display = {"Prompt" : "Apple\nBanana", "Responses" : []}
            speak_and_display(True, False, False, False, json.dumps(display), 0)
            
            dialogue = \
            { "Prompt" : "Here is the list. Apple. Banana",
                "Responses" : []}
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)

            rospy.sleep(1)

            dialogue = \
            { "Prompt" : "Now, enter as many words as you can remember, putting a space between " +
                         "each word. When you have entered all the words, hit \"Done\" on the keyboard.",
              "Responses" : [
                "$TEXT_INPUT"]}
            resp = speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            words = resp.selected_response.split()

            score = checkAnswers(words, ["apple", "banana"])

            if score["Correct"] == 2 and score["Responses"] == 2:
                dialogue = \
                    { "Prompt" : "That's right! I spoke the words \"Apple\" and \"Banana\" and you entered them " +
                                 "correctly with a space in between.",
                      "Responses" : ["Continue"]}
                resp = speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            else:
                dialogue = \
                    { "Prompt" : "Oops, that's not quite right. I spoke the words \"Apple\" and \"Banana\" so you " +
                                 "would either write \"Apple Banana\" or \"Banana Apple\".",
                      "Responses" : ["Continue"]}
                resp = speak_and_display(True, True, True, False, json.dumps(dialogue), 0)

            dialogue = \
              { "Prompt" : "Remember, the words you type in are not case-sensitive and the order doesn't matter. " +
                           "Just be sure to separate each word with a space and make sure you don't add in any " + 
                           "extra words!",
                "Responses" : ["Continue"] }
            resp = speak_and_display(True, True, True, False, json.dumps(dialogue), 0)

            publish_animation(anim_pub, "gotit")
            rospy.sleep(1.5)
      
            dialogue = \
              { "Prompt" : "Great! Remember this is supposed to be a challenging task. " +
                           "You won't get all of the words correct, especially at first, " +
                           "but we can continue to improve together.",
                "Responses" : [] }
            resp = speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()
