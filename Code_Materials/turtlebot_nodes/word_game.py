#! /usr/bin/env python

"""
Filename: word_game.py
Package: mci_ltl
Description: Do the word game exercise

If you are interested in learning more, or if you use this system in your work, 
please cite and refer to [1].

[1] A. Kubota, E. I. C. Peterson, V. Rajendren, H. Kress-Gazit, and L. D. Riek. 
JESSIE: Synthesizing Social Robot Behaviors for Personalized Neurorehabilitation 
and Beyond. In Proceedings of the 2020 ACM/IEEE International Conference on 
Human-Robot Interaction (HRI). IEEE, 2020.
"""

#CHANGES MADE: commented out animation for reset head

from common import *

word_lists = [
    ['Hammer', 'Monkey', 'Jupiter', 'Burrito', 'Saw', 'Zebra', 'Neptune',
     'Salad', 'Saturn', 'Drill', 'Giraffe', 'Noodles', 'Screwdriver', 
     'Sandwich', 'Lion', 'Venus'],
    ['Falcon', 'Engine', 'Elm', 'Eagle', 'Diamond', 'Radiator', 'Pine',
     'Maple', 'Hawk', 'Owl', 'Alternator', 'Ruby', 'Emerald', 'Oak',
     'Sapphire', 'Wheel'],
    ['Cat', 'Carrot', 'Broccoli', 'Bread', 'Mop', 'Hose', 'Asparagus',
     'Milk', 'Sponge', 'Eggs', 'Vacuum', 'Dog', 'Bird', 'Juice',
     'Hamster', 'Onion'],
    ['Grass', 'Chair', 'Stapler', 'Lawnmower', 'Tree', 'Eraser', 'Pick', 
     'Table', 'Pen', 'Rake', 'Ruler', 'Shovel', 'Flower', 'Sofa', 'Shrub',
     'Bed']
]

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
        if word.capitalize() in answers:
            score["Correct"] += 1

    return score

def give_level(curr_list, curr_level):
    dialogue = \
      { "Prompt" : "There will be " + str(len(curr_list)) + " words in this list. Here we go.",
        "Responses" : [] }
    speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
    rospy.sleep(1)

    for word in curr_list:
        dialogue = { "Prompt" : word, "Responses" : [] }
        speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
        rospy.sleep(1)
    rospy.sleep(1.5)

    dialogue = \
        { "Prompt" : "Now, type in as many words as you can remember before hitting submit. " + 
                     "Separate each word with a space.",
          "Responses" :
            ["$TEXT_INPUT"]}
    resp = speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
    words = resp.selected_response.split()

    CurrentScore = checkAnswers(words, curr_list)
    print "Got " + str(CurrentScore["Correct"]) + " / " + str(CurrentScore["Responses"]) + " correct"

    return CurrentScore

if __name__ == '__main__':
    rospy.init_node('wordGame')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/wordGame', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/wordGameComplete', Bool, callback=complete.callback)

    # publish to topics
    #anim_pub = rospy.Publisher('/command', Command, queue_size=1)
    done_pub = rospy.Publisher('/mci_ltl/wordGameSignalComplete', Bool, queue_size=1)
    score_pub = rospy.Publisher('/mci_ltl/scoreCurrent', Float32, queue_size=1)
    rospy.sleep(0.5)

    # Game Parameters
    NumListsDone = 0
    TotalWords = 0
    TotalResponses = 0
    TotalCorrect = 0

    while not rospy.is_shutdown():
        if ltl.request_data:

            curr_level = 0

            rospy.loginfo("About to start wordGame")

            #publish_animation(anim_pub, "reset_head")
            rospy.sleep(0.5)

            #### Level One ####
            dialogue = \
              { "Prompt" : "Let's begin the word game!",
                "Responses" : []}
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            curr_list = word_lists[curr_level]

            dialogue = \
              { "Prompt" : "For this list, I am going to speak the list aloud to you. Once I've read " +
                           "the list, type in as many words as you can remember.",
                "Responses" : ["Continue"] }
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)

            CurrentScore = give_level(curr_list, curr_level)
            
            NumListsDone += 1
            TotalWords += len(curr_list)
            TotalResponses += CurrentScore["Responses"]
            TotalCorrect += CurrentScore["Correct"]

            #### Level Two ####
            curr_level += 1
            curr_list = word_lists[curr_level]

            dialogue = \
              { "Prompt" : "For this second list, study the list of words, then let me know when " +
                           "you are ready to continue.",
                "Responses" : ["Continue"]}
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)

            dialogue = \
              { "Prompt" : ' '.join(curr_list),
                "Responses" : ["Continue"]}
            speak_and_display(True, True, False, False, json.dumps(dialogue), 0)

            dialogue = \
                { "Prompt" : "Now, type in as many words as you can remember",
                  "Responses" :
                    ["$TEXT_INPUT"]}
            resp = speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            words = resp.selected_response.split()

            CurrentScore = checkAnswers(words, curr_list)
            print "Got " + str(CurrentScore["Correct"]) + " / " + str(CurrentScore["Responses"]) + " correct"

            NumListsDone += 1
            TotalWords += len(curr_list)
            TotalResponses += CurrentScore["Responses"]
            TotalCorrect += CurrentScore["Correct"]

            #### Level Three ####
            curr_level += 1
            curr_list = word_lists[curr_level]

            dialogue = \
              { "Prompt" : "For this final list, read the list of words. Put the words into categories " +
                           "and use visual imagery to help you remember the categories or words. " +
                           "Then, let me know when you are ready to continue.",
                "Responses" : ["Continue"]}
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)

            dialogue = \
              { "Prompt" : ' '.join(curr_list),
                "Responses" : ["Continue"]}
            speak_and_display(True, True, False, False, json.dumps(dialogue), 0)

            dialogue = \
                { "Prompt" : "Now, type in as many words as you can remember",
                  "Responses" :
                    ["$TEXT_INPUT"]}
            resp = speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            words = resp.selected_response.split()

            CurrentScore = checkAnswers(words, curr_list)
            print "Got " + str(CurrentScore["Correct"]) + " / " + str(CurrentScore["Responses"]) + " correct"

            NumListsDone += 1
            TotalWords += len(curr_list)
            TotalResponses += CurrentScore["Responses"]
            TotalCorrect += CurrentScore["Correct"]

            #### Feedback ####
            dialogue = \
              { "Prompt" : "You did a fantastic job today! We managed to get through " + 
                           str(NumListsDone) + " lists today for a total of " + str(TotalWords) + 
                           " words. Out of those, you typed in " + str(TotalResponses) + 
                           " responses and got " + str(TotalCorrect) + " correct. Nicely done!",
                "Responses" : ["Continue"]}
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            score_pub.publish(float(TotalCorrect) / TotalWords)
            rospy.sleep(3)

            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()
