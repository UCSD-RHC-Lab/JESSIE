#! /usr/bin/env python

"""
Filename: assessment.py
Package: mci_ltl
Description: Give an assessment

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

prompts = [
    "I lost myself in this experience.",
    "The time I spent doing this activity just slipped away.",
    "I was absorbed in this experience.",
    "I felt frustrated while doing this activity.",
    "I found this activity confusing to me.",
    "Doing this activity was taxing.",
    "Doing this activity was worthwhile.",
    "My experience was rewarding.",
    "I felt interested in this experience."
]

if __name__ == '__main__':
    rospy.init_node('assessment')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/assessment', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/assessmentComplete', Bool, callback=complete.callback)

    # publish to topics
    anim_pub = rospy.Publisher('/command', Command, queue_size=1)
    done_pub = rospy.Publisher('/mci_ltl/assessmentSignalComplete', Bool, queue_size=1)
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        if ltl.request_data:

            rospy.loginfo("About to start assessment")

            publish_animation(anim_pub, "reset_head")
            rospy.sleep(0.5)

            #### Ask to give assessment ####
            dialogue = \
                { "Prompt" : "One last thing! Before we end today, would you mind filling " +
                                         "out a brief survey about your experience?",
                    "Responses" : ["Continue"] }
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
    
            dialogue = \
                { "Prompt" : "I am trying to learn how the way I act affects your engagement " +
                             "with the activities. Your responses will help me to be able to " +
                             "better serve you and make all of our interactions more fun and " +
                             "engaging.",
                  "Responses" : ["Continue"] }
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
    
            #### Give survey ####
            EngagementSurveyResults = []

            dialogue = \
                { "Prompt" : "The following statements ask you to reflect on your experience of engaging " +
                             "with this activity. For each statement, please select the response that " +
                             "indicates what is most true for you.",
                  "Responses" : ["Continue"] }
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)

            dialogue["Responses"] = [
                "$RADIO_INPUT", "5",
                "Strongly Agree",
                "Strongly Disagree"]

            for prompt in prompts:
                dialogue["Prompt"] = prompt
                resp = speak_and_display(True, True, False, False, json.dumps(dialogue), 0)
                EngagementSurveyResults.append(int(resp.selected_response))

            print("EngagementSurveyResults: {}".format(EngagementSurveyResults))
                    
            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()
