#! /usr/bin/env python

"""
Filename: break.py
Package: mci_ltl
Description: Take a break
"""

# CHANGES: made boredom a noise, different noise on resume

from common import *

class ltlStack(object):
    def __init__(self):
        self.request_data = False

    def callback(self, data):
        self.request_data = data.data

if __name__ == '__main__':
    rospy.init_node('break')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/break', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/breakComplete', Bool, callback=complete.callback)

    # publish to topics
    done_pub = rospy.Publisher('/mci_ltl/breakSignalComplete', Bool, queue_size=1)
    #ADDED: sound publisher
    sound_pub = rospublisher('/mobile_base/commands/sound', 'kobuki_msgs/Sound')
    sound_msg = rosmessage('kobuki_msgs/Sound')
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        if ltl.request_data:

            rospy.loginfo("About to start break")

            #publish_animation(anim_pub, "reset_head")
            #rospy.sleep(0.5)

            dialogue = \
              { "Prompt" : "We've been going nonstop for a while now. " +
                           "Let's take a short break.",
                "Responses" : [] }
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)

            dialogue = \
              { "Prompt" : "It's important to take " +
                           "breaks occasionally! Taking frequent short breaks helps to reduce " +
                           "fatigue and increase energy, improves your attention and focus, and "
                           "can even help your performance on our activity. ",
                "Responses" : [] }
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)
            
            #### Break Start ####
            dialogue = \
              { "Prompt" : "Let's relax for about a minute, then we will be nice and refreshed " +
                           "for when we start up again.",
                "Responses" : [] }
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)
            
            #### Break Prompt Resume ####
            #publish_animation(anim_pub, "boredom_60s_docked")
            #ADDED: sound message (value can be 0-6)
            sound_msg.value = 5
            sound_pub.publish(sound_msg)
      
            dialogue = \
              { "Prompt" : "Hit \"I'm ready to resume\" whenever you're ready.",
                "Responses" : ["I'm ready to resume" ] }
            resp = speak_and_display(True, True, False, False, json.dumps(dialogue), 60)
            selected_response = int(resp.selected_response)
            
            #### Break Reprompt Resume ####
            # Keep prompting the user until they respond
            while selected_response != 0:
                #publish_animation(anim_pub, "boredom_60s_docked")
                #ADDED: sound message (value can be 0-6)
                sound_msg.value = 5
                sound_pub.publish(sound_msg)
          
                dialogue = \
                  { "Prompt" : "Hey! Did you fall asleep on me? Let me know when you're ready to " +
                               "get back to it.",
                    "Responses" : ["I'm ready to resume"] }
                resp = speak_and_display(True, True, True, False, json.dumps(dialogue), 120)
                selected_response = int(resp.selected_response)
          
            #### Break End ####
            #publish_animation(anim_pub, "gotit")
            #ADDED: sound message (value can be 0-6)
            sound_msg.value = 0
            sound_pub.publish(sound_msg)
            rospy.sleep(1)
      
            dialogue = \
              { "Prompt" : "Great, let's get right back to it. Here we go.",
                "Responses" : [] }
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)
                
            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()
