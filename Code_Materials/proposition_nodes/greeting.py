#! /usr/bin/env python

"""
Filename: greeting.py
Package: mci_ltl
Description: Give a greeting.

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
    rospy.init_node('greeting')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/greeting', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/greetingComplete', Bool, callback=complete.callback)

    # publish to topics
    anim_pub = rospy.Publisher('/command', Command, queue_size=1)
    done_pub = rospy.Publisher('/mci_ltl/greetingSignalComplete', Bool, queue_size=1)
    rospy.sleep(0.5)

    # Put Kuri in animation mode
    cmd = Command()
    cmd.name = 'start_animation'
    anim_pub.publish(cmd)

    while not rospy.is_shutdown():
        if ltl.request_data:
            
            rospy.loginfo("About to start greeting")

            publish_animation(anim_pub, "reset_head")
            rospy.sleep(0.5)
            
            publish_animation(anim_pub, "gotit")
            rospy.sleep(1.5)

            dialogue = \
              { "Prompt" : "Hi there! It's great to see you! "+
                           "Let me know when you are ready to start.",
                "Responses" : ["Continue"] }
            speak_and_display(True, True, True, False, json.dumps(dialogue), 0)

            publish_animation(anim_pub, "yes")
            rospy.sleep(1.5)
      
            dialogue = \
              { "Prompt" : "Great! Let's get started.",
                "Responses" : [] }
            speak_and_display(False, False, True, False, json.dumps(dialogue), 0)
            rospy.sleep(1)
            
            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()
