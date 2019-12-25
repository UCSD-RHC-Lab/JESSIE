#! /usr/bin/env python

"""
Filename: tactile_interaction.py
Package: mci_ltl
Description: Signals whether or not the robot's head has been touched

If you are interested in learning more, or if you use this system in your work, 
please cite and refer to [1].

[1] A. Kubota, E. I. C. Peterson, V. Rajendren, H. Kress-Gazit, and L. D. Riek. 
JESSIE: Synthesizing Social Robot Behaviors for Personalized Neurorehabilitation 
and Beyond. In Proceedings of the 2020 ACM/IEEE International Conference on 
Human-Robot Interaction (HRI). IEEE, 2020.
"""

from common import *
from std_msgs.msg import Empty

class ltlStack(object):
    def __init__(self, pub):
        self.request_data = False
        self.pub = pub

    def callback(self, data):
        print("Head touched!")
        self.request_data = True
        self.pub.publish(True)

if __name__ == '__main__':
    rospy.init_node('tactileInteraction')
    rate = rospy.Rate(10)

    pub = rospy.Publisher('/mci_ltl/inputs/tactileInteraction', Bool, queue_size=1)
    ltl = ltlStack(pub)
    rospy.Subscriber('/gizmo_brain/touch', Empty, callback=ltl.callback)

    rospy.spin()
