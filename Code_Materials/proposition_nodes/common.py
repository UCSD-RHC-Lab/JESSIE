#!/usr/bin/env python

"""
If you are interested in learning more, or if you use this system in your work, 
please cite and refer to [1].

[1] A. Kubota, E. I. C. Peterson, V. Rajendren, H. Kress-Gazit, and L. D. Riek. 
JESSIE: Synthesizing Social Robot Behaviors for Personalized Neurorehabilitation 
and Beyond. In Proceedings of the 2020 ACM/IEEE International Conference on 
Human-Robot Interaction (HRI). IEEE, 2020.
"""

import rospy

from mci_interactions.srv import DialoguePrompt
from std_msgs.msg import Bool, String, Float32
from gizmo_msgs.msg import Command
from mayfield_msgs.msg import KeyValue
import json
import argparse

speak_and_display = rospy.ServiceProxy('dialogue/speak_and_display', DialoguePrompt)

def publish_animation(publisher, animation):
	publisher.publish(Command("play_stored_animation_command",
                    [KeyValue("name", animation)]))
