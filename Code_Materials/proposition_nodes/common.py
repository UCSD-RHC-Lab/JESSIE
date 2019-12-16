#!/usr/bin/env python

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
