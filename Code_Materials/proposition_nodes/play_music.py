#! /usr/bin/env python

"""
Filename: play_music.py
Package: mci_ltl
Description: Play music
"""

from common import *
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class ltlStack(object):
    def __init__(self):
        self.request_data = False

    def callback(self, data):
        self.request_data = data.data

if __name__ == '__main__':
    rospy.init_node('playMusic')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/playMusic', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/playMusicComplete', Bool, callback=complete.callback)

    # publish to topics
    anim_pub = rospy.Publisher('/command', Command, queue_size=1)
    done_pub = rospy.Publisher('/mci_ltl/playMusicSignalComplete', Bool, queue_size=1)
    rospy.sleep(0.5)

    soundhandle = SoundClient()
    song = soundhandle.waveSound("/home/mayfield/LTL_MCI/LTL_stack/catkin_ws/src/LTL_stack/mci_ltl/media/Bright_Wish.wav")

    while not rospy.is_shutdown():
        if ltl.request_data:

            rospy.loginfo("About to start playMusic")

            publish_animation(anim_pub, "reset_head")
            rospy.sleep(0.5)

            song.play()
            rospy.sleep(43)

            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()
