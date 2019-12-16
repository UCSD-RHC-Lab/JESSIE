#! /usr/bin/env python

"""
Filename: sneeze.py
Package: mci_ltl
Description: Sneeze
"""

from common import *


class ltlStack(object):
    def __init__(self):
        self.request_data = False

    def callback(self, data):
        self.request_data = data.data

if __name__ == '__main__':
    rospy.init_node('sneeze')
    rate = rospy.Rate(10)

    ltl = ltlStack()
    complete = ltlStack()

    # subscribe to ltlstack controller
    rospy.Subscriber('/mci_ltl/outputs/sneeze', Bool, callback=ltl.callback)
    rospy.Subscriber('/mci_ltl/inputs/sneezeComplete', Bool, callback=complete.callback)

    # publish to topics
    anim_pub = rospy.Publisher('/command', Command, queue_size=1)
    done_pub = rospy.Publisher('/mci_ltl/sneezeSignalComplete', Bool, queue_size=1)
    rospy.sleep(0.5)


    while not rospy.is_shutdown():
        if ltl.request_data:

            rospy.loginfo("About to start sneezing")

            publish_animation(anim_pub, "reset_head")
            rospy.sleep(0.5)

            publish_animation(anim_pub, "sneeze_normal")
            rospy.sleep(0.5)

            # Signal that the event is complete
            while not complete.request_data:
                done_pub.publish(True)
                rospy.sleep(3)

        rate.sleep()
