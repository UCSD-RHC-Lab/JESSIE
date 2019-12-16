#! /usr/bin/env python

"""
Filename: event_node_complete.py
Package: mci_ltl
Description: Signals that an event node has completed.
"""

from common import *

class ltlStack(object):
    def __init__(self, pub):
        self.request_data = False
        self.pub = pub

    def callback(self, data):
        self.request_data = data.data
        self.pub.publish(self.request_data)

if __name__ == '__main__':
    # Arguments to help generalize for each node
    parser = argparse.ArgumentParser()
    parser.add_argument('node_name', type=str)
    parser.add_argument('node_publish_topic', type=str)
    parser.add_argument('signal_complete_topic', type=str)
    args, unknown = parser.parse_known_args()

    rospy.init_node(args.node_name)
    rate = rospy.Rate(10)

    pub = rospy.Publisher(args.node_publish_topic, Bool, queue_size=1)
    ltl = ltlStack(pub)
    rospy.Subscriber(args.signal_complete_topic, Bool, callback=ltl.callback)

    rospy.spin()
