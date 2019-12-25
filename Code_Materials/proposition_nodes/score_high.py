#! /usr/bin/env python

"""
Filename: score_high.py
Package: mci_ltl
Description: Signals that the current score is high

If you are interested in learning more, or if you use this system in your work, 
please cite and refer to [1].

[1] A. Kubota, E. I. C. Peterson, V. Rajendren, H. Kress-Gazit, and L. D. Riek. 
JESSIE: Synthesizing Social Robot Behaviors for Personalized Neurorehabilitation 
and Beyond. In Proceedings of the 2020 ACM/IEEE International Conference on 
Human-Robot Interaction (HRI). IEEE, 2020.
"""

from common import *

class ltlStack(object):
    def __init__(self, high_threshold, pub):
        self.request_data = 0.0
        self.high_threshold = high_threshold
        self.pub = pub

    def callback(self, data):
        self.request_data = data.data
        self.pub.publish(self.request_data >= args.high_threshold)


if __name__ == '__main__':
    # Arguments to help generalize for each node
    parser = argparse.ArgumentParser()
    parser.add_argument('node_name', type=str)
    parser.add_argument('node_publish_topic', type=str)
    parser.add_argument('high_threshold', type=float, default=0)
    args, unknown = parser.parse_known_args()

    rospy.init_node('scoreHigh')
    rate = rospy.Rate(10)

    pub = rospy.Publisher('/mci_ltl/inputs/scoreHigh', Bool, queue_size=1)
    ltl = ltlStack(args.high_threshold, pub)
    rospy.Subscriber('/mci_ltl/scoreCurrent', Float32, callback=ltl.callback)

    rospy.spin()
