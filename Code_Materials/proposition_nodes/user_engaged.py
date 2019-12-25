#! /usr/bin/env python

"""
Filename: user_engaged.py
Package: mci_ltl
Description: Signals whether or not the user is looking at the robot

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
    # Arguments to help generalize for each node
    parser = argparse.ArgumentParser()
    parser.add_argument('node_name', type=str)
    parser.add_argument('node_publish_topic', type=str)
    args, unknown = parser.parse_known_args()

    rospy.init_node('userEngaged')
    rate = rospy.Rate(10)

    pub = rospy.Publisher('/mci_ltl/inputs/userEngaged', Bool, queue_size=1)
    ltl = ltlStack()
    rospy.Subscriber('/mci_ltl/eyetracking', String, callback=ltl.callback)

    time_unengaged = 10 # 10 seconds
    time_of_last_true = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        if ltl.request_data:
            time_of_last_true = rospy.Time.now().to_sec()

        time_since_last_true = rospy.Time.now().to_sec() - time_of_last_true
        if time_since_last_true >= time_unengaged:
            time_of_last_true = rospy.Time.now().to_sec()
            pub.publish(False)
        
        rate.sleep()
