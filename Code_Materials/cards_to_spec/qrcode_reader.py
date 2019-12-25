#! /usr/bin/env python

"""
Filename: qrcode_reader.py
Translates an image of the card program to an LTLspec and saves it to a file.
If the output file is unspecified, it saves it to a "mci_ltl.slugs" in the current directory.

Use: python3 qrcode_reader.py name_of_image [output_location]

If you are interested in learning more, or if you use this system in your work, 
please cite and refer to [1].

[1] A. Kubota, E. I. C. Peterson, V. Rajendren, H. Kress-Gazit, and L. D. Riek. 
JESSIE: Synthesizing Social Robot Behaviors for Personalized Neurorehabilitation 
and Beyond. In Proceedings of the 2020 ACM/IEEE International Conference on 
Human-Robot Interaction (HRI). IEEE, 2020.
"""

from copy import deepcopy
from pyzbar import pyzbar
import argparse
import json
import sys, os

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2

def decode_image(img_file):
	img = cv2.imread(img_file)
	return pyzbar.decode(img)

def get_nodes(qrcodes):
	nodes = {}
	# loop over the detected barcodes
	for qrcode in qrcodes:
		# get the bounding box of the qrcode
		data = json.loads(qrcode.data.decode("utf-8"))
		nodes[data['Name']] = {'Type' : data['Type'], 
							   'Rect' : list(qrcode.rect)} 
							   # left, top, width, height
	return nodes

def get_node_order(nodes):
	unsorted_nodes = deepcopy(nodes)
	node_order = [] 
	# Get order of nodes

	# unsorted_nodes are all nodes that have not yet been sorted
	# nodes are the set of nodes we are currently considering

	while unsorted_nodes:

		# Find top-leftmost node
		minx = min([value['Rect'][0] for value in nodes.values()])
		miny = min([value['Rect'][1] for value in nodes.values()])

		# Normalize the values to the top left
		for node_name, value in nodes.items():
			nodes[node_name]['Rect'][0] -= minx
			nodes[node_name]['Rect'][1] -= miny

		# Assume top-leftmost activity module is the one closest to the top left corner
		node_sort = sorted(nodes.items(), key=lambda x: x[1]['Rect'][0] + x[1]['Rect'][1])
		node_sort_names = [x[0] for x in node_sort]

		next_node = node_sort_names[0]
		
		# Remove the found node from unsorted_nodes
		next_node_data = unsorted_nodes.pop(next_node)

		# Add the next node to node_order
		node_order.append([next_node, next_node_data])

		# Update miny to be below the most recently found node
		miny = next_node_data['Rect'][1] + next_node_data['Rect'][3]

		# Remove nodes that are above miny
		to_remove = []
		for node_name, value in nodes.items():
			if value['Rect'][1] < miny:
				to_remove.append(node_name)

		for node_name in to_remove:
			nodes.pop(node_name)

		# If there are no more nodes in the column, restart with all unsorted_nodes
		if not nodes:
			nodes = deepcopy(unsorted_nodes)

	# node_order: [ [node_name1, {'Rect': [x,y,width,height], 'Type': activity_or_sensor}], 
	#				...]
	return node_order

def get_activities_and_sensors(node_order):
	# Assume node_order is in the correct order and the activity immediately after
	# a sensor node is the intended reaction
	activity_order = []
	sensor_order = []

	# Indicates whether or not the next activity module is a reaction node
	next_is_sensor_reaction = False
	for i in range(len(node_order)):
		node_type = node_order[i][1]['Type']
		if node_type == 'Sensor':
			sensor_order.append((node_order[i][0], node_order[i+1][0]))
			next_is_sensor_reaction = True

		elif node_type == 'Activity Module':
			if not next_is_sensor_reaction:
				activity_order.append(node_order[i][0])
			else:
				next_is_sensor_reaction = False

	# activity_order: [activity1, activity2, ...]
	# sensor_order: [(sensor1, reaction_activity1), (sensor2, reaction_activity2), ...]
	return activity_order, sensor_order

def make_ltl_spec(activities, sensors):
	all_activities = activities + [x[1] for x in sensors]
	
	########################
	## INPUT PROPOSITIONS ##
	########################

	spec = "[INPUT]\n"
	# Completion nodes for activity modules
	spec += ("###### Completion Nodes ######\n")
	for node_name in all_activities:
		spec += ("{}Complete\n".format(node_name))

	spec += ("###### Sensing Nodes ######\n")
	# Sensor nodes
	for sensor_node, _ in sensors:
		spec += ("{}\n".format(sensor_node))
	spec += ("\n")

	#########################
	## OUTPUT PROPOSITIONS ##
	#########################
	
	spec += ("[OUTPUT]\n")
	for node_name in all_activities:
		# Activity modules
		spec += ("{}\n".format(node_name))

	spec += ("\n")

	############################################
	## ENVIRONMENT INITIALIZATION ASSUMPTIONS ##
	############################################

	spec += ("[ENV_INIT]\n")
	# Assume activities modules do not begin completed
	for node_name in all_activities:
		spec += ("!{}Complete\n".format(node_name))
	# Assume no initial interaction
	for sensor_node, _ in sensors:
		spec += ("!{}\n".format(sensor_node))
	spec += ("\n")

	######################################
	## SYSTEM INITIALIZATION GUARANTEES ##
	######################################
	
	spec += ("[SYS_INIT]\n")
	# Assume robot starts not doing anything
	for node_name in all_activities:
		spec += ("!{}\n".format(node_name))
	spec += ("\n")

	####################################
	## ENVIRONMENT SAFETY ASSUMPTIONS ##
	####################################

	spec += ("[ENV_TRANS]\n")
	# Assume actions that were not started cannot be completed
	spec += ("###### Actions that were not started cannot be completed\n")
	for node_name in all_activities:
		spec += ("(!{}Complete & !{}) -> !{}Complete'\n".format(node_name, node_name, node_name))

	# Assume that actions completed remain completed
	spec += ("###### Actions that completed remain completed\n")
	for node_name in all_activities:
		spec += ("{}Complete -> {}Complete'\n".format(node_name, node_name))
	spec += ("\n")

	##############################
	## SYSTEM SAFETY GUARANTEES ##
	##############################

	spec += ("[SYS_TRANS]\n")
	# Assume mutual exclusion on activities
	spec += ("###### Mutual exclusion on actions\n")
	spec += ("(" + " & ".join(["!{}".format(act) for act in all_activities]) + ")")
	for node_name in all_activities:
		spec += (" | (" + " & ".join(["!{}".format(act) if node_name != act else act for act in all_activities]) + ")")
	spec += ("\n")

	# Order of activity nodes
	spec += ("###### Order of activity nodes\n")
	for first, second in zip(activities, activities[1:]):
		spec += ("!{}Complete -> !{}'\n".format(first, second))
	
	# Reactions to person
	spec += ("###### Reactions to person\n")
	for sensor, reaction in sensors:
		spec += ("({}' & !{}Complete') -> {}'\n".format(sensor, reaction, reaction))
	spec += ("\n")

	######################################
	## ENVIRONMENT LIVENESS ASSUMPTIONS ##
	######################################

	spec += ("[ENV_LIVENESS]\n")
	# Assume that if an activity starts, it completes
	spec += ("###### If an activity starts, it completes\n")
	for node_name in all_activities:
		spec += ("{} -> {}Complete\n".format(node_name, node_name))
	spec += ("\n")

	################################
	## SYSTEM LIVENESS GUARANTEES ##
	################################

	spec += ("[SYS_LIVENESS]\n")
	# The end goal
	spec += ("###### The end goal\n")
	spec += ("{}Complete\n".format(activities[-1]))

	return spec

def save_spec_to_file(ltl_spec, output_path):
	with open(output_path, 'w') as file:
		file.write(ltl_spec)
	print("LTL spec successfully written to {}\n".format(output_path))


if __name__ == "__main__":
	filedir = os.path.dirname(os.path.realpath('__file__'))
	parser = argparse.ArgumentParser()
	parser.add_argument('img_file', type=str,
						 help='Path to the image file with card program')
	parser.add_argument('--output_path', metavar='O', type=str,
						 default=os.path.join(filedir, 'mci_ltl.slugs'),
						 help='Path to save the resulting LTL specification')
	args = parser.parse_args()

	qrcodes = decode_image(args.img_file)
	nodes = get_nodes(qrcodes)
	node_order = get_node_order(nodes)
	activity_order, sensor_order = get_activities_and_sensors(node_order)
	ltl_spec = make_ltl_spec(activity_order, sensor_order)
	save_spec_to_file(ltl_spec, args.output_path)