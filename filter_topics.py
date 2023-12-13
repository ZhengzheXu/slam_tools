#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
ROS bag file filtering script.
Filters out specified topics from a ROS bag file.
"""

import sys
import rosbag
import yaml

def read_filtered_topics_from_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)

def filter_topics(inbag, outbag, filtered_topics):
    total = inbag.get_message_count()
    count = 0
    for topic, msg, t in inbag.read_messages():
        if topic not in filtered_topics:
            outbag.write(topic, msg, t)
        count += 1
        if count % 10000 == 0:
            print("Copying data to output bag: {}/{}".format(count, total))

def main():
    if len(sys.argv) < 3:
        print("Usage: script.py input_bag_file.yaml output_bag_file.bag")
        sys.exit(1)

    yaml_file = sys.argv[1]
    filtered_topics = read_filtered_topics_from_yaml(yaml_file)
    filtered_topics = filtered_topics['filtered_topics']

    for bag_file in sys.argv[2:]:
        outfile = bag_file[:-4] + "_filtered.bag"
        print("-------------------- Topics to filter ------------------------------")
        for topic in filtered_topics:
            print(topic)
        with rosbag.Bag(outfile, 'w') as outbag, rosbag.Bag(bag_file) as inbag:
            print("-------------------- Input: %s ------------------------------" % bag_file)
            print(inbag)
            print("-------------------- Filtering topics ------------------------------")
            filter_topics(inbag, outbag, filtered_topics)
            print("------------------- Output: %s ------------------------------" % outfile)
            print(outbag)

if __name__ == '__main__':
    main()