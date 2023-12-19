#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
ROS bag file creation script.
Converts a series of RGB and depth images into a ROS bag file.
"""

import os
import sys
import rospy
import rosbag
from cv_bridge import CvBridge
import cv2

def create_rosbag(input_dir, output_bag_name):
    bridge = CvBridge()
    bag = rosbag.Bag(output_bag_name, 'w')

    rgb_dir = os.path.join(input_dir, 'rgb')
    depth_dir = os.path.join(input_dir, 'depth')

    rgb_files = sorted([f for f in os.listdir(rgb_dir) if f.endswith('.png')])
    depth_files = sorted([f for f in os.listdir(depth_dir) if f.endswith('.png')])
    img_num = len(rgb_files) + len(depth_files)
    count = 0

    for rgb_file in rgb_files:
        if count % 100 == 0:
            print("Processing {} / {}".format(count, img_num))

        # Extract timestamp from the file name
        timestamp = float(os.path.splitext(rgb_file)[0])
        time = rospy.Time.from_sec(timestamp)

        # Read RGB image
        rgb_path = os.path.join(rgb_dir, rgb_file)
        rgb_img = cv2.imread(rgb_path)
        rgb_msg = bridge.cv2_to_imgmsg(rgb_img, encoding="bgr8")
        rgb_msg.header.stamp = time
        bag.write('camera/rgb/image_raw', rgb_msg, time)
        count += 1


    for depth_file in depth_files:
        if count % 100 == 0:
            print("Processing {} / {}".format(count, img_num))

        # Extract timestamp from the file name
        timestamp = float(os.path.splitext(depth_file)[0])
        time = rospy.Time.from_sec(timestamp)

        # Read depth image
        depth_path = os.path.join(depth_dir, depth_file)
        depth_img = cv2.imread(depth_path, -1)
        depth_msg = bridge.cv2_to_imgmsg(depth_img, encoding="passthrough")
        depth_msg.header.stamp = time
        bag.write('camera/depth/image_raw', depth_msg, time)
        count += 1
    
    bag.close()
    print("------------------- Output: {} ------------------------------".format(output_bag_name))
    print(rosbag.Bag(output_bag_name))

def main():
    if len(sys.argv) < 2:
        print("Usage: images2bag.py <input_directory>")
        sys.exit(1)

    input_dir = sys.argv[1]
    dataset_name = os.path.basename(input_dir.rstrip('/'))
    output_bag_name = dataset_name + '.bag'

    create_rosbag(input_dir, output_bag_name)

if __name__ == '__main__':
    main()