#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Author: Zhengzhe Xu
# Email: xuzhengzhe810@gmail.com

import os
import tkinter as tk
from tkinter import filedialog
import rosbag
import pandas as pd
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

def select_bag_file():
    filepath = filedialog.askopenfilename(title="Chose ROS Bag file", filetypes=(("ROS Bag files", "*.bag"), ("All files", "*.*")))
    if filepath:
        extract_data(filepath)

def extract_data(bag_file):
    data_odom = []
    data_pose = []

    bag_directory = os.path.dirname(bag_file)
    odom_csv_file = os.path.join(bag_directory, 'orb_body_odom.csv')
    pose_csv_file = os.path.join(bag_directory, 'orb_camera_pose.csv')

    if os.path.exists(odom_csv_file) or os.path.exists(pose_csv_file):
        label.config(text="CSV files already exist! Please check the file.")
        return

    # Read ROS Bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/orb_slam3/body_odom', '/orb_slam3/camera_pose']):
            timestamp = t.to_nsec()
            if topic == '/orb_slam3/body_odom':
                data_odom.append({
                    'timestamp': timestamp,
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z,
                    'qx': msg.pose.pose.orientation.x,
                    'qy': msg.pose.pose.orientation.y,
                    'qz': msg.pose.pose.orientation.z,
                    'qw': msg.pose.pose.orientation.w
                })
            elif topic == '/orb_slam3/camera_pose':
                data_pose.append({
                    'timestamp': timestamp,
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z,
                    'qx': msg.pose.orientation.x,
                    'qy': msg.pose.orientation.y,
                    'qz': msg.pose.orientation.z,
                    'qw': msg.pose.orientation.w
                })

    df_odom = pd.DataFrame(data_odom)
    df_pose = pd.DataFrame(data_pose)

    # Save to CSV
    df_odom.to_csv(odom_csv_file, index=False, header=None)
    df_pose.to_csv(pose_csv_file, index=False, header=None)
    label.config(text="CSV files saved successfully!")

root = tk.Tk()
root.title("ROS Bag to EuRoC CSV Converter")

tk.Button(root, text="Choose ROS Bag file", command=select_bag_file).pack(pady=20)
label = tk.Label(root, text="")
label.pack(pady=20)

root.mainloop()
