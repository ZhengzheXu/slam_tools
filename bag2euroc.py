#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Author: Zhengzhe Xu
# Email: xuzhengzhe810@gmail.com

import tkinter as tk
from tkinter import filedialog
import rosbag
import pandas as pd
from nav_msgs.msg import Odometry
import os

def select_bag_file():
    filepath = filedialog.askopenfilename(title="Select ROS Bag file", filetypes=(("ROS Bag files", "*.bag"), ("All files", "*.*")))
    if filepath:
        extract_data(filepath)

def extract_data(bag_file):
    data = []

    # Read ROS Bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/odometry']):
            timestamp = t.to_nsec() // 1000
            data.append({
                'timestamp': timestamp,
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z,
                'qx': msg.pose.pose.orientation.x,
                'qy': msg.pose.pose.orientation.y,
                'qz': msg.pose.pose.orientation.z,
                'qw': msg.pose.pose.orientation.w
            })

    df = pd.DataFrame(data)
    output_file = os.path.join(os.path.dirname(bag_file), 'odometry_euroc.csv')

    # Save DataFrame to CSV
    df.to_csv(output_file, index=False, header=None)
    label.config(text=f"CSV file saved to {output_file}")

root = tk.Tk()
root.title("ROS Bag to EuRoC CSV Converter")

tk.Button(root, text="Select ROS Bag file", command=select_bag_file).pack(pady=20)
label = tk.Label(root, text="")
label.pack(pady=20)

root.mainloop()