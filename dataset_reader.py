#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import os

class DatasetReader:
    def __init__(self, dataset_dir):
        self.dataset_dir = dataset_dir

    def read_rgb_data(self):
        rgb_file = os.path.join(self.dataset_dir, "rgb.txt")
        rgb_timestamps = []
        rgb_dict = {}
        with open(rgb_file, "r") as file:
            for line in file:
                if line.startswith("#"):
                    continue
                line = line.strip()
                timestamp = float(line.split()[0])
                rgb_timestamps.append(timestamp)
                rgb_dict[timestamp] = line.split()[1]
        return np.array(rgb_timestamps), rgb_dict

    def read_depth_data(self):
        depth_file = os.path.join(self.dataset_dir, "depth.txt")
        depth_timestamps = []
        depth_dict = {}
        with open(depth_file, "r") as file:
            for line in file:
                if line.startswith("#"):
                    continue
                line = line.strip()
                timestamp = float(line.split()[0])
                depth_timestamps.append(timestamp)
                depth_dict[timestamp] = line.split()[1]
        return np.array(depth_timestamps), depth_dict

    def read_groundtruth_data(self):
        groundtruth_file = os.path.join(self.dataset_dir, "groundtruth.txt")
        groundtruths = []
        with open(groundtruth_file, "r") as file:
            for line in file:
                if line.startswith("#"):
                    continue
                line = line.strip()
                groundtruths.append([float(x) for x in line.split()])
        return np.array(groundtruths)

    def read_groundtruth_rgb_data(self):
        groundtruth_rgb_file = os.path.join(self.dataset_dir, "groundtruth_rgb.txt")
        gt_rgb = []
        with open(groundtruth_rgb_file, "r") as file:
            for line in file:
                if line.startswith("#"):
                    continue
                line = line.strip()
                gt_rgb.append([float(x) for x in line.split()])
        return np.array(gt_rgb)

# 用法示例:
# dataset_reader = DatasetReader("path_to_dataset")
# rgb_timestamps, rgb_dict = dataset_reader.read_rgb_data()
# depth_timestamps, depth_dict = dataset_reader.read_depth_data()
# groundtruths = dataset_reader.read_groundtruth_data()
# gt_rgb = dataset_reader.read_groundtruth_rgb_data()