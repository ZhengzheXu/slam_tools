#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
Fastly merge gyro and accel topics into one rosbag file with linear interpolation.
"""

import rospy
import rosbag
import sys
import numpy as np
import scipy.interpolate as interp
from sensor_msgs.msg import Imu


def merge_gyro_accel(file, file_index):
    """ Merge gyro and accel data from a ROS bag file. """
    outfile = "vins{}.bag".format(file_index)
    outbag = rosbag.Bag(outfile, 'w')

    # Initialize lists to store data
    d400_accer, d400_gyr = [], []
    d400_accer_data, d400_gyr_data = [], []

    with rosbag.Bag(file) as inbag:
        print("-------------------- Input: {} ------------------------------".format(file))
        print(inbag)
        msg_num = inbag.get_message_count()

        for count, (topic, msg, t) in enumerate(inbag.read_messages()):
            # If topic is not imu, copy it to output bag
            if topic == "/d400/imu0":
                # This means we have already merged gyro and accel data
                continue

            # Copy all topics to output bag
            outbag.write(topic, msg, t)

            # Store gyro and accel data
            if topic == "/d400/accel/sample":
                process_sensor_data(d400_accer, d400_accer_data, msg, t, 'linear_acceleration')
            elif topic == "/d400/gyro/sample":
                process_sensor_data(d400_gyr, d400_gyr_data, msg, t, 'angular_velocity')

            # Print progress
            if count % 10000 == 0:
                print("Copying data to output bag: {}/{}".format(count, msg_num))

        print("Start to merge gyro and accel topics with linear interpolation.")
        process_interpolation(d400_accer, d400_accer_data, d400_gyr, d400_gyr_data, outbag)

    outbag.close()
    print("------------------- Output: {} ------------------------------".format(outfile))
    print(rosbag.Bag(outfile))


def process_sensor_data(sensor_list, sensor_data_list, msg, t, attr_name):
    """ Process sensor data for gyro or accel. """
    sensor_list.append(msg)
    sensor_time = rospy.Time.to_sec(t)
    sensor_data_list.append((
        getattr(msg, attr_name).x,
        getattr(msg, attr_name).y,
        getattr(msg, attr_name).z,
        sensor_time
    ))


def process_interpolation(d400_accer, d400_accer_data, d400_gyr, d400_gyr_data, outbag):
    """ Interpolate and merge gyro and accel data. """
    d400_accer_data, d400_gyr_data = np.array(d400_accer_data), np.array(d400_gyr_data)
    accer_times, gyr_times = d400_accer_data[:, -1], d400_gyr_data[:, -1]

    # First step: Interpolate gyr data on accer timestamps
    print("Interpolating gyro data on accel timestamps.")
    interpolate_and_merge(d400_accer, d400_accer_data, d400_gyr_data, outbag, "/d400/imu0", 'angular_velocity')

    # Second step: Interpolate accer data on gyr timestamps
    print("Interpolating accel data on gyro timestamps.")
    interpolate_and_merge(d400_gyr, d400_gyr_data, d400_accer_data, outbag, "/d400/imu0", 'linear_acceleration')


def interpolate_and_merge(sensor_list, sensor_data, other_sensor_data, outbag, topic, attr_name):
    """ Interpolate sensor data and merge into output bag. """

    # Filter out invalid data
    valid_indices = (sensor_data[:, -1] >= other_sensor_data[:, -1].min()) & (sensor_data[:, -1] <= other_sensor_data[:, -1].max())
    sensor_data = sensor_data[valid_indices]
    sensor_list = [sensor_list[i] for i in range(len(sensor_list)) if valid_indices[i]]

    # Interpolate sensor data
    func_x = interp.interp1d(other_sensor_data[:, -1], other_sensor_data[:, 0], kind='linear')
    func_y = interp.interp1d(other_sensor_data[:, -1], other_sensor_data[:, 1], kind='linear')
    func_z = interp.interp1d(other_sensor_data[:, -1], other_sensor_data[:, 2], kind='linear')

    # Merge sensor data
    for count, (sensor, t) in enumerate(zip(sensor_list, sensor_data[:, -1])):
        sensor_attr = getattr(sensor, attr_name)
        sensor_attr.x = func_x(t)
        sensor_attr.y = func_y(t)
        sensor_attr.z = func_z(t)

        outbag.write(topic, sensor, rospy.Time.from_sec(t))
        if count % 1000 == 0:
            print("Writing data to output bag: {}/{}".format(count, len(sensor_data)))


if __name__ == '__main__':
    for index, file in enumerate(sys.argv[1:], start=1):
        merge_gyro_accel(file, index)