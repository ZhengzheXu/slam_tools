# SLAM Tools 
 
This repository, `slam_tools`, is a collection of utility scripts and tools designed to facilitate various operations related to SLAM. It currently includes scripts for merging IMU data in ROS bag files, and it is planned to be expanded with more tools in the future.
 
## Contents
- `merge_imu_topics.py`     : The original script for merging accelerometer and gyroscope data from ROS bag files.
- `fast_merge_imu_topics.py`: An optimized version of the original script, using numpy and scipy for improved performance.
- `filter_topics.py`        : A script for filtering topics from a ROS bag file.

### 1. merge_imu_topics.py
This script merges data from the `/d400/accel/sample` and `/d400/gyro/sample` topics of a ROS bag file into a more commonly used IMU data format `/d400/imu0`. This format is particularly useful for running visual SLAM algorithms like VINS-Mono.
 
#### Usage
To run the script, use the following command: 
```bash
python merge_imu_topics.py <bag_file_1> <bag_file_2> ...
```

Replace `<bag_file_1>`, `<bag_file_2>`, etc. with the names of the bag files you wish to process. 
 
### 2. fast_merge_imu_topics.py
 
An improved version of the original `merge_imu_topics.py` script. This script is significantly faster, thanks to the utilization of numpy and scipy libraries for efficient data handling. The usage is similar to the original script.
 
#### Features
- Enhanced performance for large data sets.
- Maintains the original timestamps from accelerometer and gyroscope data, leading to increased data frequency in the merged output.
- Compatible with standard visual SLAM algorithms like VINS-Mono.
 
#### Usage
Similar to the original script, run the following command: 
```bash
python fast_merge_imu_topics.py <bag_file_1> <bag_file_2> ...
```

Ensure that numpy and scipy are installed in your environment. If not, install them using the following commands:
```bash
pip install numpy scipy
```
 
#### Note
- The script may produce a "imu message in disorder!" warning when used with VINS. 
This warning indicates non-monotonically increasing timestamps in the IMU data. 
However, this does not affect the functionality of the algorithms. 
This issue is also present in the original script and is slated for future resolution.

### 3. filter_topics.py
This script is designed to filter out specified topics from ROS bag files. It's particularly useful in scenarios where you need to process or analyze bag files without certain topics. 

### Features
- Ability to filter out specific topics from ROS bag files.
- Reads the list of topics to be filtered from a YAML file, allowing for easy customization.
- Suitable for preprocessing bag files for various SLAM and robotics applications.

### Usage
To use the script, you need to have a YAML file that contains the list of topics to be filtered. The format of the YAML file should be as follows:

```yaml
filtered_topics:
  - /example_topic1
  - /example_topic2
  # Add more topics as needed
```

After preparing the YAML file, run the script using the following command:
```bash
python ros_bag_filter.py <yaml_file> <input_bag_file_1> <input_bag_file_2> ...
```

Replace `<yaml_file>` with the name of the YAML file containing the list of topics to be filtered.
Replace `<input_bag_file_1>`, `<input_bag_file_2>`, etc. with the names of the bag files you wish to process.

## Future Additions
More utility scripts and tools will be added to this repository as they are developed. 
These tools aim to streamline various aspects of SLAM processing and data manipulation.
 
## Contributions
Contributions to `slam_tools` are welcome. 
Whether it's a new utility script, bug fixes, or enhancements to existing tools, please feel free to submit a pull request or open an issue.
 
## License
This project is licensed under the MIT License - see the LICENSE file for details.
