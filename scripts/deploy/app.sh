#!/bin/bash

echo "Hello App.sh!"

ros2 launch limo_simulation limo.launch.py

# After the ROS nodes have been terminated, the error plot should be generated
python3 /root/generate_error_plot.py

# > target_error.csv