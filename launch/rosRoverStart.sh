#!/usr/bin/env bash
HOME="/home/nvidia"
REPO_HOME="$HOME/robotics-orin"
OPT_HUMBLE_SETUP="/opt/ros/humble/setup.bash"
ROS_PACKAGES_SETUP="$REPO_HOME/install/local_setup.sh"
ROSLAUNCH_FILE="$REPO_HOME/launch/robot_wheels.py"
CANBUS_SETUP_FILE="$REPO_HOME/configure-can0.sh"

# source venv
cd $REPO_HOME

#source "$REPO_HOME/venv/bin/activate"
source ~/.bashrc

python3 "$REPO_HOME/setup.py" develop
PYTHONPATH=$PYTHONPATH:/home/$USER/Programming/robotics-prototype
PYTHONPATH=$PYTHONPATH:/home/$USER/robotics-orin

# Setup canbus


# source primary catkin_ws setup bash script and execute one launch script to rule them all
source $OPT_HUMBLE_SETUP && source $ROS_PACKAGES_SETUP && ros2 launch $ROSLAUNCH_FILE
