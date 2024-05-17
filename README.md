### Setup
The code in this repo was built around ROS Humble. First [install that](https://docs.ros.org/en/humble/Installation.html).
Then, from this folder:
- I recommend you setup a [Python venv](https://docs.python.org/3/library/venv.html). See steps below.
- Install rosdep, colcon, catkin, and pip (if not already) (`sudo apt install python3-colcon-common-extensions catkin_pkg python3-pip python3-rosdep2`)
- Update rosdep (`rosdep update`)
- Run rosdep so it installs packages: `rosdep install --from-paths src --ignore-src -r -y`. Enter your password when prompted.
- Install misc python deps (`pip install -r requirements.txt`)
- Install JetsonGPIO [from GitHub](https://github.com/pjueon/JetsonGPIO/blob/master/docs/installation_guide.md). This must be 
manually installed. The default options will work.
- Build: `colcon build --symlink-install --packages-skip usb_cam` (usb_cam can only compile on the Jetson, no easy workaround for it).

#### Setup venv
Run `python3 -m venv ./space-env` from the robotics-orin folder.

So you use it automatically, add the source to your ~/.bashrc: `echo "source ${PWD}/space-env/bin/activate" >> ~/.bashrc`

#### Setting up ros2_aruco
To get this working, you may see the following error:
```
Installing the transforms3d library by hand required. Please run
        sudo pip3 install transforms3d
```
Run this command (in my case without the sudo and with pip) and it should work.
Adding to PYTHONPATH is likely necessary. You will add a command like the following: 
`export PYTHONPATH="/home/marc/Programming/robotics-orin/space-env/lib/python3.10/site-packages:$PYTHONPATH"`