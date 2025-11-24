# Base Simulation for UAV Simulation with ROS2 interface
## Requirements
- Ubuntu 22.04
- ROS2 Humble

## Installation
### PX4 Autopilot
Based on: https://docs.px4.io/main/en/ros2/user_guide

Clone the PX4-Autopilot repository:
```
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
```

Install the PX4-Autopilot:
```
# Run setup script
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# Navigate to the PX4 directory
cd PX4-Autopilot

# Compile the code in SITL mode
make px4_sitl_default
```
Note: This also installs gazebo modules which are not used but this is the safest way to install the software. 

Install and Setup Micro XRCE-DDS Agent and Client:
```
# Clone repository
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

# Build the uXRCE-DDS Agent from source
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib
```

The installation can be verified by starting an agent in a terminal:
```
# Start a uXRCE Agent (port 8888)
MicroXRCEAgent udp4 -p 8888
```

### ROS2 Gz bridge
Based on: https://gazebosim.org/docs/latest/ros_installation/
The ros_gz_bridge is used for mapping gz topics to ROS2 topics.
```
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```



