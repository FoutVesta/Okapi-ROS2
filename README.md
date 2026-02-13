# Okapi-ROS2
Okapi Localization for ROS2 Humble Distribution

# Camera Setup

1. Install the RealSense SDK
```bash
  sudo apt install ros-humble-librealsense2*
```
3. Install ROS Wrapper for RealSense Cameras (from source since debian packages doesnt work for some reason)
```bash
  mkdir -p ~/realsense_ws/src
  cd ~/realsense_ws/src/
  git clone https://github.com/realsenseai/realsense-ros.git -b ros2-master
  cd ~/realsense_ws
```
5. Install dependencies
```bash
  sudo apt-get install python3-rosdep -y
  sudo rosdep init 
  rosdep update 
  rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
```
7. Build
```bash
  colcon build
```
9. Source Environment
```bash
  ROS_DISTRO=humble
  source /opt/ros/$ROS_DISTRO/setup.bash
  cd ~/realsense_ws
  . install/local_setup.bash
```

Bluetooth for RFD8500 Reader (if needed)
```bash
  sudo apt update
  sudo apt install bluetooth bluez bluez-tools rfkill -y
```

Imu Madgwick Filter 
```bash
  sudo apt install ros-humble-imu-filter-madgwick
```

# Cloning and building the Repository
```bash
  mkdir -p ~/okapiros2_ws
  cd ~/okapiros2_ws
  git clone https://github.com/FoutVesta/Okapi-ROS2.git
  cd ~/ros2_okapi/src
  colcon build
  source /opt/ros/humble/setup.bash
```

# Running
1. Run this in first terminal
```bash
  source install/setup.bash
  ros2 launch rfh_controller rfh_realsensemapping_bringup.launch.xml
```
3. Run this in second terminal
```bash
  source install/setup.bash
  ros2 launch rfh_controller rfh_rfid_bringup.launch.xml
```

# NOTES
- Because only the D435i is the only camera in use for this build, it can be quite slow because its being used to capture all needed data, so it cannot be moved very fast. I will work on a branch that uses Foxy distribution so we can continue using the t265.
