# Okapi-ROS2 (T265 + D435i)
Okapi Localization for ROS2 Humble Distribution

This branch of Okapi ROS2 allows for functionality of the T255 camera alongside the D435i by using an older distribution of the RealSense SDK (2.51) and Wrapper (4.51). 

## Camera Setup

1. Install and Build the RealSense SDK from source (v2.51.1)
```bash
sudo apt update
sudo apt install -y \
    git cmake build-essential \
    libssl-dev libusb-1.0-0-dev pkg-config \
    libgtk-3-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    libudev-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.51.1
```
From here, go into /librealsense/src/libusb/libusb.h
You'll find code that looks like
```bash
#if 0
  //A bunch of warning stuff
#endif
```
Delete everything in between the #if 0 and #endif, because it causes a libusb error upon building

Once you've done that

```bash
mkdir -p build
mkdir -p ~/opt/librealsense-2.51.1
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/opt/librealsense-2.51.1 \
  -DBUILD_EXAMPLES=true \
  -DFORCE_RSUSB_BACKEND=ON
make -j1
make install

cd
echo 'export LD_LIBRARY_PATH=$HOME/opt/librealsense-2.51.1/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PATH=$HOME/opt/librealsense-2.51.1/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```


2. Install and Build the ROS Wrapper for RealSense Cameras from source (v4.51.1)
```bash
  mkdir -p ~/realsense_ws/src
  cd ~/realsense_ws/src/
  git clone https://github.com/realsenseai/realsense-ros.git -b 4.51.1

  source /opt/ros/humble/setup.bash
  sudo apt-get install -y python3-rosdep
  sudo rosdep init || true
  rosdep update
  rosdep install -i --from-path src --rosdistro humble --skip-keys=librealsense2 -y

  colcon build
```
5. Source Environment
```bash
  ROS_DISTRO=humble
  source /opt/ros/$ROS_DISTRO/setup.bash
  cd ~/realsense_ws
  . install/local_setup.bash
```

6. Bluetooth for RFD8500 Reader (if needed)
```bash
  sudo apt update
  sudo apt install bluetooth bluez bluez-tools rfkill -y
```

## Cloning and building the Repository
```bash
  mkdir -p ~/okapiros2_ws
  cd ~/okapiros2_ws
  git clone -b OkapiROS2-+-D435i https://github.com/FoutVesta/Okapi-ROS2.git
  cd ~/ros2_okapi
  colcon build
  source /opt/ros/humble/setup.bash
```

## Running
1. Run this in first terminal
```bash
  source install/setup.bash
  ros2 launch rfh_controller rfh_realsensemapping_t265_bringup.launch.xml
```
3. Run this in second terminal
```bash
  source install/setup.bash
  ros2 launch rfh_controller rfh_rfid_bringup.launch.xml
```

# NOTES

- Upon launching the realsensemapping_bringup, sometimes there is a usb interference error that causes both cameras to not work. If that happens, terminate and try again. 
- One thing I've noticed when localizing tags is that the marker position in Rviz relative to the actual tag is correct, but moved back slightly. I've made sure that the frames are set correctly and they are, but for some reason there is still some slight offset of the tags. My temporary solution to this is by applying -0.25 change in distance in the x direction to offset this and it works good. Might want to work on this later though.
- You can tweak the settings of the D435i via line 79 in the rfh_realsensemapping_bringup.launch.xml to make the camera better handle faster motion. It's not perfect though, so using the t265 is the preferable choice.
- Because the camera is not good at handling fast movements it does't capture positional data as well as it should, which causes localization of tags to be off by a little bit sometimes.
