# Okapi-ROS2
Okapi Localization for ROS2

Make sure ROS2 repository is configured

COMPLETED:
- installed realsense2_camera
- installed realsense2_camera_descriptions
- Migrated CMakeLists.txt and package.xml files for these okapi packages:
  - rfh_controller
  - rfh_handheld_description
  - rfidbot_tags_localization
  - rfidbot_tags_reader

TO-DO:
- colcon build indivduial launch files
- Migrate launch files
- Migrate Scripts 

==============================================================================================================================================================================
STEPS FOR MIGRATING A PACKAGE: (assuming necassary prereqs installed, (see awslabs git))
1. cd ~/ROS1_Package
2. catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
3. make sure compile_commands.json exists within package
4. cd ~/ros2-migration-tools
5. python3 ros_upgrader.py \
   -c ~/ROS1_Package/build/ROS1_Package/compile_commands.json \
   -p ~/ROS1_Package/package.xml
6. Check output folder in ~/ros2-migration-tools for migrated ROS1 package!
