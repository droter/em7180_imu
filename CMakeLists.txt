cmake_minimum_required(VERSION 2.8.3)
project(em7180_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs  
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs visualization_msgs 
)

catkin_install_python(
  PROGRAMS nodes/imu_driver_node nodes/imu_viz_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

