cmake_minimum_required(VERSION 3.8)
project(visual_slam)

set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(compressed_depth_image_transport REQUIRED)

#${ORB_SLAM3_ROOT_DIR}
include_directories(
  include
  /home/jpalves/ORB_SLAM3/include
  /home/jpalves/ORB_SLAM3/include/CameraModels
  #/home/jpalves/ORB_SLAM3/include/CameraModels/GeometricCamera.h
  /home/jpalves/ORB_SLAM3/Thirdparty/Sophus
)

add_executable(mono src/mono.cpp)

ament_target_dependencies(mono 
  rclcpp
  sensor_msgs
  OpenCV
  ORB_SLAM3
  Pangolin
  Eigen3
  tf2_ros 
  tf2_geometry_msgs 
  tf2
  nav_msgs
  PCL 
)

add_executable(rgbd src/rgbd.cpp src/rgbd-slam-node.cpp)
ament_target_dependencies(rgbd 
  rclcpp
  sensor_msgs
  compressed_depth_image_transport
  OpenCV
  cv_bridge
  ORB_SLAM3
  Pangolin
  Eigen3
  tf2_ros 
  tf2_geometry_msgs 
  tf2
  nav_msgs
  PCL 
)

target_include_directories(mono PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(mono PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS mono rgbd
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
