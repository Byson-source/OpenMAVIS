cmake_minimum_required(VERSION 3.10)
project(openmavis_ros2)

# C++17, no -march=native (build node and Euler run node may be different CPUs).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Headless build: the e2e pipeline never opens a viewer, and the Euler apptainer
# SIF has no Pangolin. ORB_HEADLESS compiles out all Pangolin/OpenGL code
# (Viewer::Run body, MapDrawer draw methods) so the fork builds on any container.
add_compile_definitions(ORB_HEADLESS)

# Path to the OpenMAVIS (ORB-SLAM3 fork) core (this package lives at <ORB3>/ros2/openmavis_ros2).
get_filename_component(ORB3 "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(okvis_pose_graph_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
  cv_bridge image_transport okvis_pose_graph_msgs
  OpenCV Eigen3)

# Build the bundled Thirdparty projects in-tree so the whole thing comes up with
# a single colcon build (no separate build.sh step).
add_subdirectory(${ORB3}/Thirdparty/DBoW2 ${CMAKE_CURRENT_BINARY_DIR}/DBoW2_build)
add_subdirectory(${ORB3}/Thirdparty/g2o   ${CMAKE_CURRENT_BINARY_DIR}/g2o_build)

include_directories(
  ${ORB3}
  ${ORB3}/include
  ${ORB3}/include/CameraModels
  ${ORB3}/Thirdparty
  ${ORB3}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# ORB-SLAM3 core compiled directly into a shared library (proven container recipe).
add_library(openmavis_lib SHARED
  ${ORB3}/src/System.cc
  ${ORB3}/src/Tracking.cc
  ${ORB3}/src/LocalMapping.cc
  ${ORB3}/src/LoopClosing.cc
  ${ORB3}/src/ORBextractor.cc
  ${ORB3}/src/ORBmatcher.cc
  ${ORB3}/src/FrameDrawer.cc
  ${ORB3}/src/Converter.cc
  ${ORB3}/src/MapPoint.cc
  ${ORB3}/src/KeyFrame.cc
  ${ORB3}/src/Atlas.cc
  ${ORB3}/src/Map.cc
  ${ORB3}/src/MapDrawer.cc
  ${ORB3}/src/Optimizer.cc
  ${ORB3}/src/Frame.cc
  ${ORB3}/src/KeyFrameDatabase.cc
  ${ORB3}/src/Sim3Solver.cc
  ${ORB3}/src/Viewer.cc
  ${ORB3}/src/ImuTypes.cc
  ${ORB3}/src/G2oTypes.cc
  ${ORB3}/src/CameraModels/Pinhole.cpp
  ${ORB3}/src/CameraModels/KannalaBrandt8.cpp
  ${ORB3}/src/OptimizableTypes.cpp
  ${ORB3}/src/MLPnPsolver.cpp
  ${ORB3}/src/GeometricTools.cc
  ${ORB3}/src/TwoViewReconstruction.cc
  ${ORB3}/src/Config.cc
  ${ORB3}/src/Settings.cc
)
target_link_libraries(openmavis_lib
  ${OpenCV_LIBS}
  DBoW2
  g2o
  -lboost_serialization
  -lcrypto
)

# ROS2 mono-inertial front-end node.
add_executable(hilti_mono_inertial_node
  src/hilti_mono_inertial_node.cpp
  src/pose_graph_publisher.cpp
  src/edge_hessian.cpp
)
ament_target_dependencies(hilti_mono_inertial_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(hilti_mono_inertial_node openmavis_lib)

install(TARGETS openmavis_lib DBoW2 g2o
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS hilti_mono_inertial_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
