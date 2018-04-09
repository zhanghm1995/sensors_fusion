cmake_minimum_required(VERSION 2.8.3)
project(sensors_fusion)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

set(PACKAGE_DEPENDENCIES
  eigen_conversions
  geometry_msgs
  message_runtime
  nav_msgs
  rosbag
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_eigen
  tf2_ros
  urdf
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(ivcommon REQUIRED)

find_package(sensor_driver REQUIRED)
set(BUILD_SHARED_LIBS OFF)
include("${IVCOMMON_CMAKE_DIR}/functions.cmake")
google_initialize_ivcommon_project()

install(DIRECTORY launch DESTINATION share/test/)
install(DIRECTORY rviz_cfg DESTINATION share/test/)

include(FindPkgConfig)
find_package(LuaGoogle REQUIRED)
find_package(PCL REQUIRED)
#set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)

#install(DIRECTORY launch DESTINATION share/covgrid_slam_ros/)
#install(DIRECTORY rviz_cfg DESTINATION share/covgrid_slam_ros/)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS src
)
include_directories(SYSTEM  ${catkin_INCLUDE_DIRS} ${ivcommon_INCLUDE_DIRS} ${sensordriver_INCLUDE_DIRS} ${LUA_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} 
					${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} src include)

add_executable(data_sync src/data_fusion/data_sync.cpp)
target_link_libraries(data_sync PUBLIC ${catkin_LIBRARIES}  ${ivcommon_LIBRARIES}  ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  ${OpenCV_LIBS} ${sensor_driver_INCLUDE_DIRS} utils ivsensor glog gflags)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} -std=c++11")
set_target_properties(data_sync PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(TARGETS data_sync
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS data_sync
  EXPORT sensors_fusionExport
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
