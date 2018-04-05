add_library(cloud SHARED cloud.cpp rich_point.cpp)
target_link_libraries(cloud
                      projections
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES})

install(TARGETS cloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS cloud
  EXPORT cloudExport
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

add_library(velodyne_utils SHARED velodyne_utils.cpp)
target_link_libraries(velodyne_utils
                      projections
                      cloud
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})

install(TARGETS velodyne_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS velodyne_utils
  EXPORT velodyne_utilsExport
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

find_package(Boost REQUIRED COMPONENTS filesystem regex)
include_directories(${Boost_INCLUDES})
add_library(folder_reader SHARED folder_reader.cpp)
target_link_libraries(folder_reader
                      ${Boost_LIBRARIES})

install(TARGETS folder_reader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS folder_reader
  EXPORT folder_readerExport
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

add_library(bbox bbox.cpp)
target_link_libraries(bbox cloud)

add_library(depth_utils utils.cpp)
target_link_libraries(depth_utils ${OpenCV_LIBS})




