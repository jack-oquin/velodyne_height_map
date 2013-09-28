cmake_minimum_required(VERSION 2.8.3)
project(velodyne_height_map)

set(${PROJECT_NAME}_CATKIN_DEPS
    nodelet
    roscpp
    pcl_ros
    sensor_msgs
    tf)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS}
             angles
             pcl_conversions
             roslaunch rostest)
find_package(Boost)
find_package(PCL REQUIRED QUIET COMPONENTS common)

include_directories(include
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIR})

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

add_executable(heightmap_node src/heightmap_node.cpp src/heightmap.cpp)
target_link_libraries(heightmap_node
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES})

add_library(heightmap_nodelet src/heightmap_nodelet.cpp src/heightmap.cpp)
target_link_libraries(heightmap_nodelet
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES})

# install the files needed to run this package
install(TARGETS heightmap_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS heightmap_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# unit tests
if (CATKIN_ENABLE_TESTING)
  message("-- ${PROJECT_NAME} unit testing enabled")

  # Download a packet capture (PCAP) file containing test data.
  # Store it in devel-space, so rostest can easily find it.
  catkin_download_test_data(
    ${PROJECT_NAME}_tests_class.pcap
    http://download.ros.org/data/velodyne/class.pcap
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 65808d25772101358a3719b451b3d015)
  
  # declare rostest scripts
  add_rostest(tests/heightmap_node_hz.test)
  add_rostest(tests/heightmap_nodelet_hz.test)
  
  # parse check all the launch/*.launch files
  ##roslaunch_add_file_check(launch)

else ()
  message("-- ${PROJECT_NAME} unit testing disabled")
endif (CATKIN_ENABLE_TESTING)
