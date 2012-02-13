rosbuild_add_executable(image circular_image.cc)
target_link_libraries(image circular_image)

rosbuild_add_executable(image2 overhead_image.cc)
target_link_libraries(image2 overhead_image)

#rosbuild_add_executable(obstacle obstacle.cc)
#target_link_libraries(obstacle velodyne_image)
