cmake_minimum_required(VERSION 2.4.6)
project(turtlebot_example)
find_package(catkin REQUIRED COMPONENTS std_msgs rospy sensor_msgs geometry_msgs roscpp tf)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#uncomment if you have defined services
#rosbuild_gensrv()
add_message_files(DIRECTORY msg FILES ips_msg.msg)
generate_messages(DEPENDENCIES std_msgs)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
add_executable(turtlebot_example src/turtlebot_example.cpp src/Map.cpp src/marker.cpp src/RRT.cpp src/tracking.cpp)
add_executable(sim_pose_publisher src/sim_pose_publisher.cpp)
add_executable(vis_pose_publisher src/vis_pose_publisher.cpp)
#target_link_libraries(turtlebot_example ${PROJECT_NAME})
target_link_libraries(turtlebot_example
   ${catkin_LIBRARIES}
)

target_link_libraries(sim_pose_publisher
   ${catkin_LIBRARIES}
)

target_link_libraries(vis_pose_publisher
   ${catkin_LIBRARIES}
)
