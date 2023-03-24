cmake_minimum_required(VERSION 2.8.3)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(perception_lidar)

# note major changes:
# rclcpp does not exist in ros, consider using roscpp
# rclpy does not exist in ros, consider using rospy
# std_msgs exsists in ros but may have slight functionality changes
# sensor_msgs exsists in ros but may have slight functionality changes
# message_filters exsists in ros but may have slight functionality changes
# tf2_geometry_msgs exsists in ros but may have slight functionality changes
# tf2 exsists in ros but may have slight functionality changes
# tf2_ros exsists in ros but may have slight functionality changes

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS
	roscpp #rclcpp does not exsist in ros1 need similar functionality
	rospy #rclpy does not exsist in ros1 need similar functionality
	std_msgs
	sensor_msgs
	message_filters
	tf2_geometry_msgs
	pcl_conversions
	pcl_ros
)

find_package(PCL 1.7 REQUIRED)

include_directories(${roslib_INCLUDE_DIRS})

include_directories(
${catkin_INCLUDE_DIRS}
# ${PCL_INCLUDE_DIRS}
# ${catkin_INCLUDE_DIRS}/include/**
)

catkin_package(
        INCLUDE_DIRS  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS}
#  LIBRARIES vmxpi_ros_bringup
	CATKIN_DEPENDS  roscpp rospy pcl_conversions pcl_ros
#	DEPENDS system_lib
)




# find_file(TF2_CPP_HEADERS
#         NAMES tf2_geometry_msgs.hpp
#         PATHS ${tf2_geometry_msgs_INCLUDE_DIRS}
#         NO_CACHE
#         PATH_SUFFIXES tf2_geometry_msgs
#         )




link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(intensity_filter src/intensity_filter.cpp)
target_link_libraries(intensity_filter 
${catkin_LIBRARIES}
${roslib_LIBRARIES}
${PCL_LIBRARIES}
)

add_executable(planar_filter src/planar_filter.cpp)
target_link_libraries(planar_filter
${catkin_LIBRARIES}
${roslib_LIBRARIES}
${PCL_LIBRARIES}
)

add_executable(polygon_filter src/polygon_filter.cpp)
target_link_libraries(polygon_filter
${catkin_LIBRARIES}
${roslib_LIBRARIES}
${PCL_LIBRARIES}
)

install(
    TARGETS
    intensity_filter
    planar_filter
    polygon_filter

    DESTINATION
    lib/${PROJECT_NAME}
#    lib/planar_filter
)

# Install Python modules
catkin_install_python(PROGRAMS scripts/inference.py DESTINATION ${PROJECT_NAME})

catkin_python_setup()

# install python executables
install(
    PROGRAMS
    scripts/inference.py

    DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

#if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
#endif()

#ament_package()

