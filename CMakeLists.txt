cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_grid)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 4 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# find_package(catkin REQUIRED COMPONENTS
#   dynamic_reconfigure
#   sensor_msgs
#   nav_msgs
#   pcl_ros
#   roscpp
# )


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/MyParams.cfg
# )

# catkin_package(
#   INCLUDE_DIRS include
#   #LIBRARIES pointcloud_to_grid
#   #CATKIN_DEPENDS dynamic_reconfigure nav_msgs pcl_ros roscpp
#   #DEPENDS roscpp std_msgs
# )
  
  
## Specify additional locations of header files
## Your package locations should be listed before other locations
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

include_directories(
  include
  # ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
  
add_executable(pointcloud_to_grid_node src/pointcloud_to_grid_node.cpp)
ament_target_dependencies(pointcloud_to_grid_node rclcpp ${OpenCV_LIBS} ${PCL_LIBRARIES}) 
#add_dependencies(pointcloud_to_map_node ${PROJECT_NAME}_gencfg)
# target_link_libraries(pointcloud_to_grid_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

# Install Cpp executables
install(TARGETS
pointcloud_to_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
  
  