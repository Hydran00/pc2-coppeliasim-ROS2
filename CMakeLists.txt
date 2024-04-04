cmake_minimum_required(VERSION 3.5)
project(pc2_coppeliasim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



# Create an executable
add_executable(raw_to_pc2 src/raw_to_pc2.cpp)
# Link the executable with necessary libraries
ament_target_dependencies(raw_to_pc2
rclcpp
std_msgs
pcl_conversions
sensor_msgs
)
target_link_libraries(raw_to_pc2 ${PCL_LIBRARIES})

# Install the executable
install(TARGETS
  raw_to_pc2
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
