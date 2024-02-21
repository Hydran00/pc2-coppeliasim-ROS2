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
add_executable(pc2_coppeliasim src/pc2_coppeliasim.cpp)
# Link the executable with necessary libraries
ament_target_dependencies(pc2_coppeliasim
rclcpp
std_msgs
pcl_conversions
sensor_msgs
)
target_link_libraries(pc2_coppeliasim ${PCL_LIBRARIES})

# Install the executable
install(TARGETS
  pc2_coppeliasim
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
