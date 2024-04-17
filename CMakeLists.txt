cmake_minimum_required(VERSION 3.5)
project(pc2_coppeliasim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (CMAKE_PREFIX_PATH "/home/nardi/CoppeliaSim/programming/zmqRemoteApi/clients/cpp/build/cppzmq")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cppzmq REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Create an executable
add_executable(raw_to_pc2 src/raw_to_pc2.cpp)

# ZMQ CoppeliaSim dependencies
# target_compile_definitions(raw_to_pc2 PUBLIC -DSIM_REMOTEAPICLIENT_OBJECTS)
# Include directories for dependencies
include_directories(
  ${PCL_INCLUDE_DIRS}
  /home/nardi/CoppeliaSim/programming/zmqRemoteApi/clients/cpp/
  /home/nardi/CoppeliaSim/programming/zmqRemoteApi/clients/cpp/build/jsoncons/include/
  /home/nardi/CoppeliaSim/programming/zmqRemoteApi/clients/cpp/build/generated/
)
# link_directories(raw_to_pc2
#   PUBLIC
#   /home/nardi/CoppeliaSim/programming/zmqRemoteApi/clients/cpp/build/
# )

add_library(RemoteAPIClient STATIC IMPORTED) # or STATIC instead of SHARED
set_target_properties(RemoteAPIClient PROPERTIES
  IMPORTED_LOCATION "/home/nardi/CoppeliaSim/programming/zmqRemoteApi/clients/cpp/build/libRemoteAPIClient.a"
  INTERFACE_INCLUDE_DIRECTORIES "/home/nardi/CoppeliaSim/programming/zmqRemoteApi/clients/cpp/" 
)

# set cppzmq_DIR
target_compile_definitions(raw_to_pc2 PUBLIC -DSIM_REMOTEAPICLIENT_OBJECTS)

# Link the executable with necessary libraries
ament_target_dependencies(raw_to_pc2
rclcpp
std_msgs
pcl_conversions
sensor_msgs
)
target_link_libraries(raw_to_pc2 RemoteAPIClient cppzmq ${PCL_LIBRARIES})

# Install the executable
install(TARGETS
  raw_to_pc2
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  rviz_config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()



