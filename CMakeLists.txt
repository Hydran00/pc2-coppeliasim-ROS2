cmake_minimum_required(VERSION 3.5)
project(pc2_coppeliasim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# list(APPEND CMAKE_PREFIX_PATH "$ENV{COPPELIASIM_ROOT_DIR}/programming/zmqRemoteApi/clients/cpp/build/cppzmq/")


# message("CMAKE_PREFIX_PATH:" ${CMAKE_PREFIX_PATH})
set(cppzmq_DIR "$ENV{COPPELIASIM_ROOT_DIR}/programming/zmqRemoteApi/clients/cpp/build/cppzmq/")
list(APPEND CMAKE_PREFIX_PATH cppzmq_DIR)


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

include_directories(
  ${PCL_INCLUDE_DIRS}
  $ENV{COPPELIASIM_ROOT_DIR}/programming/zmqRemoteApi/clients/cpp/
  $ENV{COPPELIASIM_ROOT_DIR}/programming/zmqRemoteApi/clients/cpp/build/jsoncons/include/
)


add_library(RemoteAPIClient STATIC IMPORTED) # or STATIC instead of SHARED
set_target_properties(RemoteAPIClient PROPERTIES
  IMPORTED_LOCATION "$ENV{COPPELIASIM_ROOT_DIR}/programming/zmqRemoteApi/clients/cpp/build/libRemoteAPIClient.a"
  INTERFACE_INCLUDE_DIRECTORIES "$ENV{COPPELIASIM_ROOT_DIR}/programming/zmqRemoteApi/clients/cpp/build/cppzmq/" 
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



