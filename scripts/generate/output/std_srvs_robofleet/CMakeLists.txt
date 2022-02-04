cmake_minimum_required(VERSION 3.0.2)
project(std_srvs_robofleet)

find_package(catkin COMPONENTS
  robofleet_client
  std_srvs
  REQUIRED
)

catkin_package(
  LIBRARIES std_srvs_robofleet
  CATKIN_DEPENDS robofleet_client std_srvs
)

set(PLUGIN_SRC
  src/EmptyRequest.cpp
  src/EmptyResponse.cpp
  src/SetBoolRequest.cpp
  src/SetBoolResponse.cpp
  src/TriggerRequest.cpp
  src/TriggerResponse.cpp
)

add_library(std_srvs_robofleet ${PLUGIN_SRC})
target_link_libraries(std_srvs_robofleet ${catkin_LIBRARIES})
target_include_directories(std_srvs_robofleet PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)
