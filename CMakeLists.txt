cmake_minimum_required(VERSION 3.1.0)
project(robofleet_client)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ROS dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pluginlib
)

find_package(yaml-cpp REQUIRED)

# Qt setup
set(CMAKE_AUTOMOC on)
set(CMAKE_AUTORCC on)
set(CMAKE_AUTOUIC on)
set(CMAKE_INCLUDE_CURRENT_DIR on)
find_package(Qt5 REQUIRED COMPONENTS
  Core
  WebSockets
)

set(Qt5_INCLUDE_DIRS
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(Qt5_LIBRARIES Qt5::Core Qt5::WebSockets)

set(PLUGINS_LIBRARY ${PROJECT_NAME}_plugins)

set(flatbuffers_INCLUDE_DIRS scripts/generate/msg2fbs/flatbuffers/include)

catkin_package(
  INCLUDE_DIRS include ${flatbuffers_INCLUDE_DIRS}
  LIBRARIES ${PLUGINS_LIBRARY}
  CATKIN_DEPENDS
    roscpp
    rosmsg
    rospack
    pluginlib
  DEPENDS Qt5 YAML_CPP
)

catkin_python_setup()

add_library(${PLUGINS_LIBRARY})
target_link_libraries(${PLUGINS_LIBRARY}
  ${Qt5_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_include_directories(${PLUGINS_LIBRARY}
PUBLIC
  include
  ${Qt5_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${flatbuffers_INCLUDE_DIRS}
INTERFACE
  include
)

add_executable(client)
add_dependencies(client ${PLUGINS_LIBRARY})
target_link_libraries(client
  ${PLUGINS_LIBRARY}
  ${YAML_CPP_LIBRARIES}
)

target_include_directories(client
PRIVATE
  ${YAML_CPP_INCLUDE_DIR}
)
add_subdirectory(src)

catkin_python_setup()

## Package tests
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()