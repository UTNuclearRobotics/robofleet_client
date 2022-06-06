cmake_minimum_required(VERSION 3.16.3)
project(robofleet_client)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ROS dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pluginlib
  message_generation
)

find_package(yaml-cpp REQUIRED)

# Qt setup
set(CMAKE_AUTOMOC on)
set(CMAKE_AUTORCC on)
set(CMAKE_AUTOUIC on)
set(CMAKE_INCLUDE_CURRENT_DIR on)
set(MIN_QT_VERSION "5.5.0")
find_package(Qt5 ${MIN_QT_VERSION} REQUIRED COMPONENTS
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

catkin_python_setup()

add_message_files(DIRECTORY msg
  FILES
  RobofleetSubscription.msg
)
generate_messages(DEPENDENCIES)

catkin_package(
  INCLUDE_DIRS include ${flatbuffers_INCLUDE_DIRS}
  LIBRARIES ${PLUGINS_LIBRARY}
  CATKIN_DEPENDS
    roscpp
    rosmsg
    rospack
    pluginlib
    message_runtime
  DEPENDS Qt5 YAML_CPP
)

add_library(${PLUGINS_LIBRARY}
  src/ROSMsgHandlers.cpp
  src/ROSSrvHandlers.cpp
  src/MessageScheduler.cpp
  src/common_conversions.cpp
  src/MessageSchedulerLib.hpp
)
target_link_libraries(${PLUGINS_LIBRARY}
  ${Qt5_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_include_directories(${PLUGINS_LIBRARY}
PUBLIC
  include
  src
  ${Qt5_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${flatbuffers_INCLUDE_DIRS}
INTERFACE
  include
)

add_executable(client
  src/main.cpp
  src/WsClient.hpp
  src/WsServer.hpp
  src/RosClientNode.cpp
  src/MessageScheduler.hpp
)
add_dependencies(client
  ${PLUGINS_LIBRARY}
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(client
  ${PLUGINS_LIBRARY}
  ${YAML_CPP_LIBRARIES}
)

target_include_directories(client
PRIVATE
  ${YAML_CPP_INCLUDE_DIR}
)

## Package tests
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
