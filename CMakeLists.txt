cmake_minimum_required(VERSION 3.1.0)
project(robofleet_client)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ROS dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  amrl_msgs
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

catkin_package(
  INCLUDE_DIRS include/public robofleet_client_lib/include robofleet_client_lib
  LIBRARIES ${PLUGINS_LIBRARY}
  CATKIN_DEPENDS
    roscpp
    amrl_msgs
    pluginlib
  DEPENDS Qt5
)

include_directories(
  include/public
  include/private
  src
  ${catkin_INCLUDE_DIRS}
  ${Qt5_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  robofleet_client_lib
  robofleet_client_lib/include
)

set(QT_MOC_SRC
  include/private/robofleet_client/RosClientNode.hpp
  include/public/robofleet_client/MessageScheduler.hpp
)

add_executable(client)
target_sources(client PRIVATE
  src/main.cpp
  src/WsClient.hpp
  src/WsServer.hpp
  src/WebVizConstants.hpp
  src/RosClientNode.cpp
  src/MessageScheduler.cpp
  ${QT_MOC_SRC}
)
add_dependencies(client ${PLUGINS_LIBRARY})
target_link_libraries(client
  ${catkin_LIBRARIES}
  ${PLUGINS_LIBRARY}
  ${Qt5_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_library(${PLUGINS_LIBRARY} src/ROSMsgHandlers.cpp src/MessageScheduler.cpp)
target_link_libraries(${PLUGINS_LIBRARY} ${catkin_LIBRARIES})

## Package tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(TEST_NAME "${PROJECT_NAME}_test")

  add_rostest_gtest(${TEST_NAME} test/${PROJECT_NAME}.test test/main.cpp)
                    
  add_dependencies(${TEST_NAME} client)
    
  target_link_libraries(${TEST_NAME} ${catkin_LIBRARIES})
endif()
