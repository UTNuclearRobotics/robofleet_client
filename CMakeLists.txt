cmake_minimum_required(VERSION 3.1.0)
project(robofleet_client)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ROS dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  amrl_msgs
)

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

catkin_package(
  CATKIN_DEPENDS roscpp amrl_msgs
  DEPENDS Qt5
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Qt5_INCLUDE_DIRS})
include_directories(robofleet_client_lib)
include_directories(robofleet_client_lib/include)

add_executable(client src/main.cpp)
target_link_libraries(client ${catkin_LIBRARIES} ${Qt5_LIBRARIES})

add_subdirectory(src)

## Package tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(TEST_NAME "${PROJECT_NAME}_test")

  add_rostest_gtest(${TEST_NAME} test/${PROJECT_NAME}.test test/main.cpp)
                    
  add_dependencies(${TEST_NAME} client)
    
  target_link_libraries(${TEST_NAME} ${catkin_LIBRARIES})
endif()
