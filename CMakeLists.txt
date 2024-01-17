cmake_minimum_required(VERSION 3.5)
project(robofleet_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robofleet_client_msgs REQUIRED)
find_package(rosidl_adapter REQUIRED)

find_package(Flatbuffers REQUIRED)
find_package(yaml-cpp REQUIRED)

# Qt setup
set(CMAKE_AUTOMOC on)
set(CMAKE_AUTORCC on)
set(CMAKE_AUTOUIC on)
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

set(Qt5_LIBRARIES Qt5Core Qt5Network Qt5WebSockets)

set(PLUGINS_LIBRARY ${PROJECT_NAME}_plugins)

add_library(${PLUGINS_LIBRARY})
ament_target_dependencies(${PLUGINS_LIBRARY}
  rclcpp
  pluginlib
  Qt5
  Flatbuffers
)
target_include_directories(${PLUGINS_LIBRARY}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(${PLUGINS_LIBRARY}
  PRIVATE -fPIC
)

target_sources(${PLUGINS_LIBRARY}
  PRIVATE
    src/ROSMsgHandlers.cpp
    src/ROSSrvHandlers.cpp
    src/common_conversions.cpp
  #PUBLIC
    src/MessageScheduler.cpp
    src/MessageSchedulerLib.hpp
)

add_executable(client)
ament_target_dependencies(client
  rclcpp
  robofleet_client_msgs
  pluginlib
  Qt5
  Flatbuffers
)
target_include_directories(client
  PUBLIC
    include
)
target_link_libraries(client
  yaml-cpp
  ${PLUGINS_LIBRARY}
)
target_compile_options(client
  PRIVATE -fPIC
)

target_sources(client
  PRIVATE
    src/main.cpp
    src/WsClient.hpp
    src/WsServer.hpp
    src/RosClientNode.cpp
    src/MessageScheduler.hpp
)

## Package tests
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

install(
  TARGETS client
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PLUGINS_LIBRARY} HAS_LIBRARY_TARGET)
ament_export_dependencies(pluginlib Qt5Core)

install(
  TARGETS ${PLUGINS_LIBRARY}
  EXPORT export_${PLUGINS_LIBRARY}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  PROGRAMS scripts/generate_plugin_pkg.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY ${PROJECT_NAME}/generate/templates
            ${PROJECT_NAME}/generate/msg2fbs
  DESTINATION share/${PROJECT_NAME}/generate
)

ament_package()
