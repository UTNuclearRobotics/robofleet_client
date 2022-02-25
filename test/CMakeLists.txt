find_package(rostest REQUIRED)
set(TEST_NAME "${PROJECT_NAME}_test")

add_rostest_gtest(${TEST_NAME} ${PROJECT_NAME}.test main.cpp)
                  
add_dependencies(${TEST_NAME} client)
  
target_link_libraries(${TEST_NAME} ${catkin_LIBRARIES})