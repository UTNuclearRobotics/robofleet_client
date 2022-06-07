find_package(rostest REQUIRED)

set(TEST_NAME client_test)
add_rostest_gtest(${TEST_NAME} client.test client_test.cpp)
                  
add_dependencies(${TEST_NAME} client)
  
target_link_libraries(${TEST_NAME} ${catkin_LIBRARIES})

target_include_directories(${TEST_NAME}
PRIVATE
  ${catkin_INCLUDE_DIRS}
)