# Locate necessary packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Add the test executable
add_executable(
  cpp_test
  # Source files for the test target:
  test_level1.cpp)

# Include directories needed for the test
target_include_directories(
  cpp_test PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS}
                  ${rclcpp_INCLUDE_DIRS})


# Link dependencies for the test
ament_target_dependencies(
  cpp_test
  tf2
  tf2_geometry_msgs
  nav_msgs
  OpenCV
  rclcpp
  std_msgs
  gazebo_msgs)


# Link GTest explicitly
target_link_libraries(cpp_test gtest gtest_main myDebris)


# Enable code coverage flags for the test target
target_compile_options(cpp_test PUBLIC --coverage)
target_link_options(cpp_test PUBLIC --coverage)


# Discover tests using GoogleTest
gtest_discover_tests(cpp_test)


# Install the test binary (optional)
install(TARGETS cpp_test DESTINATION lib/${PROJECT_NAME})
