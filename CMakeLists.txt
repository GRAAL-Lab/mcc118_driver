cmake_minimum_required(VERSION 3.8)
project(mcc118_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom ROS 2 messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CableTension.msg"
  "msg/Time.msg"
)

# Create the publisher executable
add_executable(mcc118_publisher
  src/mcc118_publisher.cpp
)

# Include MCC DAQ HAT headers and project headers
target_include_directories(mcc118_publisher PRIVATE
  /usr/local/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link against the MCC DAQ HAT library
target_link_directories(mcc118_publisher PRIVATE
  /usr/local/lib
)

target_link_libraries(mcc118_publisher
  daqhats
)

ament_target_dependencies(mcc118_publisher
  rclcpp
  std_msgs
  geometry_msgs
)

# Link the generated ROS 2 C++ message support
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(mcc118_publisher
  "${cpp_typesupport_target}"
)

# Ensure messages are generated before compiling the publisher
add_dependencies(mcc118_publisher
  ${PROJECT_NAME}__rosidl_generator_cpp
)

install(TARGETS
  mcc118_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
