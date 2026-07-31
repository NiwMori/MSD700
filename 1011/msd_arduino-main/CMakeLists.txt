cmake_minimum_required(VERSION 3.8)
project(msd_arduino)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(msd_msgs REQUIRED)

include_directories(include)

add_executable(msd_arduino_node
  src/msd_arduino_node.cpp
  src/serial_port.cpp
  src/msd_link.cpp
)
ament_target_dependencies(msd_arduino_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  msd_msgs
)

add_executable(msd_arduino_dummy_node
  src/msd_arduino_dummy_node.cpp
)
ament_target_dependencies(msd_arduino_dummy_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  msd_msgs
)

install(TARGETS
  msd_arduino_node
  msd_arduino_dummy_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(PROGRAMS
  scripts/record_controller_trial.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  msd-arduino.service
  msd-arduino.env
  DESTINATION share/${PROJECT_NAME}/systemd/system
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
