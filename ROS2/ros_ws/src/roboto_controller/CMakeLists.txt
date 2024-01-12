cmake_minimum_required(VERSION 3.8)
project(roboto_controller)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(sensor_publisher src/sensor_pub/includes/sensor_pub.hpp src/sensor_pub/APDS9960.cpp src/sensor_pub/BMP280.cpp src/sensor_pub/LIS3MDL.cpp src/sensor_pub/LSM6DS33.cpp src/sensor_pub/SHT31D.cpp  src/sensor_pub/sensor_pub.cpp src/uart/uart.hpp  src/uart/uart.cpp)
target_include_directories(sensor_publisher PRIVATE src/sensor_pub src/uart)
target_link_libraries(sensor_publisher i2c)
target_link_libraries(sensor_publisher tf2)
target_compile_options(sensor_publisher PUBLIC "-pthread")
ament_target_dependencies(sensor_publisher rclcpp tf2 tf2_ros sensor_msgs std_msgs nav_msgs robot_msgs)

install(TARGETS sensor_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(robot_controller src/controller/includes/controller.hpp src/controller/controller.cpp src/uart/uart.cpp)
target_include_directories(robot_controller PRIVATE src/controller src/uart)
target_link_libraries(robot_controller tf2)
# target_compile_options(sensor_publisher PUBLIC "-pthread")
ament_target_dependencies(robot_controller rclcpp tf2 tf2_ros sensor_msgs std_msgs nav_msgs robot_msgs geometry_msgs)

install(TARGETS robot_controller DESTINATION lib/${PROJECT_NAME})

ament_package()
