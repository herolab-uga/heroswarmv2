cmake_minimum_required(VERSION 3.14)
project(roboto_controller)

set(CMAKE_CXX_STANDARD 20)

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

add_executable(
  apds9960_publisher 
  src/sensors/APDS9960/APDS9960.cpp 
)

target_include_directories(apds9960_publisher PRIVATE src/sensors)
target_link_libraries(apds9960_publisher i2c)
target_link_libraries(apds9960_publisher ${tf2_LIBRARIES})
target_compile_options(apds9960_publisher PUBLIC "-pthread")
ament_target_dependencies(apds9960_publisher rclcpp tf2 tf2_ros sensor_msgs std_msgs robot_msgs)

install(TARGETS apds9960_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(robot_controller
  src/crc/crc.cpp 
  src/router/router.cpp
  src/stream_header/stream_header.cpp
  src/uart/uart.cpp
  src/controller/controller.cpp
)

target_include_directories(robot_controller PRIVATE 
  src/controller 
  src/uart 
  src/crc
  src/utils
  src/router
  src/stream_header
)

target_link_libraries(robot_controller ${tf2_LIBRARIES})

# target_compile_options(apds9960_publisher PUBLIC "-pthread")
ament_target_dependencies(robot_controller rclcpp tf2 tf2_ros sensor_msgs std_msgs nav_msgs robot_msgs geometry_msgs)

install(TARGETS robot_controller DESTINATION lib/${PROJECT_NAME})

ament_package()
