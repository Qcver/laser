cmake_minimum_required(VERSION 3.8)
project(lio_naver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(position_printer src/position_printer.cpp)
ament_target_dependencies(position_printer
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

add_executable(chassis_odom src/chassis_odom.cpp)
ament_target_dependencies(chassis_odom
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

add_executable(chassis_printer src/chassis_printer.cpp)
ament_target_dependencies(chassis_printer
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

# 全局系版本：输出“底盘中心在 map 中的位姿”（含雷达偏移、全局偏置、twist 杠杆臂补偿）
add_executable(global_chassis_odom src/global_chassis_odom.cpp)
ament_target_dependencies(global_chassis_odom
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

# 全局系专用打印/记录：/global/chassis_odom -> 终端 + CSV（同频，默认 10 Hz）
add_executable(global_chassis_printer src/global_chassis_printer.cpp)
ament_target_dependencies(global_chassis_printer
  rclcpp nav_msgs tf2)

install(TARGETS
  position_printer
  chassis_odom
  chassis_printer
  global_chassis_odom
  global_chassis_printer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



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

ament_package()
