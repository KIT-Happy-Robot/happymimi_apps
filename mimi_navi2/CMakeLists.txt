cmake_minimum_required(VERSION 3.5)
project(mimi_navi2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(happymimi_msgs2 REQUIRED)
find_package(tf REQUIRED)

set(action_files "action/Navi2LocationAc.action")
set(srv_files "srv/NaviLocation.srv"
              "srv/SetLocation.srv")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${srv_files}
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_executable(cpp_executable src/navi2_location_server.cpp)
ament_target_dependencies(cpp_executable
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
  happymimi_msgs2
)

install(TARGETS cpp_executable
        DESTINATION lib/${PROJECT_NAME}
)
#target_link_libraries(${PROJECT_NAME} yaml-cpp)


install(DIRECTORY
        src include scripts launch config navi2_location_acserver
        DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/navi_location2.py
  scripts/set_location2.py
  scripts/navi_coord2.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
