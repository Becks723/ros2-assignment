cmake_minimum_required(VERSION 3.8)
project(node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(serialnode src/serialnode.cpp src/parser.cpp)
add_executable(pubnode src/publishernode.cpp)

target_include_directories(serialnode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(serialnode rclcpp serial_driver std_msgs)
ament_target_dependencies(pubnode rclcpp std_msgs)

# 部署可执行文件
install(TARGETS
  serialnode
  pubnode
  DESTINATION lib/${PROJECT_NAME})
# 部署configs和launch目录
install(DIRECTORY
  configs
  launch
  DESTINATION share/${PROJECT_NAME})

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
