cmake_minimum_required(VERSION 3.8)
# 包名，与功能包名一致
project(volleyball_communication)

# C++17标准（ROS2 Humble推荐，兼容C++11/14）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找ROS2核心依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# 编译下位机C++订阅节点
add_executable(ball_point_subscriber src/ball_point_subscriber.cpp)
ament_target_dependencies(ball_point_subscriber rclcpp std_msgs)
# 安装C++节点到系统，支持ros2 run调用
install(TARGETS
  ball_point_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# 安装上位机Python节点
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/ball_point_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 测试配置（注释不影响，保留即可）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ROS2包编译收尾（固定）
ament_package()
