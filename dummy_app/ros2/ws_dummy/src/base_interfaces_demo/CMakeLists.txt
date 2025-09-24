cmake_minimum_required(VERSION 3.8)
project(base_interfaces_demo)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 可执行程序：键盘控制节点
add_executable(dummy_keyboard_teleop_node
  src/dummy_keyboard_teleop.cpp
)
ament_target_dependencies(dummy_keyboard_teleop_node
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
)

# 安装
install(TARGETS dummy_keyboard_teleop_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 如果有 launch/ 配置文件，可按需开启
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# 测试（保留你的原有配置）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权与 cpplint（与原模板一致）
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
