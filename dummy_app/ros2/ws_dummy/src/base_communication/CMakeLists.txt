cmake_minimum_required(VERSION 3.8)
project(base_communication)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++20（std::jthread）
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

# 库目标（SerialLink + DummyRobot）
add_library(${PROJECT_NAME} SHARED
  src/serial_link.cpp
  src/dummy_robot.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${Boost_LIBRARIES}
    Threads::Threads
)

# 节点可执行程序：dummy_robot_control
add_executable(dummy_robot_control
  src/dummy_robot_control.cpp
)

ament_target_dependencies(dummy_robot_control
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
)

# 统一使用 plain 签名，避免与上面的 ament_target_dependencies 冲突
target_link_libraries(dummy_robot_control
  ${PROJECT_NAME}       # 链接本包库
)

# 安装与导出
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS dummy_robot_control
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost)
ament_export_include_directories(include)

# 测试（保持原有配置）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权与 cpplint 检查
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
