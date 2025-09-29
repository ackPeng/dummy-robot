cmake_minimum_required(VERSION 3.8)
project(algorithms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 基本依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Eigen3（系统库）
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(PROJ_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 库
add_library(dof6_kinematic
  src/6dof_kinematic.cpp
)
target_include_directories(dof6_kinematic
  PUBLIC
    $<BUILD_INTERFACE:${PROJ_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(dof6_kinematic
  PUBLIC Eigen3::Eigen
)
target_compile_features(dof6_kinematic PUBLIC cxx_std_17)

# 可执行
add_executable(kinematic_test
  src/kinematic_test.cpp
)
# 这里改为“非关键字签名”，去掉 PRIVATE
target_link_libraries(kinematic_test
  dof6_kinematic
)
# 仍然使用 ament_target_dependencies（它内部是非关键字签名）
ament_target_dependencies(kinematic_test rclcpp std_msgs)

# 安装
install(TARGETS
  dof6_kinematic
  kinematic_test
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
