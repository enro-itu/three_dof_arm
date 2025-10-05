cmake_minimum_required(VERSION 3.8)
project(arm3dof)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build system + runtime deps (even if you only ship Python)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install your Python executable
install(PROGRAMS
  src/move_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install data files so they’re available after install
install(DIRECTORY
  config
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# (Optional) tests / lint
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
