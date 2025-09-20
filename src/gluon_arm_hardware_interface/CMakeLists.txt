cmake_minimum_required(VERSION 3.8)
project(gluon_arm_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

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

# Create a library for the actuator SDK
add_library(actuator_ne30_sdk
  include/gluon_arm_hardware_interface/actuator_ne30_sdk.cpp
)

# Specify include directories for the library
target_include_directories(actuator_ne30_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add dependencies to the library
target_link_libraries(actuator_ne30_sdk ${CMAKE_THREAD_LIBS_INIT})
ament_target_dependencies(actuator_ne30_sdk rclcpp)

# Create the test executable
add_executable(test_actuator src/test_actuator.cpp)

# Link the SDK library with the test executable
target_link_libraries(test_actuator actuator_ne30_sdk)

# Specify include directories for the executable
target_include_directories(test_actuator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(test_actuator rclcpp)

# Install targets
install(TARGETS
  actuator_ne30_sdk
  test_actuator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

ament_package()