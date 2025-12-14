cmake_minimum_required(VERSION 3.8)
project(px4_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXEC_NAME offboard_control)

# Set path to config folder in repo root
set(GZ_PX4_BASE_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../config)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
  include
)  
  
add_executable(${EXEC_NAME} 
  src/ros_control.cpp)

ament_target_dependencies(${EXEC_NAME} 
  rclcpp
  nav_msgs
  geometry_msgs
  px4_msgs
)

target_link_libraries(${EXEC_NAME}
  Eigen3::Eigen
)

install(TARGETS
  ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include
  DESTINATION include
)

# Install launch directory 
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )  
  
# Install workspace config folder in package
install(DIRECTORY
  ${GZ_PX4_BASE_CONFIG_DIR}/ 
  DESTINATION share/${PROJECT_NAME}/config
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

