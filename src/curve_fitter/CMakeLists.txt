cmake_minimum_required(VERSION 3.8)
project(curve_fitter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Ceres REQUIRED)
find_package(sim_interfaces REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(fmt REQUIRED)

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

add_executable(${PROJECT_NAME}_node
        src/test_node.cpp
        src/simulator.cpp
        src/curve_fitter.cpp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE include)

target_link_libraries(${PROJECT_NAME}_node
        ${CERES_LIBRARIES}
        fmt::fmt
)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp geometry_msgs sim_interfaces rm_interfaces)

install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
