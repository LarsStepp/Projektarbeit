cmake_minimum_required(VERSION 3.8)
project(lars_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME}
              SHARED
              src/neutral_flexion_axis_controller.cpp)
  target_include_directories(
              ${PROJECT_NAME}
              PRIVATE
              include
              ${EIGEN3_INCLUDE_DIR})
            
  ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        franka_msgs
        franka_semantic_components
        hardware_interface
        moveit_core
        moveit_msgs
        pinocchio
        pluginlib
        rclcpp
        rclcpp_lifecycle
        std_srvs
        realtime_tools
        hardware_interface
        geometry_msgs
  )

  pluginlib_export_plugin_description_file(
        controller_interface neutral_flexion_axis_controller_plugin.xml)

  install(
        TARGETS
        ${PROJECT_NAME}
        DESTINATION lib
  )

  install(
          DIRECTORY include/
          DESTINATION include
  )
  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME})


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
