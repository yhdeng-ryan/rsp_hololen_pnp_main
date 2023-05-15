cmake_minimum_required(VERSION 3.8)
project(main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ur5_interface REQUIRED)
find_package(moveit_ur5_msgs REQUIRED)
find_package(pnp_actionlib REQUIRED)
find_package(pnp_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(registration_service REQUIRED)
find_package(registration_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robotiq_2f_msgs REQUIRED)


include_directories(include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

add_library( main_subpub SHARED src/hololens_sub.cpp src/gripper_pub.cpp)
ament_target_dependencies( main_subpub rclcpp geometry_msgs robotiq_2f_msgs)

add_executable(main_node src/main_node.cpp)
target_link_libraries( main_node main_subpub jsoncpp)
ament_target_dependencies(main_node rclcpp moveit_ur5_interface moveit_ur5_msgs pnp_actionlib pnp_msgs registration_service registration_msgs tf2 tf2_ros tf2_geometry_msgs robotiq_2f_msgs)

install(DIRECTORY include DESTINATION include)
install(TARGETS main_subpub LIBRARY DESTINATION lib )
install(DIRECTORY json DESTINATION share/${PROJECT_NAME})
install(TARGETS main_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
