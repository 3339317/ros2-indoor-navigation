cmake_minimum_required(VERSION 3.8)
project(indoor_route_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native -flto)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

add_executable(indoor_route_planner_node src/indoor_route_planner_node.cpp)
target_compile_features(indoor_route_planner_node PUBLIC cxx_std_17)
ament_target_dependencies(indoor_route_planner_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(indoor_route_planner_node "${cpp_typesupport_target}")

add_executable(indoor_route_controller_node src/indoor_route_controller_node.cpp)
target_compile_features(indoor_route_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(indoor_route_controller_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  livox_ros_driver2
)

add_executable(indoor_map_preprocessor_node src/indoor_map_preprocessor_node.cpp)
target_compile_features(indoor_map_preprocessor_node PUBLIC cxx_std_17)
target_include_directories(indoor_map_preprocessor_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(indoor_map_preprocessor_node
  rclcpp
  ament_index_cpp
)
target_link_libraries(indoor_map_preprocessor_node
  ${PCL_LIBRARIES}
  yaml-cpp
)

pybind11_add_module(_planning_core SHARED src/indoor_planning_bindings.cpp)
target_compile_features(_planning_core PRIVATE cxx_std_17)
target_link_libraries(_planning_core PRIVATE Python3::Module)

install(TARGETS indoor_route_planner_node indoor_route_controller_node indoor_map_preprocessor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS _planning_core
  LIBRARY DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
)

install(PROGRAMS scripts/web_app.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME web_app
)

install(PROGRAMS scripts/publish_pcl_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME publish_pcl_pose
)

install(FILES scripts/core_ros.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY static launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
