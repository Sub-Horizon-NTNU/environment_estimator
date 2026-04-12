cmake_minimum_required(VERSION 3.8)
project(environment_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(object_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(environment_estimator src/main.cpp
  src/EnvironmentEstimatorNode.cpp
  src/KalmanFilter.cpp
  src/Object.cpp
  src/ObjectManager.cpp
  src/USVStates.cpp
  src/EnvironmentVisualizer.cpp
  src/ObjectUtilities.cpp
  src/StaticObject.cpp
  src/DynamicObject.cpp
)

target_include_directories(environment_estimator PRIVATE include)
ament_target_dependencies(environment_estimator 
  rclcpp 
  object_msgs 
  nav_msgs 
  tf2
  tf2_geometry_msgs
  Eigen3
  geometry_msgs
  visualization_msgs
)

install(TARGETS environment_estimator
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
