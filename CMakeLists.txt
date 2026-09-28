cmake_minimum_required(VERSION 3.8)
project(EKF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Blaze (header-only math library). blaze::inv() needs BLAS + LAPACK.
find_package(blaze REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

add_executable(robot_odo src/robot_odo.cpp)

target_compile_definitions(robot_odo PRIVATE
  BLAZE_BLAS_MODE=1
  BLAZE_BLAS_INCLUDE_FILE=<cblas.h>
)

target_link_libraries(robot_odo
  blaze::blaze
  ${BLAS_LIBRARIES}
  ${LAPACK_LIBRARIES}
)

ament_target_dependencies(robot_odo
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS robot_odo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
