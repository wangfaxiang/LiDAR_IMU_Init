cmake_minimum_required(VERSION 3.5)
project(lidar_imu_init)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# C++17 is required for std::filesystem (replaces the ROS1 boost::filesystem usage).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-O3 -fexceptions -pthread)

add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(PythonLibs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

# Custom messages (same .msg files as the ROS1 version).
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  "msg/States.msg"
  DEPENDENCIES std_msgs
)

include_directories(
  include
  include/ikd-Tree
  include/LI_init
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_executable(li_init
  src/laserMapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
  include/LI_init/LI_init.cpp
  src/preprocess.cpp
)

target_link_libraries(li_init
  ${PCL_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${CERES_LIBRARIES}
)

rosidl_target_interfaces(li_init ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(li_init
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_include_directories(li_init PRIVATE ${PYTHON_INCLUDE_DIRS})

install(TARGETS li_init
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz_cfg
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()




