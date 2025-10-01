cmake_minimum_required(VERSION 3.8)
project(polka)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

option(WITH_CUDA "Enable CUDA acceleration" ON)
if(WITH_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    set(CMAKE_CUDA_ARCHITECTURES "86;87")
    find_package(CUDAToolkit REQUIRED)
    add_definitions(-DPOLKA_CUDA_ENABLED)
    message(STATUS "polka: CUDA enabled (sm_86, sm_87)")
  else()
    message(WARNING "polka: CUDA not found, CPU-only build")
    set(WITH_CUDA OFF)
  endif()
endif()

include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

set(POLKA_CORE_SOURCES
  src/config_loader.cpp
  src/filters/range_filter.cpp
  src/filters/angular_filter.cpp
  src/filters/box_filter.cpp
  src/source_adapter.cpp
  src/merge_engine/cpu_merge_engine.cpp
  src/polka_node.cpp
)

add_library(polka_core SHARED ${POLKA_CORE_SOURCES})
target_include_directories(polka_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(polka_core
  rclcpp rclcpp_components sensor_msgs geometry_msgs nav_msgs
  tf2_ros tf2_geometry_msgs tf2_eigen pcl_conversions laser_geometry
)
target_link_libraries(polka_core ${PCL_LIBRARIES})

if(WITH_CUDA)
  add_library(polka_cuda src/merge_engine/cuda_merge_engine.cu)
  target_include_directories(polka_cuda PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
  target_compile_options(polka_cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr -diag-suppress=20012,20013,20015>)
  target_link_libraries(polka_cuda CUDA::cudart)
  set_target_properties(polka_cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION OFF
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    POSITION_INDEPENDENT_CODE ON
  )
  target_link_libraries(polka_core polka_cuda)
endif()

rclcpp_components_register_nodes(polka_core "polka::PolkaNode")

add_executable(polka_node src/main.cpp)
target_link_libraries(polka_node polka_core)

install(TARGETS polka_core ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
if(WITH_CUDA)
  install(TARGETS polka_cuda ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
endif()
install(TARGETS polka_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
