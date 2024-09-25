cmake_minimum_required(VERSION 3.5)
project(raisin_gimbal_com_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(raisin_parameter REQUIRED)
find_package(raisin_interfaces REQUIRED)
find_package(raisin_plugin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(realsense2 REQUIRED)
find_package(raisim REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

file(GLOB SRC_FILES CONFIGURE_DEPENDS "src/gimbal_plugin/*.cpp")
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  ament_index_cpp
  raisin_parameter
  raisin_plugin
  cv_bridge
  realsense2
  std_srvs
)

set(CMAKE_PREFIX_PATH "/path/to/raisim" ${CMAKE_PREFIX_PATH})
find_package(raisim REQUIRED)

target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(raisin_parameter)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(FILES config/params.yaml
        DESTINATION share/${PROJECT_NAME})

install(
        DIRECTORY include/
        DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
