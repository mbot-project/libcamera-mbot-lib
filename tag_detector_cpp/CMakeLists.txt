cmake_minimum_required(VERSION 3.8)
project(tag_detector_cpp)

# ─── C++ standard ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── Ament / ROS2 ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)

find_package(rclcpp        REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge     REQUIRED)
find_package(apriltag_msgs REQUIRED)

# ─── 3rd-party libraries ───────────────────────────────────────────────────────
find_package(OpenCV  REQUIRED COMPONENTS core imgproc calib3d)
find_package(Eigen3  REQUIRED)
find_package(apriltag REQUIRED)                       # apriltag::apriltag

# ── Clean up broken include dirs exported by apriltag target ──────────────────
get_target_property(_apriltag_inc apriltag::apriltag INTERFACE_INCLUDE_DIRECTORIES)

# Remove any bogus path under /usr/lib/include*
foreach(dir IN LISTS _apriltag_inc)
  if(dir MATCHES "^/usr/lib/include" AND NOT EXISTS "${dir}")
    list(REMOVE_ITEM _apriltag_inc "${dir}")
  endif()
endforeach()

# Ensure the real path is present
list(APPEND _apriltag_inc "/usr/include/apriltag")
list(REMOVE_DUPLICATES _apriltag_inc)

set_target_properties(apriltag::apriltag
  PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_apriltag_inc}")

# ─── Executable ────────────────────────────────────────────────────────────────
add_executable(tag_detector_node src/tag_detector_node.cpp)

ament_target_dependencies(tag_detector_node
  rclcpp sensor_msgs geometry_msgs cv_bridge apriltag_msgs
  OpenCV Eigen3)

target_link_libraries(tag_detector_node
  apriltag::apriltag
  Eigen3::Eigen)

# ─── Install ───────────────────────────────────────────────────────────────────
install(TARGETS tag_detector_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
