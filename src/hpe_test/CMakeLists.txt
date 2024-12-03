cmake_minimum_required(VERSION 3.8)
project(hpe_test)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)

find_package(visualization_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(tensorflowlite REQUIRED)

find_package(OpenCV REQUIRED)

find_package(cv_bridge REQUIRED)

find_package(hpe_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(nlohmann_json REQUIRED)

find_package(tf2 REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${tensorflowlite_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${TF_INCLUDE_DIRS}
)

add_executable(master_single src/master_node_single_main.cpp src/master_node_single.cpp src/calibration.cpp src/visualizer_node.cpp)
add_executable(slave src/slave_node_main.cpp src/slave_node.cpp src/worker_node.cpp src/webcam_node.cpp src/data.cpp src/responses.cpp)
add_executable(slave_single src/slave_node_single_main.cpp src/slave_node_single.cpp src/webcam_node.cpp src/data.cpp)
add_executable(broadcaster src/tf2_broadcaster_node_main.cpp src/tf2_broadcaster_node.cpp)
add_executable(accuracy src/accuracy_node_main.cpp src/accuracy_node.cpp)

target_link_libraries(master_single ${OpenCV_LIBS})
target_link_libraries(master_single ${cv_bridge_LIBRARIES})
target_link_libraries(master_single ${sensor_msgs_LIBRARIES})
target_link_libraries(master_single ${std_msgs_LIBRARIES})
target_link_libraries(master_single ${tf2_LIBRARIES})
target_link_libraries(master_single ${tf2_geometry_msgs_LIBRARIES})
target_link_libraries(master_single ${visualization_msgs_LIBRARIES})

target_link_libraries(slave ${tensorflowlite_LIBRARIES})
target_link_libraries(slave ${tensorflowlite_gpu_delegate_LIBRARIES})
target_link_libraries(slave ${OpenCV_LIBS})
target_link_libraries(slave ${cv_bridge_LIBRARIES})
target_link_libraries(slave ${sensor_msgs_LIBRARIES})
target_link_libraries(slave ${std_msgs_LIBRARIES})
target_link_libraries(slave ${nlohmann_json_LIBRARIES})

target_link_libraries(slave_single ${tensorflowlite_LIBRARIES})
target_link_libraries(slave_single ${OpenCV_LIBS})
target_link_libraries(slave_single ${cv_bridge_LIBRARIES})
target_link_libraries(slave_single ${sensor_msgs_LIBRARIES})
target_link_libraries(slave_single ${std_msgs_LIBRARIES})
target_link_libraries(slave_single ${nlohmann_json_LIBRARIES})

target_link_libraries(broadcaster ${YAML_CPP_LIBRARIES})
target_link_libraries(broadcaster ${tf2_LIBRARIES})
target_link_libraries(broadcaster ${tf2_geometry_msgs_LIBRARIES})
target_link_libraries(broadcaster ${geometry_msgs_LIBRARIES})

target_link_libraries(accuracy ${std_msgs_LIBRARIES})
target_link_libraries(accuracy ${visualization_msgs_LIBRARIES})
target_link_libraries(accuracy ${geometry_msgs_LIBRARIES})
target_link_libraries(accuracy ${tf2_LIBRARIES})
target_link_libraries(accuracy ${tf2_geometry_msgs_LIBRARIES})

ament_target_dependencies(master_single rclcpp std_msgs visualization_msgs sensor_msgs hpe_msgs cv_bridge tf2 tf2_geometry_msgs)
ament_target_dependencies(slave rclcpp std_msgs sensor_msgs hpe_msgs tensorflowlite cv_bridge tf2 tf2_geometry_msgs)
ament_target_dependencies(slave_single rclcpp std_msgs sensor_msgs hpe_msgs tensorflowlite cv_bridge tf2 tf2_geometry_msgs)
ament_target_dependencies(broadcaster rclcpp tf2_ros tf2_geometry_msgs tf2 geometry_msgs yaml-cpp)
ament_target_dependencies(accuracy rclcpp visualization_msgs std_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS
  slave
  slave_single
  master_single
  broadcaster
  accuracy
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)


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
