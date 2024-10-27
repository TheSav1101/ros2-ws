cmake_minimum_required(VERSION 3.8)
project(hpe_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
#message(WARNING "rclcpp_INCLUDE_DIRS: ${rclcpp_INCLUDE_DIRS}")

find_package(std_msgs REQUIRED)

find_package(visualization_msgs REQUIRED)
#message(WARNING "std_msgs_INCLUDE_DIRS: ${std_msgs_INCLUDE_DIRS}")

find_package(sensor_msgs REQUIRED)
#message(WARNING "sensor_msgs_INCLUDE_DIRS: ${sensor_msgs_INCLUDE_DIRS}")

find_package(tensorflowlite REQUIRED)
list(APPEND tensorflowlite_INCLUDE_DIRS "/usr/include/tensorflow")
#message(WARNING "MANUALLY ADDED INCLUDE DIRECTORIES FOR tensorflowlite")
message(WARNING "tensorflowlite_INCLUDE_DIRS: ${tensorflowlite_INCLUDE_DIRS}")

find_package(OpenCV REQUIRED)
#message(WARNING "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

find_package(cv_bridge REQUIRED)

find_package(hpe_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(nlohmann_json REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



# Include directories
include_directories(
  include
  ${tensorflowlite_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(worker src/worker_node_main.cpp src/worker_node.cpp src/data.cpp)
add_executable(visualizer src/visualizer_node_main.cpp src/visualizer_node.cpp)
add_executable(master src/master_node_main.cpp src/master_node.cpp src/calibration.cpp)
add_executable(master_single src/master_node_single_main.cpp src/master_node_single.cpp src/calibration.cpp)
add_executable(webcam src/webcam_node_main.cpp src/webcam_node.cpp)
add_executable(slave src/slave_node_main.cpp src/slave_node.cpp src/worker_node.cpp src/webcam_node.cpp src/data.cpp)
add_executable(slave_single src/slave_node_single_main.cpp src/slave_node_single.cpp src/webcam_node.cpp src/data.cpp)

# Link libraries
message(WARNING "MANUALLY ADDED LIBRARIES FOR tensorflowlite")
list(APPEND tensorflowlite_LIBRARIES "/usr/lib/libtensorflowlite.so")
message(WARNING "tensorflowlite_LIBRARIES: ${tensorflowlite_LIBRARIES}")


target_link_libraries(worker ${tensorflowlite_LIBRARIES})
target_link_libraries(worker ${OpenCV_LIBS})
target_link_libraries(worker ${cv_bridge_LIBRARIES})
target_link_libraries(worker ${sensor_msgs_LIBRARIES})
target_link_libraries(worker ${std_msgs_LIBRARIES})

target_link_libraries(webcam ${OpenCV_LIBS})
target_link_libraries(webcam ${cv_bridge_LIBRARIES})
target_link_libraries(webcam ${sensor_msgs_LIBRARIES})
target_link_libraries(webcam ${std_msgs_LIBRARIES})

target_link_libraries(master ${OpenCV_LIBS})
target_link_libraries(master ${cv_bridge_LIBRARIES})
target_link_libraries(master ${sensor_msgs_LIBRARIES})
target_link_libraries(master ${std_msgs_LIBRARIES})
target_link_libraries(master ${visualization_msgs_LIBRARIES})
target_link_libraries(master ${tf2_LIBRARIES})
target_link_libraries(master ${tf2_geometry_msgs_LIBRARIES})

target_link_libraries(master_single ${OpenCV_LIBS})
target_link_libraries(master_single ${cv_bridge_LIBRARIES})
target_link_libraries(master_single ${sensor_msgs_LIBRARIES})
target_link_libraries(master_single ${std_msgs_LIBRARIES})
target_link_libraries(master_single ${tf2_LIBRARIES})
target_link_libraries(master_single ${tf2_geometry_msgs_LIBRARIES})
target_link_libraries(master_single ${visualization_msgs_LIBRARIES})

target_link_libraries(visualizer ${OpenCV_LIBS})
target_link_libraries(visualizer ${cv_bridge_LIBRARIES})
target_link_libraries(visualizer ${sensor_msgs_LIBRARIES})
target_link_libraries(visualizer ${std_msgs_LIBRARIES})

target_link_libraries(slave ${tensorflowlite_LIBRARIES})
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

ament_target_dependencies(worker rclcpp std_msgs sensor_msgs hpe_msgs tensorflowlite cv_bridge)
ament_target_dependencies(visualizer rclcpp std_msgs sensor_msgs hpe_msgs cv_bridge)
ament_target_dependencies(master rclcpp std_msgs visualization_msgs sensor_msgs hpe_msgs cv_bridge tf2 tf2_geometry_msgs)
ament_target_dependencies(master_single rclcpp std_msgs visualization_msgs sensor_msgs hpe_msgs cv_bridge tf2 tf2_geometry_msgs)
ament_target_dependencies(webcam rclcpp std_msgs sensor_msgs hpe_msgs cv_bridge)
ament_target_dependencies(slave rclcpp std_msgs sensor_msgs hpe_msgs tensorflowlite cv_bridge tf2 tf2_geometry_msgs)
ament_target_dependencies(slave_single rclcpp std_msgs sensor_msgs hpe_msgs tensorflowlite cv_bridge tf2 tf2_geometry_msgs)

install(TARGETS
  visualizer
  master
  slave
  slave_single
  master_single
  webcam
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
