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
#message(WARNING "std_msgs_INCLUDE_DIRS: ${std_msgs_INCLUDE_DIRS}")

find_package(sensor_msgs REQUIRED)
#message(WARNING "sensor_msgs_INCLUDE_DIRS: ${sensor_msgs_INCLUDE_DIRS}")

find_package(tensorflow_lite REQUIRED)
list(APPEND libtensorflow_lite_INCLUDE_DIRS "/usr/include/tensorflow")
#message(WARNING "MANUALLY ADDED INCLUDE DIRECTORIES FOR libtensorflow_lite")
message(WARNING "libtensorflow_lite_INCLUDE_DIRS: ${tensorflow_lite_INCLUDE_DIRS}")

find_package(OpenCV REQUIRED)
#message(WARNING "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

find_package(cv_bridge REQUIRED)
list(APPEND cv_bridge_INCLUDE_DIRS "/opt/ros/humble/include/cv_bridge")
message(WARNING "MANUALLY ADDED INCLUDE DIRECTORIES FOR cv_bridge")
message(WARNING "cv_bridge_INCLUDE_DIRS: ${cv_bridge_INCLUDE_DIRS}")


find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Hpe.msg"
  #DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)


# Include directories
include_directories(
  include
  ${tensorflow_lite_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

include_directories(${cv_bridge_INCLUDE_DIRS})
add_executable(worker src/worker_node.cpp)
# Link libraries
message(WARNING "MANUALLY ADDED LIBRARIES FOR tensorflow_lite")
list(APPEND tensorflow_lite_LIBRARIES "/usr/lib/libtensorflowlite.so")
message(WARNING "tensorflow_lite_LIBRARIES: ${tensorflow_lite_LIBRARIES}")
message(WARNING "MANUALLY ADDED LIBRARIES FOR cv_bridge")
list(APPEND tensorflow_lite_LIBRARIES "/opt/ros/humble/lib/libcv_bridge.so")
message(WARNING "cv_bridge_LIBRARIES: ${cv_bridge_LIBRARIES}")

target_link_libraries(worker ${tensorflow_lite_LIBRARIES})
target_link_libraries(worker ${OpenCV_LIBS})
target_link_libraries(worker ${cv_bridge_LIBRARIES})
target_link_libraries(worker ${sensor_msgs_LIBRARIES})


ament_target_dependencies(worker rclcpp std_msgs sensor_msgs)
install(TARGETS
  worker
  DESTINATION lib/${PROJECT_NAME})

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
