cmake_minimum_required(VERSION 3.8)
project(row_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)
find_package(cv_bridge REQUIRED)

include_directories(/usr/local/include/opencv4/)
include_directories(/home/cody/ros_ws/vision_opencv/cv_bridge)

add_executable(cv_row_detection src/cv_row_detection.cpp src/RowDetection.cpp src/ImageProc.cpp src/RowDetection.hpp src/ImageProc.hpp)
add_executable(test_publisher src/test_publisher.cpp)
add_executable(dl_row_detection src/dl_row_detection.cpp src/ImageProc.cpp src/ImageProc.hpp)
ament_target_dependencies(cv_row_detection rclcpp sensor_msgs std_msgs cv_bridge OpenCV)
ament_target_dependencies(test_publisher rclcpp sensor_msgs cv_bridge OpenCV)
ament_target_dependencies(dl_row_detection rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(test_publisher /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_imgcodecs.so)
target_link_libraries(cv_row_detection /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_imgcodecs.so)
target_link_libraries(dl_row_detection /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_imgcodecs.so)

install(TARGETS
	test_publisher
	cv_row_detection
	dl_row_detection
	DESTINATION lib/${PROJECT_NAME})

ament_package()
