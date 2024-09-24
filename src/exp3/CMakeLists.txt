cmake_minimum_required(VERSION 3.0.2)
project(exp3)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED)



catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(exp3 src/exp3.cpp)

target_link_libraries(exp3
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)
