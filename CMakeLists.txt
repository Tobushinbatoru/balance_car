cmake_minimum_required(VERSION 3.0.2)
project(balance_car)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES balance_car
#  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(balance_car_joy src/balance_car_joy.cpp)
add_executable(balance_car_pid src/balance_car_pid.cpp src/PID.cpp)
add_executable(ethernet src/ethernet.cpp)

target_link_libraries(balance_car_joy ${catkin_LIBRARIES})
target_link_libraries(balance_car_pid ${catkin_LIBRARIES})
target_link_libraries(ethernet ${catkin_LIBRARIES})

target_include_directories(balance_car_pid PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

