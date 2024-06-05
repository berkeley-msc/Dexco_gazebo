cmake_minimum_required(VERSION 3.0.2)
project(joint_state_to_command)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_joint_state_to_command src/test_joint_state_to_command.cpp)

target_link_libraries(test_joint_state_to_command
  ${catkin_LIBRARIES}
)

add_executable(dexco_joint_state_to_command src/dexco_joint_state_to_command.cpp)

target_link_libraries(dexco_joint_state_to_command
  ${catkin_LIBRARIES}
)