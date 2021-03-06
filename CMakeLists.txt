cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_sim)

add_definitions("-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  kuka_lwr_fri
  sensor_msgs
  std_msgs
  std_srvs
  kdl_parser
  kdl_conversions
  eigen_conversions
  lwr_fri
  rtt_ros_kdl_tools
)

find_package(gazebo REQUIRED)
if(NOT ${gazebo_VERSION_MAJOR})
    # Old gazebo version (<=indigo)
    set(gazebo_VERSION_MAJOR 5)
endif()
add_definitions(-DGAZEBO_MAJOR_VERSION=${gazebo_VERSION_MAJOR})

find_package(orocos_kdl REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)
orocos_library(lwr_common src/lwr_common.cpp)
target_link_libraries(lwr_common
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
)

target_link_libraries(lwr_common ${GAZEBO_LIBRARIES})

orocos_component(${PROJECT_NAME} src/rtt_lwr_sim.cpp)
target_link_libraries(${PROJECT_NAME} lwr_common)

# orocos_component(${PROJECT_NAME}_remote src/rtt_lwr_sim_remote.cpp)
# set_property(TARGET ${PROJECT_NAME}_remote APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)
# 
# target_link_libraries(${PROJECT_NAME}_remote
#    ${catkin_LIBRARIES}
#    ${USE_OROCOS_LIBRARIES}
#    ${kdl_parser_LIBRARIES}
#    ${orocos_kdl_LIBRARIES}
#    ${kdl_conversions_LIBRARIES}
#    ${GAZEBO_LIBRARIES}
#    lwr_common
# )

# add_dependencies(${PROJECT_NAME} lwr_fri_generate_messages_cpp)
orocos_generate_package(INCLUDE_DIRS include)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
