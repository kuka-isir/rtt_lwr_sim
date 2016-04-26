cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_sim)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  kuka_lwr_fri
  sensor_msgs
  std_msgs
  std_srvs
  cmake_modules
  kdl_parser
  kdl_conversions
  eigen_conversions
  lwr_fri
  rtt_ros_kdl_tools
)

find_package(gazebo REQUIRED)
if(GAZEBO_VERSION)
  if(${GAZEBO_VERSION} GREATER 6)
      add_definitions(-DGAZEBO_GREATER_6)
  endif()
endif()

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

orocos_component(${PROJECT_NAME} src/rtt_lwr_sim.cpp src/rtt_lwr_sim_hooks.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${USE_OROCOS_LIBRARIES}
   ${kdl_parser_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${kdl_conversions_LIBRARIES}
   ${GAZEBO_LIBRARIES}
)
# orocos_component(lwr_gazebo_service src/lwr_gazebo_service.cpp)
#
# target_link_libraries(lwr_gazebo_service
#     ${GAZEBO_LIBRARIES}
#     ${catkin_LIBRARIES}
#     ${USE_OROCOS_LIBRARIES}
# )

# add_dependencies(${PROJECT_NAME} lwr_fri_generate_messages_cpp)
orocos_generate_package(INCLUDE_DIRS include)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
