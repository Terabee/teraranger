cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
project(teraranger)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(teraranger_evo src/teraranger_evo.cpp src/serial_port.cpp src/helper_lib.cpp)
target_link_libraries(teraranger_evo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(teraranger_evo ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(teraranger_evo ${PROJECT_NAME}_gencfg)

install( TARGETS
  teraranger_evo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)