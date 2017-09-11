cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
project(teraranger)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

find_package(Boost REQUIRED COMPONENTS system thread)

generate_dynamic_reconfigure_options(
  cfg/TerarangerOne.cfg
  cfg/TerarangerDuo.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs dynamic_reconfigure
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

add_executable(teraranger_duo src/teraranger_duo.cpp src/serial_port.cpp)
add_dependencies(teraranger_duo  ${catkin_EXPORTED_TARGETS})
add_dependencies(teraranger_duo ${PROJECT_NAME}_gencfg)
target_link_libraries(teraranger_duo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(teraranger_one src/teraranger_one.cpp src/serial_port.cpp)
add_dependencies(teraranger_one  ${catkin_EXPORTED_TARGETS})
add_dependencies(teraranger_one ${PROJECT_NAME}_gencfg)
target_link_libraries(teraranger_one ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install( TARGETS
  teraranger_evo
  teraranger_one
  teraranger_duo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
