cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
project(teraranger)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  serial
  teraranger_array
)

find_package(Boost REQUIRED COMPONENTS system thread)

generate_dynamic_reconfigure_options(
  cfg/TerarangerOne.cfg
  cfg/TerarangerDuo.cfg
  cfg/Evo64px.cfg
  cfg/EvoThermal.cfg
  cfg/EvoMini.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs dynamic_reconfigure serial teraranger_array
  DEPENDS Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(evo_mini src/teraranger_evo_mini.cpp src/helper_lib.cpp)
target_link_libraries(evo_mini ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(evo_mini ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(evo_mini ${catkin_EXPORTED_TARGETS})
add_dependencies(evo_mini ${PROJECT_NAME}_gencfg)

add_executable(evo src/teraranger_evo.cpp src/helper_lib.cpp)
target_link_libraries(evo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(evo ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(evo ${PROJECT_NAME}_gencfg)

add_executable(duo src/teraranger_duo.cpp src/helper_lib.cpp)
add_dependencies(duo ${catkin_EXPORTED_TARGETS})
add_dependencies(duo ${PROJECT_NAME}_gencfg)
target_link_libraries(duo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(one src/teraranger_one.cpp src/helper_lib.cpp)
add_dependencies(one  ${catkin_EXPORTED_TARGETS})
add_dependencies(one ${PROJECT_NAME}_gencfg)
target_link_libraries(one ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install( TARGETS
  evo_mini
  evo
  one
  duo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
