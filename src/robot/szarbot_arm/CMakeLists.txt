cmake_minimum_required(VERSION 3.0.2)
project(szarbot_arm)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
  geometry_msgs
  arebot_control
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES szarbot_arm
  CATKIN_DEPENDS nodelet roscpp arebot_control std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/ArmNode.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()
add_dependencies(${PROJECT_NAME} arebot_control_gencpp)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
