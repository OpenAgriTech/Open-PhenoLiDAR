cmake_minimum_required(VERSION 2.8.3)
project(lms400_driver)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(PkgConfig REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if (DEFINED CATKIN_DEVEL_PREFIX)
  find_package(catkin REQUIRED)
  message(STATUS "Catkin env set, compiling ${PROJECT_NAME} with ROS")

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES lms400_driver
    CATKIN_DEPENDS
		roscpp 
		std_msgs sensor_msgs geometry_msgs
  )
else()
  message(STATUS "Catkin env not set, compiling ${PROJECT_NAME} without ROS")
endif()

add_library(lms400_driver STATIC
  src/sick_lms400.cpp
  src/sick_lms400_node.cpp)
set_target_properties(lms400_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(lms400_driver PUBLIC ${catkin_LIBRARIES} ${ROS_LIBRARIES})
add_dependencies(lms400_driver ${catkin_EXPORTED_TARGETS})

target_include_directories(lms400_driver PUBLIC include)

add_executable(sick_lms400 src/sick_lms400_node.cpp)
target_link_libraries(sick_lms400 lms400_driver ${catkin_LIBRARIES})

install(TARGETS lms400_driver
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    TARGETS
      sick_lms400
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
