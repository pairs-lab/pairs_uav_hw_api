cmake_minimum_required(VERSION 3.5)
project(pairs_uav_hw_api)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Wextra)

if(COVERAGE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

set(CATKIN_DEPENDENCIES
  cmake_modules
  geometry_msgs
  pairs_lib
  pairs_msgs
  nav_msgs
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})

set(LIBRARIES
  PairsUavHwApi_HwApi
  PairsUavHwApi_DummyApi
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(PairsUavHwApi_HwApi src/hw_api_manager.cpp)
add_dependencies(PairsUavHwApi_HwApi
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(PairsUavHwApi_HwApi ${catkin_LIBRARIES})

add_library(PairsUavHwApi_DummyApi src/dummy_api.cpp)
add_dependencies(PairsUavHwApi_DummyApi
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(PairsUavHwApi_DummyApi ${catkin_LIBRARIES})

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/pairs_uav_hw_api/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES plugins.xml nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
