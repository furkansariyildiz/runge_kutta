cmake_minimum_required(VERSION 3.0.2)
project(runge_kutta)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES runge_kutta
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(runge_kutta INTERFACE)
target_include_directories(runge_kutta INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(rk4_demo_node src/rk4_demo_node.cpp)
target_link_libraries(rk4_demo_node runge_kutta ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS rk4_demo_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_rk4
    test/test_main.cpp
    test/test_rk4_step.cpp
    test/test_integrate.cpp
    test/test_convergence.cpp
  )
  if(TARGET test_rk4)
    target_link_libraries(test_rk4 runge_kutta)
  endif()
endif()
