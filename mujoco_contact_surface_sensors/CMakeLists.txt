cmake_minimum_required(VERSION 3.0.2)

project(mujoco_contact_surface_sensors)
add_definitions(-std=c++17)


set( CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined" )

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  mujoco_contact_surfaces
  tactile_msgs
)
find_package(drake REQUIRED)

catkin_package(
 CATKIN_DEPENDS
  pluginlib
  mujoco_contact_surfaces
  tactile_msgs
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
)

include_directories(
    include
    ${mujoco_contact_surfaces_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${drake_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mujoco_contact_surface_sensors_plugin.cpp
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  include
  ${mujoco_contact_surfaces_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${drake_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${mujoco_contact_surfaces_LIBRARIES}
  ${catkin_LIBRARIES}
  drake::drake
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
    mujoco_contact_surface_sensors_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
