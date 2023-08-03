cmake_minimum_required(VERSION 3.0.2)

project(mujoco_contact_surface_sensors)
add_definitions(-std=c++17)


set( CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined -fno-lto" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcf-protection=none")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcf-protection=none")
set(CMAKE_CXX_EXTENSIONS OFF)

option(USE_SSE "Use SSE4.1 instructions, if available" ON)

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

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(CheckSSE)
FindSSE()

include_directories(
    include
    ${mujoco_contact_surfaces_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${drake_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/tactile_sensor_base.cpp
  src/flat_tactile_sensor.cpp
  src/taxel_sensor.cpp
  src/bvh.cpp
)

if(SSE4_1_FOUND AND USE_SSE)
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_SSE ON)
  message(STATUS "Building with SSE4")
  if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /arch:SSE4.1)
  else()
    target_compile_options(${PROJECT_NAME} PRIVATE -msse4.1)
  endif()
elseif(USE_SSE)
  message(WARNING "SSE4.1 not found, building without SSE")
endif()

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
