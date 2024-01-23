cmake_minimum_required(VERSION 3.11)

project(mujoco_contact_surface_sensors)
add_definitions(-std=c++17)


set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined -fno-lto " )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcf-protection=none")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcf-protection=none")
set(CMAKE_CXX_EXTENSIONS OFF)

option(USE_SSE "Use SSE4 instructions, if available" ON)
option(ENABLE_TACT_BENCHMARK "Build tactile sensors with benchmarking support" OFF)


set(SURFACE_SENSORS_DEP_VERSION_VCGLIB
    edf834918fa5964096216a7d458eda08768d49ae
    CACHE STRING "Version of 'vcg' to be fetched."
)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  mujoco_contact_surfaces
  tactile_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  config/DynamicFlatTactile.cfg
)

include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

find_package(drake REQUIRED)
find_package(OpenMP REQUIRED)

include(FetchContent)
if(NOT vcg_DIR)
  FetchContent_Declare(
    vcg
    GIT_REPOSITORY https://github.com/cnr-isti-vclab/vcglib.git
    GIT_TAG ${SURFACE_SENSORS_DEP_VERSION_VCGLIB}
  )

  FetchContent_GetProperties(vcg)
  if(NOT vcg_POPULATED)
    FetchContent_Populate(vcg)
    # This is not a CMake project.
    set(vcg_DIR ${vcg_SOURCE_DIR})
  endif()
endif()

add_service_files(
  FILES
  GetTactileState.srv
)

generate_messages(
  DEPENDENCIES
    tactile_msgs
    std_msgs
)

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
    ${vcg_DIR}
)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(CheckSSE)
FindSSE()

add_library(${PROJECT_NAME}
  src/tactile_sensor_base.cpp
  src/flat_tactile_sensor.cpp
  src/taxel_sensor.cpp
  src/curved_sensor.cpp
  src/bvh.cpp
)

if((SSE4_1_FOUND OR SSE4_2_FOUND) AND USE_SSE)
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_SSE)
  message(STATUS "Building with SSE4")
  if(MSVC)
      target_compile_options(${PROJECT_NAME} PRIVATE /arch:SSE4.1)
  else()
      target_compile_options(${PROJECT_NAME} PRIVATE -msse4.1)
  endif()
elseif(USE_SSE)
  message(WARNING "SSE4 not found, building without SSE4")
endif()

if(ENABLE_TACT_BENCHMARK)
  message(STATUS "Building with benchmarking support")
  set(BENCHMARK_TACTILE ON)
  target_compile_definitions(${PROJECT_NAME} PUBLIC BENCHMARK_TACTILE)
endif()


target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  include
  ${mujoco_contact_surfaces_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${drake_INCLUDE_DIRS}
  ${vcg_DIR}
)
target_link_libraries(${PROJECT_NAME}
  ${mujoco_contact_surfaces_LIBRARIES}
  ${catkin_LIBRARIES}
  drake::drake
  OpenMP::OpenMP_CXX
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

if(ENABLE_TACT_BENCHMARK)
  add_subdirectory(src/benchmark)
endif()


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
