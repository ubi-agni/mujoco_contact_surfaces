add_executable(benchmark_flat
  benchmark_flat.cpp
)
target_include_directories(benchmark_flat SYSTEM PRIVATE
  ${PROJECT_NAME}
)
target_link_libraries(benchmark_flat
  ${PROJECT_NAME}
)

install(TARGETS benchmark_flat
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)