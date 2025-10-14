find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rerun_sdk REQUIRED)

set(EXECUTABLE_TARGETS
  exp_pixel_noise
)

foreach(EXECUTABLE_TARGET ${EXECUTABLE_TARGETS})
  add_executable(${EXECUTABLE_TARGET} exp/${EXECUTABLE_TARGET}.cpp)
  target_link_libraries(${EXECUTABLE_TARGET} PRIVATE ${PROJECT_NAME} yaml-cpp ${OpenCV_LIBS} rerun_sdk)
  target_include_directories(${EXECUTABLE_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endforeach()
