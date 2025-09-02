find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

set(EXECUTABLE_TARGETS
  noise_exp
  exp_pixel_noise
  exp_time_noise
  exp_rotation_noise
  exp_observation
  exp_time_window
  exp_running_time
)

foreach(EXECUTABLE_TARGET ${EXECUTABLE_TARGETS})
  add_executable(${EXECUTABLE_TARGET} exp/${EXECUTABLE_TARGET}.cpp)
  target_link_libraries(${EXECUTABLE_TARGET} PRIVATE ${PROJECT_NAME} yaml-cpp ${OpenCV_LIBS})
  target_include_directories(${EXECUTABLE_TARGET} PRIVATE ${PROJECT_SOURCE_DIR}/experiments/include)
endforeach()
