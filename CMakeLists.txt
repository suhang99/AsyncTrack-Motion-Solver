cmake_minimum_required(VERSION 3.20)
project(motion-solver VERSION 0.1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_EXPERIMENTS "Build experiments" ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### Require out-of-source builds
file(TO_CMAKE_PATH "${PROJECT_BINARY_DIR}/CMakeLists.txt" LOC_PATH)
if(EXISTS "${LOC_PATH}")
    message(FATAL_ERROR "You cannot build in a source directory (or any directory with a CMakeLists.txt file). Please make a build subdirectory. Feel free to remove CMakeCache.txt and CMakeFiles.")
endif()

# -------- Find thirdparty library -------- #
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Ceres 2 REQUIRED)

# ------------- Build library ------------- #
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "src/*.cpp")
add_library(motion-solver SHARED ${SOURCES})
target_include_directories(
    motion-solver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(motion-solver PUBLIC Eigen3::Eigen Ceres::ceres OpenMP::OpenMP_CXX)

# ------------ Build experiments ---------- #
if(BUILD_EXPERIMENTS)
    add_subdirectory(experiments)
endif()

# -------- Install & Export Targets ------- #
include(CMakePackageConfigHelpers)

install(TARGETS motion-solver
    EXPORT motion-solverTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT motion-solverTargets
    NAMESPACE motion-solver::
    DESTINATION lib/cmake/motion-solver
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/motion-solverConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/motion-solverConfig.cmake
    INSTALL_DESTINATION lib/cmake/motion-solver
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/motion-solverConfig.cmake
    DESTINATION lib/cmake/motion-solver
)

# Create a version file
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/motion-solverConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/motion-solverConfigVersion.cmake
    DESTINATION lib/cmake/motion-solver
)
