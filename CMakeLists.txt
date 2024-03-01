cmake_minimum_required(VERSION 3.7.2)

project(pybricks_ev3_ffi_demo)

MESSAGE(STATUS "Using toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
MESSAGE(STATUS "Binary dir: ${CMAKE_BINARY_DIR}")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(abseil-cpp)

add_library(pid SHARED pid.cc)
target_link_libraries(pid -static absl::algorithm absl::base absl::debugging absl::flat_hash_map absl::flags absl::memory absl::meta absl::numeric absl::random_random absl::strings  absl::synchronization absl::time absl::utility)