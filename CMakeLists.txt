cmake_minimum_required(VERSION 3.7.2)

project(pybricks_ev3_ffi_demo)

cmake_policy(SET CMP0069 NEW) 
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)


MESSAGE(WARNING "Using toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
MESSAGE(WARNING "Binary dir: ${CMAKE_BINARY_DIR}")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

MESSAGE(WARNING "${CMAKE_CXX_FLAGS}")
set(CMAKE_C_FLAGS "-fdata-sections -ffunction-sections -Wall -Wmaybe-uninitialized")
set(CMAKE_CXX_FLAGS "-fno-exceptions -fdata-sections -ffunction-sections")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GenerateExportHeader)
add_library(pid SHARED pid.c)
generate_export_header(pid)

target_link_libraries(pid PUBLIC -Wl,--gc-sections)

include(CheckIPOSupported)
check_ipo_supported(RESULT result OUTPUT output)
if(result)
  # set_property(TARGET pid PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
  message(WARNING "IPO is not supported: ${output}")
endif()
 