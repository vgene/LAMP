# Project
cmake_minimum_required(VERSION 3.4.3)
project(LAMP)

# Programming languages to use
enable_language(C CXX)

# Find and link with LLVM
find_package(LLVM REQUIRED CONFIG)
message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")
message(STATUS "Using LLVMConfig.cmake in: ${LLVM_DIR}")

# Find and set SCAF
if(DEFINED ENV{SCAF_INSTALL_DIR})
    set(SCAF_INSTALL_DIR $ENV{SCAF_INSTALL_DIR})
    message(STATUS "Found SCAF at ${SCAF_INSTALL_DIR}")
else()
    message(FATAL_ERROR "Cannot find SCAF, did you export $SCAF_INSTALL_DIR?")
endif()

# Find and set NOELLE
if(DEFINED ENV{NOELLE_INSTALL_DIR})
  set(NOELLE_INSTALL_DIR $ENV{NOELLE_INSTALL_DIR})
  message(STATUS "Found NOELLE at ${NOELLE_INSTALL_DIR}")
else()
  message(FATAL_ERROR "Cannot find Noelle, did you export $NOELLE_INSTALL_DIR?")
endif()

# Link with LLVM
include_directories(${LLVM_INCLUDE_DIRS})
add_definitions(${LLVM_DEFINITIONS})

# Prepare the pass to be included in the source tree
list(APPEND CMAKE_MODULE_PATH "${LLVM_CMAKE_DIR}")
include(AddLLVM)

# All include files for CPF
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_BINARY_DIR}/include
                    ${CMAKE_INSTALL_PREFIX}/include
                    ${SCAF_INSTALL_DIR}/include
                    ${NOELLE_INSTALL_DIR}/include)

# Pass
add_subdirectory(lib)

# Install
INSTALL (
   DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ ${CMAKE_CURRENT_BINARY_DIR}/include/ 
   DESTINATION include
   FILES_MATCHING PATTERN "*.h*")
