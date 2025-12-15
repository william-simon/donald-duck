cmake_minimum_required(VERSION 3.16.0)
project(standalone_example C CXX)

# Add cmake directory to search for custom cmake functions
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

# CXX options
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 17)

include(../../cmake/CPM.cmake)
include(../../cmake/GitUtils.cmake)

# boost
set(BOOST_INCLUDE_LIBRARIES "graph")

CPMAddPackage(
  NAME Boost
  VERSION 1.86.0
  URL https://github.com/boostorg/boost/releases/download/boost-1.86.0/boost-1.86.0-cmake.tar.xz
  OPTIONS "BOOST_ENABLE_CMAKE ON"
)
CPMAddPackage(
  NAME GHL
  GIT_TAG Tunaberk-Almaci
  GIT_REPOSITORY [IBM]
  DOWNLOAD_ONLY YES
)
include_directories("${GHL_SOURCE_DIR}/include")
execute_process(
  COMMAND git apply "${PROJECT_SOURCE_DIR}/vf3.patch"
  WORKING_DIRECTORY "${GHL_SOURCE_DIR}"
  RESULT_VARIABLE PATCH_RESULT
  OUTPUT_VARIABLE PATCH_OUTPUT
  ERROR_VARIABLE PATCH_ERROR
)

add_executable(02_isomorphism 02_isomorphism.cpp)

foreach(boost_lib ${BOOST_INCLUDE_LIBRARIES})
  if(TARGET Boost::${boost_lib})
    target_link_libraries(02_isomorphism PRIVATE Boost::${boost_lib})
  endif()
endforeach()
