cmake_minimum_required(VERSION 2.8)
project(test_opencv)

set(CMAKE_CXX_STANDARD 11)

add_compile_options(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -std=gnu++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++11")
add_definitions(-DBOOST_LOG_DYN_LINK=1)
find_package(OpenCV 3.0 QUIET)
if (NOT OpenCV_FOUND)
  find_package(OpenCV 2.4.3 QUIET)
  if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
  endif ()
endif ()
find_package(Eigen3  REQUIRED)
find_package(GTest REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${GTEST_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
) 

set(SOURCE_FILES main.cpp)
add_executable(test_relativepose ${SOURCE_FILES})
target_link_libraries(test_relativepose
${OpenCV_LIBS} 
glog
gtest
pthread
${EIGEN3_LIBS}
)

add_executable(test_scale test_scale.cpp)
target_link_libraries(test_scale
${OpenCV_LIBS} 
glog
gtest
pthread
${EIGEN3_LIBS}
)

