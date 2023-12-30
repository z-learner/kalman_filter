cmake_minimum_required ( VERSION 3.10 )

project(kalman_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)



include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Python3_INCLUDE_DIRS})
include_directories(${Python3_INCLUDE_DIRS})
include_directories(/usr/local/lib/python3.10/dist-packages/numpy/core/include)

message("Python3_INCLUDE_DIRS = ${Python3_INCLUDE_DIRS}")


add_executable(test_kf test/test_linear_kalman.cc)
target_link_libraries(test_kf ${Python3_LIBRARIES})