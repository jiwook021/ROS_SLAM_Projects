CMAKE_MINIMUM_REQUIRED(VERSION 3.15)
project(ceres_tutorial)

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(Ceres REQUIRED)

include_directories(
  ${CERES_INCLUDE_DIRS}
  "/usr/include/eigen3/"
)

add_executable(ceres_tutorial ceres_tutorial.cpp)
target_link_libraries(ceres_tutorial ${CERES_LIBRARIES})

