cmake_minimum_required(VERSION 3.10)
project(apollo_ha)

set(CMAKE_CXX_STANDARD 11)
if(MSVC)
  add_compile_options(/Ox)
else()
  add_compile_options(-O3)
endif()

find_package(Matlab REQUIRED)

include_directories(./)

file(GLOB ha_SRC
  "math/*.cc"
  "parking_planner/*.cc"
  )

message(INFO ${ha_SRC})

matlab_add_mex(NAME apollo_ha SRC ${ha_SRC} main.cpp)

add_executable(apollo_ha_test ${ha_SRC} test.cpp)