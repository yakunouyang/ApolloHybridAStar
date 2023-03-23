cmake_minimum_required(VERSION 3.10)
project(apollo_ha)

set(CMAKE_CXX_STANDARD 11)
#if(MSVC)
#  add_compile_options(/Ox)
#else()
#  add_compile_options(-O3)
#endif()


include_directories(
  ./
  third_party/
)

file(GLOB ha_SRC
  "math/*.cc"
  "parking_planner/*.cc"
  )

find_package(Matlab)

if (${Matlab_FOUND})
  add_definitions(-DNO_VISUAL)
  matlab_add_mex(NAME apollo_ha SRC ${ha_SRC} main.cpp my_env.cpp)
endif()

if(EXISTS /opt/ros/)
  list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic/")
  find_package(visualization_msgs REQUIRED)
  include_directories(include ${visualization_msgs_INCLUDE_DIRS})

  add_definitions(-DROS_VISUALIZATION)
  find_package(roscpp REQUIRED)
  include_directories(${roscpp_INCLUDE_DIRS})

  add_executable(apollo_ha_test ${ha_SRC} test.cpp
    visualization/color.cc visualization/plot_ros.cc)
  target_link_libraries(apollo_ha_test ${roscpp_LIBRARIES})
else()
  add_executable(apollo_ha_test ${ha_SRC} test.cpp
    visualization/color.cc visualization/plot_ws.cc visualization/publisher.cc visualization/easywsclient.cpp)
endif()