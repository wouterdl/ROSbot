cmake_minimum_required(VERSION 2.8.3)
project(stop_pkg)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
roscpp
sensor_msgs
geometry_msgs
tf
std_msgs

)
# find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stop_pkg
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
# include
${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/stop_pkg.cpp
# )


add_executable(${PROJECT_NAME}_node src/stop_pkg_node.cpp)

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
 )
