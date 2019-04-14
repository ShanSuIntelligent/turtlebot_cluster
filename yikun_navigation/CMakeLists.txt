cmake_minimum_required(VERSION 2.8.3)
project(yikun_navigation)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  yikun_cluster_msgs
  tf
  image_geometry
  image_transport
  message_filters
  roslint
  tf2
  tf2_geometry_msgs
  tf2_ros
  xmlrpcpp
  yikun_common
)
find_package(Boost REQUIRED
  COMPONENTS
  filesystem
  program_options
  signals
  system
  thread
)

#####mysql###########
set(MYSQL_INCLUDE_DIRS /usr/include/mysql)
set(MYSQL_LIBRARY_DIRS /usr/lib/x86_64-linux-gnu -lmysqlclient -lpthread -lz -lm -ldl)

#########eigen3###########
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ekf
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${MYSQL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${TINKERFORGE_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})


###########导航###################
#路径跟踪
add_executable(planner_node
  src/planner.cpp
  src/local_planner/path_follower.cpp)
target_link_libraries(planner_node db_helper
  ${catkin_LIBRARIES}
)
