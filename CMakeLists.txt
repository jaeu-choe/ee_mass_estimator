cmake_minimum_required(VERSION 3.0.2)
project(ee_mass_estimator)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs message_generation)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  OdarWrench.msg
  EEMass.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_library(ee_mass_lib src/least_square.cpp)
target_link_libraries(ee_mass_lib PUBLIC Eigen3::Eigen)
target_include_directories(ee_mass_lib PUBLIC include)

add_executable(ee_mass_node src/ee_mass_node.cpp)
target_include_directories(
  ee_mass_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
target_link_libraries(ee_mass_node ${catkin_LIBRARIES} Eigen3::Eigen ee_mass_lib)







