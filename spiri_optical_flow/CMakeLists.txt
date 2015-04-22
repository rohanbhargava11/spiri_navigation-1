cmake_minimum_required(VERSION 2.8.3)
project(spiri_optical_flow)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  geometry_msgs
  tf
)

find_package(OpenCV REQUIRED)

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES optflow
#  CATKIN_DEPENDS cv_bridge image_geometry image_transport geometry_msgs
#  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#add_library(optflow src/${PROJECT_NAME}/optflow.cpp)

add_executable(optflow_node src/optflow_node.cpp)

target_link_libraries(optflow_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(dense_optflow_node src/dense_optflow.cpp)

target_link_libraries(dense_optflow_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS dense_optflow_node optflow_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
