cmake_minimum_required(VERSION 2.8.3)
project(socketcan_bridge_fixed)

find_package(catkin REQUIRED
  COMPONENTS
    can_msgs
    roscpp
    socketcan_interface
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    socketcan_to_topic
    topic_to_socketcan
  CATKIN_DEPENDS
    can_msgs
    roscpp
    socketcan_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# socketcan_to_topic
add_library(socketcan_to_topic
  src/socketcan_to_topic.cpp
)
target_link_libraries(socketcan_to_topic
  ${catkin_LIBRARIES}
)
add_dependencies(socketcan_to_topic
  ${catkin_EXPORTED_TARGETS}
)

# topic_to_socketcan
add_library(topic_to_socketcan
  src/topic_to_socketcan.cpp
)
target_link_libraries(topic_to_socketcan
  ${catkin_LIBRARIES}
)
add_dependencies(topic_to_socketcan
  ${catkin_EXPORTED_TARGETS}
)

# socketcan_to_topic_node
add_executable(socketcan_to_topic_node
  src/socketcan_to_topic_node.cpp
)
target_link_libraries(socketcan_to_topic_node
  socketcan_to_topic
  ${catkin_LIBRARIES}
)

# topic_to_socketcan_node
add_executable(topic_to_socketcan_node
  src/topic_to_socketcan_node.cpp

)
target_link_libraries(topic_to_socketcan_node
  topic_to_socketcan
  ${catkin_LIBRARIES}
)

# socketcan_bridge_node
add_executable(socketcan_bridge_node
  src/socketcan_bridge_node.cpp
)
target_link_libraries(socketcan_bridge_node
  topic_to_socketcan
  socketcan_to_topic
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    socketcan_bridge_node
    socketcan_to_topic
    socketcan_to_topic_node
    topic_to_socketcan
    topic_to_socketcan_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
