cmake_minimum_required(VERSION 2.8.3)
project(example_ros_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## If your project needs additional non-catkin dependencies, use CMake's convention
# find_package(Boost REQUIRED COMPONENTS system)

## The following commented sections show how to configure building of custom messages
## and services, in case you want to add them into your project.

## Generate messages in the 'msg' folder
# add_message_files(FILES
#   Message1.msg
# )

## Generate services in the 'srv' folder
# add_service_files(FILES
#   Service1.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(DEPENDENCIES
#   std_msgs  # or other packages containing msgs
# )

## The macro catkin_package() specifies catkin-specific information, such as exported
## headers, exported libraries and dependencies on other packages. This function must
## be called before declaring any targets with add_library() or add_executable().
catkin_package(
  ## the directories containing the headers exported by your project are 
  ## specified using INCLUDE_DIRS
  INCLUDE_DIRS include
  ## if you project exports any library targets, specify them using LIBRARIES  
  # LIBRARIES example_ros_library
  CATKIN_DEPENDS roscpp std_msgs
  ## dependencies on non-catkin packages, if any, are specified using DEPENDS
  # DEPENDS OpenCV
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Define a variabale for all source files
set(example_ros_cpp_node_SRCS
      src/pid_controller_base.cpp
      src/pid_controller_ros.cpp 
      src/nodes/pid_controller_node.cpp
)

## A library target would be specified in the following way
# add_library(example_ros_library ${example_ros_library_SRCS})

## Declare a C++ executable target
add_executable(pid_controller_node ${example_ros_cpp_node_SRCS})

## If building the target requires building another target (e.g. another project's messages), add it as a dependency
# add_dependencies(example_ros_cpp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pid_controller_node
  ${catkin_LIBRARIES}
  ## If the executable target depends on a library, it needs to be linked against it here
  # auxiliary_library
)
