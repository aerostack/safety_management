cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME safety_monitor_process)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)
#add_definitions(-std=c++0x)
#add_definitions(-std=c++03)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries


set(SAFETYMONITORPROCESS_SOURCE_DIR
        src/source)

set(SAFETYMONITORPROCESS_INCLUDE_DIR
        src/include)

set(SAFETYMONITORPROCESS_TEST_DIR
        src/test
)

set(SAFETYMONITORPROCESS_HEADER_FILES
        ${SAFETYMONITORPROCESS_INCLUDE_DIR}/safety_monitor_process.h
)

set(SAFETYMONITORPROCESS_SOURCE_FILES
        ${SAFETYMONITORPROCESS_SOURCE_DIR}/safety_monitor_process.cpp
        ${SAFETYMONITORPROCESS_SOURCE_DIR}/safety_monitor_process_main.cpp

)


find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs 
    robot_process 
    droneMsgsROS 
    geometry_msgs
    aerostack_msgs
    behavior_execution_manager_msgs
    belief_manager_msgs
)

catkin_package(
        INCLUDE_DIRS ${SAFETYMONITORPROCESS_INCLUDE_DIR}
        LIBRARIES safety_monitor_process
        CATKIN_DEPENDS 
        roscpp
        std_msgs
        aerostack_msgs
        behavior_execution_manager_msgs
        robot_process 
        droneMsgsROS
        belief_manager_msgs
        DEPENDS yaml-cpp
  )

include_directories(${SAFETYMONITORPROCESS_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})


add_library(safety_monitor_process ${SAFETYMONITORPROCESS_SOURCE_FILES} ${SAFETYMONITORPROCESS_HEADER_FILES})
add_dependencies(safety_monitor_process ${catkin_EXPORTED_TARGETS})
target_link_libraries(safety_monitor_process ${catkin_LIBRARIES})


add_executable(safety_monitor_process_main ${SAFETYMONITORPROCESS_SOURCE_DIR}/safety_monitor_process_main.cpp)
add_dependencies(safety_monitor_process_main ${catkin_EXPORTED_TARGETS})
target_link_libraries(safety_monitor_process_main safety_monitor_process)
target_link_libraries(safety_monitor_process_main ${catkin_LIBRARIES})
target_link_libraries(safety_monitor_process yaml-cpp)
