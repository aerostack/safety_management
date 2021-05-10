cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME recovery_manager_process)
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


set(RECOVERY_MANAGER_PROCESS_SOURCE_DIR
        src/source)

set(RECOVERY_MANAGER_PROCESS_INCLUDE_DIR
        src/include)

set(RECOVERY_MANAGER_PROCESS_TEST_DIR
        src/test
)

set(RECOVERY_MANAGER_PROCESS_HEADER_FILES
        ${RECOVERY_MANAGER_PROCESS_INCLUDE_DIR}/recovery_manager_process.h
)

set(RECOVERY_MANAGER_PROCESS_SOURCE_FILES
        ${RECOVERY_MANAGER_PROCESS_SOURCE_DIR}/recovery_manager_process.cpp
        ${RECOVERY_MANAGER_PROCESS_SOURCE_DIR}/recovery_manager_process_main.cpp

)


find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs 
    robot_process 
    droneMsgsROS 
    geometry_msgs
    aerostack_msgs
    behavior_coordinator_msgs
    belief_manager_msgs
)

catkin_package(
        INCLUDE_DIRS ${RECOVERY_MANAGER_PROCESS_INCLUDE_DIR}
        LIBRARIES recovery_manager_process
        CATKIN_DEPENDS 
        roscpp
        std_msgs
        aerostack_msgs 
        robot_process 
        droneMsgsROS
	behavior_coordinator_msgs
        belief_manager_msgs
        DEPENDS yaml-cpp
  )

include_directories(${RECOVERY_MANAGER_PROCESS_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})


add_library(recovery_manager_process ${RECOVERY_MANAGER_PROCESS_SOURCE_FILES} ${RECOVERY_MANAGER_PROCESS_HEADER_FILES})
add_dependencies(recovery_manager_process ${catkin_EXPORTED_TARGETS})
target_link_libraries(recovery_manager_process ${catkin_LIBRARIES})


add_executable(recovery_manager_process_main ${RECOVERY_MANAGER_PROCESS_SOURCE_DIR}/recovery_manager_process_main.cpp)
add_dependencies(recovery_manager_process_main ${catkin_EXPORTED_TARGETS})
target_link_libraries(recovery_manager_process_main recovery_manager_process)
target_link_libraries(recovery_manager_process_main ${catkin_LIBRARIES})
target_link_libraries(recovery_manager_process yaml-cpp)
