cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(mbot_autonomy)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(Threads REQUIRED)
find_package(lcm REQUIRED)
find_package(GTK2 2.6 REQUIRED gtk)
find_package(mbot_lcm_msgs)

include(${LCM_USE_FILE})

# Common utils.
add_library(common_utils STATIC
  src/utils/geometric/pose_trace.cpp
  src/utils/getopt.c
  src/utils/timestamp.c
  src/utils/zarray.c
  src/utils/zhash.c
)
target_link_libraries(common_utils
  ${GTK2_LIBRARIES}
)
target_include_directories(common_utils PUBLIC
  ${GTK2_INCLUDE_DIRS}
  include
)

# MOTION CONTROLLER
set(MBOT_TYPE "OMNI" CACHE STRING "Type of MBot motion controller to build (OMNI or DIFF)")
set_property(CACHE MBOT_TYPE PROPERTY STRINGS "OMNI" "DIFF")

# Define the source file based on MBOT_TYPE
if(${MBOT_TYPE} STREQUAL "OMNI")
  set(MOTION_CONTROLLER_SRC src/mbot/omni_motion_controller.cpp)
elseif(${MBOT_TYPE} STREQUAL "DIFF")
  set(MOTION_CONTROLLER_SRC src/mbot/diff_motion_controller.cpp)
else()
  message(FATAL_ERROR "Invalid MBOT_TYPE: ${MBOT_TYPE}. Must be 'OMNI' or 'DIFF'.")
endif()

add_executable(mbot_motion_controller ${MOTION_CONTROLLER_SRC})
target_link_libraries(mbot_motion_controller
  common_utils
  lcm
)
target_include_directories(mbot_motion_controller PRIVATE
  include
)

# SLAM
add_executable(mbot_slam src/slam/slam_main.cpp
  src/slam/action_model.cpp
  src/slam/mapping.cpp
  src/slam/moving_laser_scan.cpp
  src/slam/occupancy_grid.cpp
  src/slam/particle_filter.cpp
  src/slam/sensor_model.cpp
  src/slam/slam.cpp
)
target_link_libraries(mbot_slam
  ${CMAKE_THREAD_LIBS_INIT}
  common_utils
  lcm
)
target_include_directories(mbot_slam PRIVATE
  include
)

# EXPLORATION
add_executable(exploration src/planning/exploration_main.cpp
                           src/planning/exploration.cpp
                           src/planning/motion_planner.cpp
                           src/planning/frontiers.cpp
                           src/slam/occupancy_grid.cpp
                           src/planning/obstacle_distance_grid.cpp
                           src/planning/astar.cpp
)
target_link_libraries(exploration
  mbot_lcm_msgs-cpp
  common_utils
  lcm
  ${CMAKE_THREAD_LIBS_INIT}
)
target_include_directories(exploration PRIVATE
  include
  /usr/local/include
)

# PLANNING SERVER
add_executable(motion_planning_server src/planning/motion_planner_server_main.cpp
                                      src/planning/motion_planner.cpp
                                      src/planning/motion_planner_server.cpp
                                      src/slam/occupancy_grid.cpp
                                      src/planning/obstacle_distance_grid.cpp
                                      src/planning/astar.cpp
)
target_link_libraries(motion_planning_server
  mbot_lcm_msgs-cpp
  common_utils
  lcm
  ${CMAKE_THREAD_LIBS_INIT}
)
target_include_directories(motion_planning_server PRIVATE
  include
)

# Obstacle_Distance_Grid-TESTER
add_executable(odg_test   src/planning/obstacle_distance_grid_test.cpp
  src/slam/occupancy_grid.cpp
  src/planning/obstacle_distance_grid.cpp
)
target_link_libraries(odg_test
  common_utils
  lcm
  ${CMAKE_THREAD_LIBS_INIT}
)
target_include_directories(odg_test PRIVATE
  include
)

# ASTAR-TESTER
add_executable(astar_test src/planning/astar_test.cpp
  src/planning/astar.cpp
  src/slam/occupancy_grid.cpp
  src/planning/obstacle_distance_grid.cpp
  src/planning/motion_planner.cpp
)
target_link_libraries(astar_test
  common_utils
  lcm
  ${CMAKE_THREAD_LIBS_INIT}
)
target_include_directories(astar_test PRIVATE
  include
)

# TODO: Remove from this project. Moved to LCM base repo.
# TIMESYNC
# add_executable(timesync src/mbot/timesync.cpp
# )
# target_link_libraries(timesync
#   common_utils
#   lcm
# )
# target_include_directories(timesync PRIVATE
#   include
# )

# This is needed to find the shared libraries correctly on RPi OS.
set_target_properties(mbot_slam mbot_motion_controller PROPERTIES
    INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib
    BUILD_WITH_INSTALL_RPATH 1
)

# INSTALL needed binaries.
install(
  TARGETS mbot_slam mbot_motion_controller
  DESTINATION bin
)


add_executable(drive_square src/mbot/drive_square.cpp)
target_link_libraries(drive_square
  common_utils
  lcm
)
target_include_directories(drive_square PRIVATE
  include
)

add_executable(drive_maze src/mbot/drive_maze.cpp)
target_link_libraries(drive_maze
  common_utils
  lcm
)
target_include_directories(drive_maze PRIVATE
  include
)

add_executable(drive_event1 src/mbot/drive_event1.cpp)
target_link_libraries(drive_event1
  common_utils
  lcm
)
target_include_directories(drive_event1 PRIVATE
  include
)