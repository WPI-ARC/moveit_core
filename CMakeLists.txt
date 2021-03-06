cmake_minimum_required(VERSION 2.8.3)
project(moveit_core)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread iostreams)
find_package(catkin REQUIRED 
COMPONENTS
  moveit_msgs
  resource_retriever
  urdfdom
  urdfdom_headers
  srdfdom
  fcl
  kdl_parser
  geometric_shapes
  shape_tools
  eigen_stl_containers 
  eigen_conversions
  random_numbers
  console_bridge
  visualization_msgs
  rostime
)

find_package(octomap REQUIRED)

# This is where the version file will be generated
set(VERSION_FILE_PATH "${CMAKE_CURRENT_BINARY_DIR}/version")
file(MAKE_DIRECTORY "${VERSION_FILE_PATH}")

set(THIS_PACKAGE_INCLUDE_DIRS 
    ${VERSION_FILE_PATH}
    background_processing/include
    exceptions/include
    collision_detection/include
    collision_detection_fcl/include
    constraint_samplers/include
    controller_manager/include
    distance_field/include
    dynamics_solver/include
    kinematics_base/include
    kinematics_metrics/include
    robot_model/include
    transforms/include
    robot_state/include
    robot_trajectory/include
    kinematic_constraints/include
    macros/include
    planning_interface/include
    planning_request_adapter/include
    planning_scene/include
    profiler/include
    sensor_manager/include
    trajectory_processing/include
)

catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
  LIBRARIES
    moveit_exceptions
    moveit_background_processing
    moveit_kinematics_base
    moveit_robot_model
    moveit_transforms
    moveit_robot_state
    moveit_robot_trajectory
    moveit_planning_interface
    moveit_collision_detection
    moveit_collision_detection_fcl
    moveit_kinematic_constraints
    moveit_planning_scene
    moveit_constraint_samplers
    moveit_planning_request_adapter
    moveit_profiler
    moveit_trajectory_processing
    moveit_distance_field
    moveit_kinematics_metrics
    moveit_dynamics_solver
    ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS
    geometric_shapes
    shape_tools
    eigen_stl_containers
    eigen_conversions
    random_numbers
    moveit_msgs
    kdl_parser
    srdfdom
  DEPENDS
    Boost
    Eigen
    urdfdom
    urdfdom_headers
    console_bridge
    )

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS}
                           ${Boost_INCLUDE_DIR}
                           )

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} 
                    ${catkin_INCLUDE_DIRS} 
                    ${OCTOMAP_INCLUDE_DIRS}
                    )

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Generate and install version.h
string(REGEX REPLACE "^([0-9]+)\\..*" "\\1" MOVEIT_VERSION_MAJOR "${moveit_core_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1" MOVEIT_VERSION_MINOR "${moveit_core_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" MOVEIT_VERSION_PATCH "${moveit_core_VERSION}")
set(MOVEIT_VERSION_EXTRA "Beta")
set(MOVEIT_VERSION "${MOVEIT_VERSION_MAJOR}.${MOVEIT_VERSION_MINOR}.${MOVEIT_VERSION_PATCH}-${MOVEIT_VERSION_EXTRA}")
message(STATUS " *** Building MoveIt! ${MOVEIT_VERSION} ***")
configure_file("version/version.h.in" "${VERSION_FILE_PATH}/moveit/version.h")
install(FILES "${VERSION_FILE_PATH}/moveit/version.h" DESTINATION include/moveit)

# If the resources package is present, the tests can be built
set(BUILD_MOVEIT_TESTS FALSE)
find_package(moveit_resources QUIET)
if (${moveit_resources_FOUND})
  message(STATUS " *** Building MoveIt! Tests ***")
  include_directories(${moveit_resources_INCLUDE_DIRS})
  set(BUILD_MOVEIT_TESTS TRUE)
endif()

add_subdirectory(version)
add_subdirectory(macros)
add_subdirectory(exceptions)
add_subdirectory(profiler)
add_subdirectory(background_processing)
add_subdirectory(kinematics_base)
add_subdirectory(controller_manager)
add_subdirectory(sensor_manager)
add_subdirectory(robot_model)
add_subdirectory(transforms)
add_subdirectory(robot_state)
add_subdirectory(robot_trajectory)
add_subdirectory(collision_detection)
add_subdirectory(collision_detection_fcl)
add_subdirectory(kinematic_constraints)
add_subdirectory(planning_scene)
add_subdirectory(constraint_samplers)
add_subdirectory(planning_interface)
add_subdirectory(planning_request_adapter)
add_subdirectory(trajectory_processing)
add_subdirectory(distance_field)
add_subdirectory(kinematics_metrics)
add_subdirectory(dynamics_solver)
