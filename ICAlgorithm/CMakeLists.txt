cmake_minimum_required(VERSION 3.16)
project(hic_controller LANGUAGES C CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fvisibility=hidden -fvisibility-inlines-hidden")

option(HIC_ENABLE_DEBUG_PRINT "Enable throttled debug prints in HIC C API hot paths" ON)

set(HIC_EIGEN_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen3)
set(HIC_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/hic_controller/src/common)

add_library(hic_controller SHARED
    hic_controller/src/hic_c_api.cpp
    hic_controller/src/coordinator/hic_control_coordinator.cpp
    hic_controller/src/kinematics/hic_kinematics_adapter.cpp
    hic_controller/src/kinematics/kinematicsBase.cpp
    hic_controller/src/kinematics/DSKinematics.cpp
    hic_controller/src/kinematics/URKinematics.cpp
    hic_controller/src/kinematics/elfinKinematics.cpp
    hic_controller/src/kinematics/palletKinematics.cpp
    hic_controller/src/dynamics/hic_dynamics_adapter.cpp
    hic_controller/src/dynamics/dynamicsBase.cpp
    hic_controller/src/dynamics/anthorDynamics.cpp
    hic_controller/src/dynamics/dsDynamics.cpp
    hic_controller/src/dynamics/elfinDynamics.cpp
    hic_controller/src/dynamics/palletDynamics.cpp
    hic_controller/src/dynamics/sevendofDynamics.cpp
    hic_controller/src/dynamics/urDynamics.cpp
    hic_controller/src/control/hic_cartesian_impedance_core.cpp
    hic_controller/src/control/hic_joint_impedance_core.cpp
    hic_controller/src/state/hic_robot_state_observer.cpp
    hic_controller/src/sensing/hic_interaction_force_observer.cpp
    hic_controller/src/common/hic_frames.cpp
    hic_controller/src/common/hic_kdl_utility.cpp
)

target_compile_definitions(hic_controller PRIVATE HIC_BUILD_SHARED)

if(HIC_ENABLE_DEBUG_PRINT)
    target_compile_definitions(hic_controller PRIVATE HIC_ENABLE_DEBUG_PRINT)
endif()

target_include_directories(hic_controller PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/hic_controller/include
    ${HIC_COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/hic_controller/src/dynamics
    ${CMAKE_CURRENT_SOURCE_DIR}/hic_controller/src/kinematics
    ${CMAKE_CURRENT_SOURCE_DIR}/external/Components
    ${CMAKE_CURRENT_SOURCE_DIR}/external/Platforms/Linux
    ${HIC_EIGEN_INCLUDE_DIRS}
)

add_executable(hic_impedance_demo
    hic_controller/test/hic_impedance_demo.cpp
)

target_link_libraries(hic_impedance_demo hic_controller)

target_include_directories(hic_impedance_demo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/hic_controller/include
)

add_executable(hic_state_generator
    hic_controller/test/hic_state_generator.cpp
)

target_include_directories(hic_state_generator PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/hic_controller/include
)

add_executable(hic_controller_cli
    hic_controller/test/hic_controller_cli.cpp
)

target_link_libraries(hic_controller_cli hic_controller)

target_include_directories(hic_controller_cli PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/hic_controller/include
)
