# Copyright (c) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

include(FetchContent)

find_package(Boost CONFIG REQUIRED COMPONENTS program_options chrono system container_hash)
set(Boost_DEBUG ON)


#################### General configuration ####################

if(DubinsFleetPlanner_ENABLE_ASSERTIONS)
    set(ASSERTIONS 1)
else()
    set(ASSERTIONS 0)
endif()

set(PRECISION 1e-3)
set(ZERO_TOLERANCE 1e-9)
set(SOLVE_WITH_DERIVATIVES 0)
set(DEBUG_MSG 1)

if(DubinsFleetPlanner_PARALLEL)
    set(PARALLEL 1)
endif()

configure_file(ProjectHeader.h.in ProjectHeader.h)

set(DubinsFleetPlannerLib_SRC
    FleetPlanner.cpp
    Aircraft.cpp
    BaseDubins.cpp
    ConflictDetection.cpp
    ConflictList.cpp
    Dubins.cpp
    Ellipse.cpp
    ExtendedDubins.cpp
    Fitting.cpp
    ioUtils.cpp
    PathGenerators/Extended.cpp
    PathGenerators/Sampling.cpp
    Primitives.cpp
    utils.cpp)

set(DubinsFleetPlannerLib_INCL
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${libs_dir}/eigen
    ${libs_dir}/ScalarMin
    ${libs_dir}/csv_reader)

if(DubinsFleetPlanner_PARALLEL)
    ### HiGHS Solver ###

    set(HIGHS_DIR ${libs_dir}/HiGHS/build)
    find_package(HIGHS REQUIRED)

    set(DubinsFleetPlannerLib_SRC
        ${DubinsFleetPlannerLib_SRC}
        ConflictList.cpp)

endif()

add_library(DubinsFleetPlannerLib SHARED ${DubinsFleetPlannerLib_SRC})

target_include_directories(DubinsFleetPlannerLib PUBLIC ${DubinsFleetPlannerLib_INCL})

if(DubinsFleetPlanner_PARALLEL)
    ### HiGHS Solver + BOOST ###

    target_link_libraries(DubinsFleetPlannerLib nlohmann_json::nlohmann_json Eigen3::Eigen highs::highs ${Boost_LIBRARIES})
else()
    ### BOOST only ###
    target_link_libraries(DubinsFleetPlannerLib nlohmann_json::nlohmann_json Eigen3::Eigen ${Boost_LIBRARIES})

endif()

# target_link_libraries(DubinsFleetPlanner PUBLIC Boost::headers ScalarMin)

add_executable(DubinsFleetPlanner main.cpp)

target_include_directories(DubinsFleetPlanner PUBLIC ${DubinsFleetPlannerLib_INCL} ${Boost_INCLUDE_DIRS})
target_link_libraries(DubinsFleetPlanner LINK_PUBLIC DubinsFleetPlannerLib ${Boost_LIBRARIES})
