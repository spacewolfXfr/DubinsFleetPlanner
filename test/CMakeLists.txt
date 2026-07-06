# Copyright (C) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This file is part of DubinsFleetPlannerLib.
# 
# DubinsFleetPlannerLib is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# DubinsFleetPlannerLib is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with DubinsFleetPlannerLib.  If not, see <https://www.gnu.org/licenses/>.


#################### Test folder ####################

include(FetchContent)

########## Visual tests ##########

option(DubinsFleetPlanner_VISUALISATION "Include visualisation in the solver (for debug/demo purpose; will download Sciplot)" ON)

# if(DubinsFleetPlanner_VISUALISATON)
    FetchContent_Declare(
        sciplot
        GIT_REPOSITORY https://github.com/sciplot/sciplot.git
        GIT_TAG        f8d779a # release-0.3.0
        )

    set(SCIPLOT_BUILD_DOCS OFF)
    set(SCIPLOT_BUILD_EXAMPLES OFF)
    set(SCIPLOT_BUILD_TESTS OFF)
    
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    FetchContent_MakeAvailable(sciplot)

    add_executable(TestHighs call_highs_from_cpp.cpp)
    target_link_libraries(TestHighs PUBLIC sciplot DubinsFleetPlannerLib)
    target_include_directories(TestHighs PUBLIC "${PROJECT_SOURCE_DIR}/src")

    add_executable(PlotDubins plotDubins.cpp)
    target_link_libraries(PlotDubins PUBLIC sciplot DubinsFleetPlannerLib)
    target_include_directories(PlotDubins PUBLIC "${PROJECT_SOURCE_DIR}/src")

    add_executable(PlotGeometries plotGeometries.cpp)
    target_link_libraries(PlotGeometries PUBLIC sciplot DubinsFleetPlannerLib)
    target_include_directories(PlotGeometries PUBLIC "${PROJECT_SOURCE_DIR}/src")

    add_executable(PlotDubinsMoveDist plotDubinsMoveDist.cpp)
    target_link_libraries(PlotDubinsMoveDist PUBLIC sciplot DubinsFleetPlannerLib)
    target_include_directories(PlotDubinsMoveDist PUBLIC "${PROJECT_SOURCE_DIR}/src")

    add_executable(PlotDubinsWithGeometries plotDubinsWithGeometries.cpp)
    target_link_libraries(PlotDubinsWithGeometries PUBLIC sciplot DubinsFleetPlannerLib)
    target_include_directories(PlotDubinsWithGeometries PUBLIC "${PROJECT_SOURCE_DIR}/src")

    add_executable(PlotDubinsDist plotDubinsDist.cpp)
    target_link_libraries(PlotDubinsDist PUBLIC sciplot DubinsFleetPlannerLib)
    target_include_directories(PlotDubinsDist PUBLIC "${PROJECT_SOURCE_DIR}/src")

    add_executable(PlotBasicDubinsFleet plotBasicDubinsFleet.cpp)
    target_link_libraries(PlotBasicDubinsFleet PUBLIC sciplot DubinsFleetPlannerLib)
    target_include_directories(PlotBasicDubinsFleet PUBLIC "${PROJECT_SOURCE_DIR}/src")

    add_executable(SequenceDubinsFleet sequenceDubinsFleet.cpp)
    target_link_libraries(SequenceDubinsFleet PUBLIC sciplot DubinsFleetPlannerLib)
    target_include_directories(SequenceDubinsFleet PUBLIC "${PROJECT_SOURCE_DIR}/src")

    add_executable(ParseAndPlot parseAndPlot.cpp)
    target_link_libraries(ParseAndPlot PUBLIC sciplot DubinsFleetPlannerLib ${Boost_LIBRARIES})
    target_include_directories(ParseAndPlot PUBLIC "${PROJECT_SOURCE_DIR}/src" ${Boost_LIBRARIES})
# endif()

########## Coverage ##########

if (DubinsFleetPlanner_COVERAGE) # Check if the build type is 'Coverage'
  find_package(Python3 REQUIRED COMPONENTS Interpreter)  # Required for gcovr
  add_custom_target(coverage
    COMMAND ${CMAKE_COMMAND} -E make_directory coverage
    COMMAND ${Python3_EXECUTABLE} -m gcovr -r ${CMAKE_SOURCE_DIR} --html --html-nested -o coverage/index.html
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating HTML coverage report..."
  )
endif()

########## Procedural tests ##########

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        52eb8108c5bdec04579160ae17225d66034bd723 # release-1.17.0
    )

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

macro(make_DubinsPathPlanning_test testName testFile)
    add_executable(${testName} ${testFile})
    target_link_libraries(${testName} PUBLIC DubinsFleetPlannerLib GTest::gtest_main)

    target_compile_options(${testName} PUBLIC
        "$<${gcc_like_cxx}:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused;-Wsuggest-attribute=pure;-g;-Og>"
        "$<${msvc_cxx}:-W3>"
        )

    target_compile_options(${testName} PRIVATE --coverage)
    target_link_options(${testName} PRIVATE --coverage)

    target_include_directories(${testName} PUBLIC 
                            "${PROJECT_SOURCE_DIR}"
                            "${PROJECT_SOURCE_DIR}/src"
                            "${PROJECT_SOURCE_DIR}/libs"
                            )

    gtest_discover_tests(${testName})
endmacro()

make_DubinsPathPlanning_test(TestGeometricDistance testGeometricDistance.cpp)
make_DubinsPathPlanning_test(TestTemporalDistance testTemporalDistance.cpp)
make_DubinsPathPlanning_test(TestDubinsSeparation testDubinsSeparation.cpp)




