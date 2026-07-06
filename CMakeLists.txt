# Copyright (C) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This file is part of DubinsFleetPlanner.
# 
# DubinsFleetPlanner is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# DubinsFleetPlanner is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with DubinsFleetPlanner.  If not, see <https://www.gnu.org/licenses/>.

# CMake general configuration
cmake_minimum_required(VERSION 3.28.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_DEBUG_POSTFIX d)
set(CMAKE_POLICY_VERSION_MINIMUM 3.28)

set(CMAKE_VERBOSE_MAKEFILE ON)

include(FetchContent)

# -------------------- Project declaration -------------------- #
project(DubinsFleetPlanner VERSION 0.1.0 LANGUAGES C CXX)

# -------------------- Compilers -------------------- #

set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")

# -------------------- Required packages -------------------- #
### Boost ###

set(Boost_USE_STATIC_LIBS        ON)  # only find static libs
set(Boost_USE_DEBUG_LIBS        OFF)  # ignore debug libs and
set(Boost_USE_RELEASE_LIBS       ON)  # only find release libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     ON)
find_package(Boost 1.89 CONFIG REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

### Eigen ###

set(EIGEN_BUILD_DEMOS OFF)
set(EIGEN_BUILD_DOC OFF)
set(EIGEN_BUILD_SPBENCH OFF)
set(EIGEN_BUILD_TESTING OFF)
set(EIGEN_LEAVE_TEST_IN_ALL_TARGET OFF)

### JSON ###

option(DubinsFleetPlanner_USE_EXTERNAL_JSON "Use an external JSON library (if none found, will download using FetchContent)" OFF)

### Add lib subdirectory ###

add_subdirectory(libs)

set(libs_dir "${PROJECT_SOURCE_DIR}/libs")

# -------------------- Options -------------------- #

option(DubinsFleetPlanner_PARALLEL "Enable parallel solver (Requires HiGHS Library)" ON)

option(DubinsFleetPlanner_ENABLE_ASSERTIONS "Enable assertion checks" ON)

option(DubinsFleetPlanner_TESTING "Enable testing for path planning (will download GTest)" ON)

# -------------------- Targets -------------------- #

add_subdirectory(src)


option(DubinsFleetPlanner_COVERAGE "Perform coverage analysis while running tests" OFF)

# -------------------- Testing -------------------- #

if(DubinsFleetPlanner_TESTING)
    enable_testing()
    add_subdirectory(test)
endif()
