// Copyright (C) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
// 
// This file is part of DubinsFleetPlanner.
// 
// DubinsFleetPlanner is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// DubinsFleetPlanner is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with DubinsFleetPlanner.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <Eigen/Dense>
#include <random>
#include <tuple>

#include "ConflictDetection.hpp"
#include "randomPathShape.hpp"


#ifndef TEST_NUM
#define TEST_NUM 50
#endif

#ifndef TEST_POS_RANGE
#define TEST_POS_RANGE 5
#endif

#ifndef TEST_SPEED_RANGE
#define TEST_SPEED_RANGE 5
#endif

#ifndef TEST_PATH_DURATION
#define TEST_PATH_DURATION 10
#endif

#ifndef TEST_PRECISION
#define TEST_PRECISION DubinsFleetPlanner_PRECISION
#endif

