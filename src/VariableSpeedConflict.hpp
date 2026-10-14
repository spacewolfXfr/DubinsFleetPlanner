#pragma once

#include <tuple>

#include "utils.hpp"
#include "Dubins.hpp"
#include "Primitives.hpp"
#include "ProjectHeader.h"

/**
 * @brief Given two base trajectories, find the minimal amount of speed-based slack between the two.
 * 
 * This amount to the minimal planar euclidean distance between the two trajectories, *ignoring* the vertical component.
 * 
 * @tparam use_derivatives Set if the solver should use the derivatives for computations of not (may improve speed)
 * @param s1 First trajectory
 * @param s2 Second trajectory
 * @param s1_stats Motion statistics for the first trajectory
 * @param s2_stats Motion statistics for the second trajectory
 * @param duration Duration (in [time]) of the trajectories
 * @param tdist Time distance required between the two trajectories (an actual distance is computed by multiplying by the current highest speed)
 * @param tol Solver tolerance when looking for the minimum distance
 * @return std::pair<double,double> Location and value of the minimal 2D slack (may be negative)
 */
template<bool use_derivatives=DubinsFleetPlanner_SOLVE_WITH_DERIVATIVES>
std::pair<double,double> temporal_XY_time_slack(const DynamicPathShape &s1, const DynamicPathShape &s2,
    const AircraftStats& s1_stats, const AircraftStats& s2_stats, double duration, double tdist, double tol);
