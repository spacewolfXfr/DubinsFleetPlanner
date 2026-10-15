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

#include <tuple>

#include "utils.hpp"
#include "Dubins.hpp"
#include "Primitives.hpp"
#include "ProjectHeader.h"
#include "SpeedProfile.hpp"


#ifndef DubinsFleetPlanner_PRECISION
#define DubinsFleetPlanner_PRECISION 1e-6
#endif


// ==================== Geometric distance ==================== //


/**
 * @brief Given two shapes described by paths, compute their XY geometric separation 
 * 
 * This amount to the minimal euclidean distance between the two shapes, *ignoring* the vertical component.
 * 
 * @tparam m1 First shape type (STRAIGHT or a turn, RIGHT or LEFT)
 * @tparam m2 Second shape type (STRAIGHT or a turn, RIGHT or LEFT)
 * @param s1,s2 Shapes parameters
 * @param d1,d2 Durations (in s) for which the shapes are followed, defining segment and circle arcs
 * @param duration Duration (in s) for which the shape are followed, defining segment and circle arcs
 * @return double The minimal XY separation distance, with its location parameter with respect to the first and second paths
 */
template<DubinsMove m1, DubinsMove m2>
std::tuple<double,double,double> geometric_XY_dist(const PathShape<m1>& s1, const PathShape<m2>& s2, double d1, double d2);

/**
 * @brief Specialization of its 4 arguments version such that the two durations are equal
 */
template<DubinsMove m1, DubinsMove m2>
std::tuple<double,double,double> geometric_XY_dist(const PathShape<m1>& s1, const PathShape<m2>& s2, double duration)
{
    return geometric_XY_dist<m1,m2>(s1,s2,duration,duration);
}

/**
 * @brief Given two shapes described by paths, compute their XY geometric separation 
 * 
 * This amount to the minimal euclidean distance between the two shapes, *ignoring* the vertical component.
 * 
 * @param s1,s2 Shapes
 * @param d1,d2 Durations (in s) for which the shapes are followed, defining segment and circle arcs
 * @return std::tuple<double,double,double> The minimal XY separation distance, with its location parameter with respect to the first and second paths
 */
std::tuple<double,double,double> geometric_XY_dist(const DynamicPathShape& s1, const DynamicPathShape& s2, double d1, double d2);

/**
 * @brief Specialization of its 4 arguments version such that the two durations are equal
 */
inline std::tuple<double,double,double> geometric_XY_dist(const DynamicPathShape& s1, const DynamicPathShape& s2, double duration)
{
    return geometric_XY_dist(s1,s2,duration,duration);
}

/**
 * @brief Compute the XY geometric distance between a path and a point
 * 
 * @param s Obstacle path
 * @param p Point (ignore vertical and orientation components)
 * @param d Duration (in s) for which the path is followed
 * @return std::tuple<double,double> The minimal XY separation distance, with its location parameter with respect to the path 
 */
std::tuple<double,double> geometric_XY_point_dist(const DynamicPathShape& s, const Pose3D& p, double d);

/**
 * @brief Given two shapes described by paths, compute their XY geometric separation by sampling
 * 
 * This amount to the minimal euclidean distance between the two shapes, *ignoring* the vertical component.
 * 
 * @tparam m1 First shape type (STRAIGHT or a turn, RIGHT or LEFT)
 * @tparam m2 Second shape type (STRAIGHT or a turn, RIGHT or LEFT)
 * @tparam samples Number of samples for computing the minimal distance
 * @param s1,s2 Shapes parameters
 * @param d1,d2 Durations (in s) for which the shapes are followed, defining segment and circle arcs
 * @return double The minimal XY separation distance
 */
template<DubinsMove m1, DubinsMove m2, uint samples>
double sampled_geometric_XY_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double d1, double d2)
{
    static_assert(samples > 1);

    std::array<Pose3D,samples> s1_samples,s2_samples;
    double min_dist = INFINITY;

    for(uint i = 0; i < samples; i++)
    {
        s1_samples[i] = follow_dubins(s1,i*d1/(samples-1));
        s2_samples[i] = follow_dubins(s2,i*d2/(samples-1));
    }

    for(uint i = 0; i <samples; i++)
    {
        for(uint j = 0; j < samples; j++)
        {
            min_dist = std::min(min_dist,pose_dist_XY(s1_samples[i],s2_samples[j]));
        }
    }

    return min_dist;
}

/**
 * @brief Specialization of the 4 arguments version, where the two durations are equal
 */
template<DubinsMove m1, DubinsMove m2, uint samples>
double sampled_geometric_XY_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration)
{
    return sampled_geometric_XY_dist<m1,m2,samples>(s1,s2,duration,duration);
}

/**
 * @brief Given two shapes described by paths, compute their vertical (Z) geometric separation
 * 
 * Since all shapes consider linear climbs, the exact type does not change a thing to the computations
 * 
 * @tparam m1 First shape type (STRAIGHT or a turn, RIGHT or LEFT)
 * @tparam m2 Second shape type (STRAIGHT or a turn, RIGHT or LEFT)
 * @param s1 First shape parameters
 * @param s2 Second shape parameters
 * @param duration Duration (in s) for which the shape are followed, defining the segment
 * @return double The minimal Z separation distance
 */
template<DubinsMove m1, DubinsMove m2>
double geometric_Z_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration);



// ==================== Temporal distance ==================== //

/**
 * @brief Given two base trajectories, find the minimal 3D euclidean distance between them on the given time interval
 * 
 * @tparam m1 First trajectory type (STRAIGHT or a turn, RIGHT or LEFT)
 * @tparam m2 Second trajectory type (STRAIGHT or a turn, RIGHT or LEFT)
 * @param s1 First trajectory parameters
 * @param s2 Second trajectory parameters
 * @param duration Duration (in s) of the trajectories
 * @param tol Solver tolerance when looking for the minimum distance
 * @return std::pair<double,double> Location and value of the minimal euclidean distance in the given duration
 */
template<DubinsMove m1, DubinsMove m2>
std::pair<double,double> temporal_3D_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration, double tol);

/**
 * @brief Given two base trajectories, find the minimal XY euclidean distance between them on the given time interval
 * using a global minimum solver.
 * 
 * This amount to the minimal planar euclidean distance between the two trajectories, *ignoring* the vertical component.
 * 
 * @tparam m1 First trajectory type (STRAIGHT or a turn, RIGHT or LEFT)
 * @tparam m2 Second trajectory type (STRAIGHT or a turn, RIGHT or LEFT)
 * @tparam use_derivatives Set if the solver should use the derivatives for computations of not (may improve speed)
 * @param s1 First trajectory parameters
 * @param s2 Second trajectory parameters
 * @param s1_stats Motion statistics for the first trajectory
 * @param s2_stats Motion statistics for the second trajectory
 * @param duration Duration (in s) of the trajectories
 * @param tol Solver tolerance when looking for the minimum distance
 * @return std::pair<double,double> Location and value of the minimal 2D euclidean distance in the given duration
 */
template<DubinsMove m1, DubinsMove m2, bool use_derivatives=DubinsFleetPlanner_SOLVE_WITH_DERIVATIVES>
std::pair<double,double> temporal_XY_dist(const PathShape<m1> &s1, const PathShape<m2> &s2,
    const SpeedProfile& s1_prof, const SpeedProfile& s2_prof, double duration, double tol);

/**
 * @brief Given two base trajectories, find the minimal XY euclidean distance between them on the given time interval
 * using a global minimum solver.
 * 
 * This amount to the minimal planar euclidean distance between the two trajectories, *ignoring* the vertical component.
 * 
 * @tparam use_derivatives Set if the solver should use the derivatives for computations of not (may improve speed)
 * @param s1 First trajectory
 * @param s2 Second trajectory
 * @param s1_stats Motion statistics for the first trajectory
 * @param s2_stats Motion statistics for the second trajectory
 * @param duration Duration (in s) of the trajectories
 * @param tol Solver tolerance when looking for the minimum distance
 * @return std::pair<double,double> Location and value of the minimal 2D euclidean distance in the given duration
 */
template<bool use_derivatives=DubinsFleetPlanner_SOLVE_WITH_DERIVATIVES>
std::pair<double,double> temporal_XY_dist(const DynamicPathShape &s1, const DynamicPathShape &s2,
    const SpeedProfile& s1_prof, const SpeedProfile& s2_prof, double duration, double tol);


/**
 * @brief Given two base trajectories, find the minimal Z euclidean distance between them on the given time interval
 * 
 * Since all trajectories use linear climbs, the exact type does not change a thing to the computations
 *  
 * @tparam m1 First trajectory type (STRAIGHT or a turn, RIGHT or LEFT)
 * @tparam m2 Second trajectory type (STRAIGHT or a turn, RIGHT or LEFT)
 * @param s1 First trajectory parameters
 * @param s2 Second trajectory parameters
 * @param duration Duration (in s) of the trajectories
 * @param tol Solver tolerance when looking for the minimum distance
 * @return std::pair<double,double> Location and value of the minimal vertical distance in the given duration
 */
template<DubinsMove m1, DubinsMove m2>
std::pair<double,double> temporal_Z_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration, double tol);
