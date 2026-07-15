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

#include "Dubins.hpp"
#include "Primitives.hpp"
#include "utils.hpp"

const uint NumberOfBaseDubins = 8;

/**
 * @brief Produces the path shapes making a path of given turn radius. Output has size 0 if unrealisable.
 * 
 * @param start         Start pose
 * @param end           End pose
 * @param turn_radius   Turn radius
 * @return std::vector<DynamicPathShape> 
 */
typedef std::vector<DynamicPathShape> (*BasePathShapeGenerator)(const Pose3D&, const Pose3D&, double, double);

// BasePathShapeGenerator for the 8 basic Dubins Paths 
std::vector<DynamicPathShape> set_radius_RSR(const Pose3D& start, const Pose3D& end, double turn_radius, double tol = DubinsFleetPlanner_PRECISION);
std::vector<DynamicPathShape> set_radius_LSL(const Pose3D& start, const Pose3D& end, double turn_radius, double tol = DubinsFleetPlanner_PRECISION);
std::vector<DynamicPathShape> set_radius_RSL(const Pose3D& start, const Pose3D& end, double turn_radius, double tol = DubinsFleetPlanner_PRECISION);
std::vector<DynamicPathShape> set_radius_LSR(const Pose3D& start, const Pose3D& end, double turn_radius, double tol = DubinsFleetPlanner_PRECISION);
std::vector<DynamicPathShape> set_radius_RLR(const Pose3D& start, const Pose3D& end, double turn_radius, double tol = DubinsFleetPlanner_PRECISION);
std::vector<DynamicPathShape> set_radius_LRL(const Pose3D& start, const Pose3D& end, double turn_radius, double tol = DubinsFleetPlanner_PRECISION);
std::vector<DynamicPathShape> set_radius_SRS(const Pose3D& start, const Pose3D& end, double turn_radius, double tol = DubinsFleetPlanner_PRECISION);
std::vector<DynamicPathShape> set_radius_SLS(const Pose3D& start, const Pose3D& end, double turn_radius, double tol = DubinsFleetPlanner_PRECISION);


/**
 * @brief Produces the path shapes making a path of given length. Output has size 0 if unrealisable.
 * 
 * @param start         Start pose
 * @param end           End pose
 * @param turn_radius   Minimal turn radius
 * @param target_len    Target length
 * @param tol           Fitted length tolerance
 * @return std::vector<DynamicPathShape> 
 */
typedef std::vector<DynamicPathShape> (*FittedPathShapeGenerator)(const Pose3D&, const Pose3D&, double, double, double);

// FittedPathShapeGenerator for the 8 basic Dubins Paths 
std::vector<DynamicPathShape> set_length_RSR(const Pose3D& start, const Pose3D& end, double turn_radius, double target_len, double tol);
std::vector<DynamicPathShape> set_length_LSL(const Pose3D& start, const Pose3D& end, double turn_radius, double target_len, double tol);
std::vector<DynamicPathShape> set_length_RSL(const Pose3D& start, const Pose3D& end, double turn_radius, double target_len, double tol);
std::vector<DynamicPathShape> set_length_LSR(const Pose3D& start, const Pose3D& end, double turn_radius, double target_len, double tol);
std::vector<DynamicPathShape> set_length_RLR(const Pose3D& start, const Pose3D& end, double turn_radius, double target_len, double tol);
std::vector<DynamicPathShape> set_length_LRL(const Pose3D& start, const Pose3D& end, double turn_radius, double target_len, double tol);
std::vector<DynamicPathShape> set_length_SRS(const Pose3D& start, const Pose3D& end, double turn_radius, double target_len, double tol);
std::vector<DynamicPathShape> set_length_SLS(const Pose3D& start, const Pose3D& end, double turn_radius, double target_len, double tol);


/**
 * @brief List all possible basic Dubins path (LSL,RSR,RSL,LSR,LRL,RLR,SLS,SRS) of with the given climb rate and turn radius
 * 
 * @param _climb        Climb rate at unit speed
 * @param _turn_radius  Turn radius for Dubins
 * @param _start        Starting point
 * @param _end          Ending point
 * @param wind_x        Wind vector, x axis (TODO: Not yet implemented)
 * @param wind_y        Wind vector, y axis (TODO: Not yet implemented)
 * @param tol           Precision used to declare a Dubins path well fit.
 * @return std::vector<std::unique_ptr<Dubins>> 
 */
std::vector<std::unique_ptr<Dubins>> list_possible_baseDubins(double _climb, double _turn_radius, const Pose3D& _start, const Pose3D& _end, double wind_x = 0., double wind_y = 0., double tol = DubinsFleetPlanner_PRECISION);

/**
 * @brief Provide the shortest Dubins path for the given configuration (TODO: Take wind into account)
 * 
 * @param _climb        Climb rate at unit speed
 * @param _turn_radius  Turn radius for Dubins
 * @param _start        Starting point
 * @param _end          Ending point
 * @param wind_x        Wind vector, x axis (TODO: Not yet implemented)
 * @param wind_y        Wind vector, y axis (TODO: Not yet implemented)
 * @param tol           Precision used to declare a Dubins path well fit.
 * @return std::unique_ptr<Dubins> 
 */
std::unique_ptr<Dubins> shortest_possible_baseDubins(double _climb, double _turn_radius, const Pose3D& _start, const Pose3D& _end, double wind_x = 0., double wind_y = 0., double tol = DubinsFleetPlanner_PRECISION);

/**
 * @brief List all possible basic Dubins path (LSL,RSR,RSL,LSR,LRL,RLR,SLS,SRS) of fitting the target length
 * 
 * TODO: Handle verticality
 * 
 * @param _climb        Climb rate at unit speed
 * @param _turn_radius  Minimal turn radius for Dubins
 * @param _start        Starting point
 * @param _end          Ending point
 * @param target_len    Target length
 * @param tol           Tolerance for target length solver
 * @return std::vector<Dubins> 
 */
std::vector<std::unique_ptr<Dubins>> fit_possible_baseDubins([[maybe_unused]] double _climb, double _turn_radius, const Pose3D& _start, const Pose3D& _end,
    double target_len, double tol);


