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

#include <assert.h>
#include <limits>
#include <cmath>
#include <array>
#include <string>

#include "Pose3D.hpp"
#include "Aircraft.h"
#include "ProjectHeader.h"


/********************  Fundamental Dubins path computing  ********************/

/**
 * There are 6 fundamental Dubins paths, and 2 extras, described using three elementary moves among
 * right turns R, left turns L and straights S
 * 
 * Fundamental:
 * - LSL, RSR (always exist)
 * - RSL, LSR (long distance paths)
 * - RLR, LRL (short distance paths)
 * 
 * Extras:
 * - SRS, SLS ("aligned" poses, their rays must intersect)
 * 
 * This file implement the length computation for these paths pased on a 'normalized' problem, where the start
 * is at (0,0) oriented with angle `alpha` (radian), the end is at (d,0) oriented with angle `beta` 
 * and the turn radius is set to 1. 
 */

double LSL_first_distance(double alpha, double beta, double d);
double LSL_middle_distance(double alpha, double beta, double d);
double LSL_last_distance(double alpha, double beta, double d);

double LSL_total_distance(double alpha, double beta, double d);
std::pair<double,double> LSL_possible_d(double alpha, double beta);


double RSR_first_distance(double alpha, double beta, double d);
double RSR_middle_distance(double alpha, double beta, double d);
double RSR_last_distance(double alpha, double beta, double d);

double RSR_total_distance(double alpha, double beta, double d);
std::pair<double,double> RSR_possible_d(double alpha, double beta);


double RSL_first_distance(double alpha, double beta, double d);
double RSL_middle_distance(double alpha, double beta, double d);
double RSL_last_distance(double alpha, double beta, double d);

double RSL_total_distance(double alpha, double beta, double d);
std::pair<double,double> RSL_possible_d(double alpha, double beta);


double LSR_first_distance(double alpha, double beta, double d);
double LSR_middle_distance(double alpha, double beta, double d);
double LSR_last_distance(double alpha, double beta, double d);

double LSR_total_distance(double alpha, double beta, double d);
std::pair<double,double> LSR_possible_d(double alpha, double beta);


double RLR_first_distance(double alpha, double beta, double d);
double RLR_middle_distance(double alpha, double beta, double d);
double RLR_last_distance(double alpha, double beta, double d);

double RLR_total_distance(double alpha, double beta, double d);
std::pair<double,double> RLR_possible_d(double alpha, double beta);


double LRL_first_distance(double alpha, double beta, double d);
double LRL_middle_distance(double alpha, double beta, double d);
double LRL_last_distance(double alpha, double beta, double d);

double LRL_total_distance(double alpha, double beta, double d);
std::pair<double,double> LRL_possible_d(double alpha, double beta);


double SRS_first_distance(double alpha, double beta, double d);
double SRS_middle_distance(double alpha, double beta, double d);
double SRS_last_distance(double alpha, double beta, double d);

double SRS_total_distance(double alpha, double beta, double d);
std::pair<double,double> SRS_possible_d(double alpha, double beta);


double SLS_first_distance(double alpha, double beta, double d);
double SLS_middle_distance(double alpha, double beta, double d);
double SLS_last_distance(double alpha, double beta, double d);

double SLS_total_distance(double alpha, double beta, double d);
std::pair<double,double> SLS_possible_d(double alpha, double beta);


/********************  Extra: Dubins interception with wind  ********************/
/**
 * We aim to solve the shortest Dubins path to a final pose while taking in
 * account a constant drift. 
 * The paper [Rapid path planning for Dubins vehicles under environmental currents](https://doi.org/10.1016/j.robot.2020.103646)
 * shows how to get solutions analytically for LSL and RSR paths (allowing multiple turns).
 * These solutions can then be used as an upper bound for an optimisation-based solver (a  lower bound can be computed
 * by using straight line interception)
 */



/********************  General templates for Dubins functions  ********************/

/**
 * @brief The three possible moves for Dubins: Straight, Left turn and Right turn
 * 
 */
short enum DubinsMove {
    STRAIGHT = 0,
    LEFT     = 1,
    RIGHT    = 2
};

[[gnu::pure]]
constexpr bool is_existing_move(short m)
{
    return (m == STRAIGHT) || (m == LEFT) || (m == RIGHT);
}

const uint DubinsMoveNum = 3;

constexpr const std::array<std::string,3> DubinsMoveNames{
    std::string("STRAIGHT"),
    std::string("LEFT"),
    std::string("RIGHT")
};


inline constexpr const std::string get_DubinsMove_name(DubinsMove m)
{
    return DubinsMoveNames[m];
}

/**
 * @brief Follow a Dubins move for a given duration
 * 
 * @tparam m            Type of move (Straight, Left turn, Right turn)
 * @param pose          Starting pose (modified on place)
 * @param duration      Move duration (s)
 * @param speed         Vehicle XY speed (m/s)
 * @param climb_rate    Vehicle Z speed ([alt]/s)
 * @param turn_radius   Vehicle XY turn radius (m)
 */
template<DubinsMove m>
void update_dubins(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius);

/**
 * @brief Follow a Dubins move for a given duration
 * 
 * @tparam m            Type of move (Straight, Left turn, Right turn)
 * @param pose          Starting pose 
 * @param duration      Move duration (s)
 * @param speed         Vehicle XY speed (m/s)
 * @param climb_rate    Vehicle Z speed ([alt]/s)
 * @param turn_radius   Vehicle XY turn radius (m)
 * @return Pose3d       Resulting pose
 */
template<DubinsMove m>
Pose3D follow_dubins(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius);


/********************  Recover path shape from two points and a hint  ********************/

/**
 * @brief Structure to hold the parameters describing a shape, either a STRAIGHT or an arc circle (LEFT or RIGHT)
 * 
 * @tparam m DubinsMove describing the shape. Notethat LEFT and RIGHT can be handled similarly since the angular speed is signed
 * (LEFT has positive speed, RIGHT has negative speed)
 */
template<DubinsMove m>
struct PathShape
{
    double x,y,z;   ///< A reference point. For a Straight: the initial point | For a turn: the circle center
    double p1;  ///< For a Straight: horizontal x speed | For a turn: Radius
    double p2;  ///< For a Straight: horizontal y speed | For a turn: signed angular speed; positive when LEFT, negative when RIGHT
    double p3;  ///< Vertical speed
    double p4;  ///< For a Straight: unused, set to 0   | For a turn: initial angle
};


/**
 * @brief A relaxed version of PathShape, with the Dubins path type is stored as a dynamic variable
 * 
 */
class DynamicPathShape
{
public:
    double x,y,z;   ///< A reference point. For a Straight: the initial point | For a turn: the circle center
    double p1;      ///< For a Straight: horizontal x speed | For a turn: Radius
    double p2;      ///< For a Straight: horizontal y speed | For a turn: signed angular speed; positive when LEFT, negative when RIGHT
    double p3;      ///< Vertical speed
    double p4;      ///< For a Straight: unused, set to 0   | For a turn: initial angle
    DubinsMove m;   ///< Type of the path shape, as a variable
    double length = NAN;  ///< GROUND Length of the shape (if finite)


    // ----- Direct constructors ----- //
    constexpr DynamicPathShape() : x(NAN), y(NAN), z(NAN), p1(NAN), p2(NAN), p3(NAN), p4(NAN), m(STRAIGHT), length(NAN) {assert(is_existing_move(m));}

    constexpr DynamicPathShape(DubinsMove _m, double _x, double _y, double _z, double _p1, double _p2, double _p3, double _p4, double _length = NAN) :
    x(_x), y(_y), z(_z), p1(_p1), p2(_p2), p3(_p3), p4(_p4), m(_m), length(_length) {assert(is_existing_move(_m));}

    template<DubinsMove _m>
    constexpr DynamicPathShape(const PathShape<_m>& s) : 
        x(s._x), y(s._y), z(s._z), p1(s._p1), p2(s._p2), p3(s._p3), p4(s._p4), m(_m), length(NAN) {assert(is_existing_move(_m));}

    template<DubinsMove _m>
    DynamicPathShape(const PathShape<_m>& s, double duration) : 
        x(s._x), y(s._y), z(s._z), p1(s._p1), p2(s._p2), p3(s._p3), p4(s._p4), m(_m), length(path_planar_speed(s)*duration) {assert(is_existing_move(_m));}

    // ----- From endpoints constructor ----- //
    DynamicPathShape(DubinsMove _m, const Pose3D& start, const Pose3D& end, double h_speed, double turn_radius, double v_speed);

    // ----- From start and duration ----- //
    DynamicPathShape(DubinsMove m, const Pose3D& start, double duration, double h_speed, double turn_radius, double v_speed);

    /**
     * @brief Compute path duration (only makes sense if planar speed is nonzero and length well defined)
     * 
     * @return double 
     */
    double duration() const
    {
        return length/planar_speed();
    }

    /**
     * @brief Return the speed magnitude in the XY plane
     * 
     * @return double 
     */
    double planar_speed() const
    {
        if (this->m == STRAIGHT)
        {
            return std::sqrt(this->p1*this->p1 + this->p2*this->p2);
        }
        else
        {
            return std::abs(this->p2*this->p1);
        }
    }

    /**
     * @brief Set the XY speed
     * 
     * @param speed 
     */
    void set_planar_speed(double speed)
    {
        if (this->m == STRAIGHT)
        {
            double curr_speed = this->planar_speed();
            this->p1 *= speed/curr_speed;
            this->p2 *= speed/curr_speed;
        }
        else
        {
            this->p2 = ((this->m == LEFT) ? 1. : -1.) * speed/this->p1;
        }
    }

    /**
     * @brief Return the achieved pose after following the shape for the given duration
     * Note: There is no check with the existing length parameter
     * 
     * @param duration 
     * @return Pose3D 
     */
    Pose3D follow(double duration) const;

    /**
     * @brief Return the initial pose associated to this shape
     * 
     * @return Pose3D 
     */
    Pose3D initial_pose() const;

    /**
     * @brief Return the final pose associated to this shape (which is well-defined only if the length attribute is)
     * 
     * @return Pose3D 
     */
    Pose3D final_pose() const
    {
        assert(std::isfinite(length));
        return follow(duration());
    }

    /**
     * @brief Copy this object into its statically type variant, PathShape
     * 
     * @tparam _m 
     * @return PathShape<_m> 
     */
    template<DubinsMove _m>
    PathShape<_m> to_PathShape() const
    {
        assert(m == _m);
        PathShape<_m> output(
            x,y,z,
            p1,p2,p3,p4
        );
        return output;
    }

    /**
     * @brief Follow the path for the given duration then set the reached pose as the new starting point
     * 
     * @param duration 
     * @param reduce_length If true, deduce the travelled length for the existing attribute. Default to false.
     */
    void shift_start(double duration, bool reduce_length=false);

    /**
     * @brief Return false if it is a STRAIGHT, true otherwise
     * 
     * @return true 
     * @return false 
     */
    bool is_turning() const
    {
        return (m != STRAIGHT);
    }

    /**
     * @brief Return by how much the initial angle is changed (in absolute value) while following the path for a given duration
     * 
     * @param dt 
     * @return double 
     */
    double turning_angle(double dt) const
    {
        if (!is_turning())
        {
            return 0.;
        }
        else
        {
            return abs(p2*dt);
        }
    }

    /**
     * @brief Return by how much the initial angle is changed (in absolute value) while following the whole path (assume length is well defined)
     * 
     * @return double 
     */
    double turning_angle() const
    {
        return turning_angle(duration());
    }

    std::vector<std::tuple<double,Pose3D>> get_sampled_poses(double dl) const
    {
        std::vector<std::tuple<double,Pose3D>> output;
        double now = 0.;
        double l = 0.;
        double dt = dl/planar_speed();
        while (now < duration())
        {
            output.push_back(std::make_tuple(l, follow(now)));
            now += dt;
            l += dl;
        }
        output.push_back(std::make_tuple(length, follow(duration())));
        return output;
    }

    std::vector<std::tuple<double,Pose3D>> get_sampled_poses(double dt, double duration) const
    {
        std::vector<std::tuple<double,Pose3D>> output;
        double now = 0.;
        while (now < duration)
        {
            output.push_back(std::make_tuple(now, follow(now)));
            now += dt;
        }
        output.push_back(std::make_tuple(duration, follow(duration)));
        return output;
    }
};

template<DubinsMove m>
PathShape<m> to_static_shape(const DynamicPathShape& s)
{
    return s.to_PathShape<m>();
}


/**
 * @brief Get the XY speed absolute value for the given structure 
 * 
 * @tparam m Structure type (either STRAIGHT or a turn, LEFT or RIGHT)
 * @param s Path structure
 * @return double XY absolute speed
 */
template<DubinsMove m>
inline double path_planar_speed(const PathShape<m>& s);

template<>
inline double path_planar_speed(const PathShape<STRAIGHT>& s)
{
    return std::sqrt(s.p1*s.p1+s.p2*s.p2);
}

template<DubinsMove m>
inline double path_planar_speed(const PathShape<m>& s)
{
    return std::abs(s.p2*s.p1);
}


/**
 * @brief Set the speed for the path. Warning! If the speed is negative, the path should be followed backward 
 * 
 * @tparam m Structure type (either STRAIGHT or a turn, LEFT or RIGHT)
 * @param s Path structure
 * @param speed XY Speed value
 */
template<DubinsMove m>
inline void path_set_planar_speed(PathShape<m>& s, double speed);

template<>
inline void path_set_planar_speed(PathShape<STRAIGHT>& s, double speed)
{
    double curr_speed = path_planar_speed(s);
    s.p1 *= speed/curr_speed;
    s.p2 *= speed/curr_speed;
}

template<DubinsMove m>
inline void path_set_planar_speed(PathShape<m>& s, double speed)
{
    s.p2 = ((m == LEFT) ? 1. : -1.) * speed/s.p1;
}

/**
 * @brief Compute the initial XY direction of the given shape
 * 
 * @tparam m Structure type (either STRAIGHT or a turn, LEFT or RIGHT)
 * @param s Path structure
 * @return double Initial direction, in radian
 */
template<DubinsMove m>
inline double path_initial_direction(const PathShape<m>& s);

template<>
inline double path_initial_direction(const PathShape<STRAIGHT>& s)
{
    return std::atan2(s.p2,s.p1);
}

template<DubinsMove m>
inline double path_initial_direction(const PathShape<m>& s)
{
    return s.p4 + M_PI_2*((m==LEFT) ? (1) : (-1));
}

/**
 * @brief Given a shape and two points on this shape, compute the parameters for its parametric equation
 * 
 * @tparam m Shape parameter, being on the Dubins move
 * @param start     First pose sampled
 * @param end       Second pose sampled
 * @param h_speed   XY vehicle speed
 * @param turn_radius   Turn radius
 * @param v_speed   Z vehicle speed
 * @return PathShape    Computed parameters
 */
template<DubinsMove m>
PathShape<m> compute_params(const Pose3D& start, const Pose3D& end, double h_speed, double turn_radius, double v_speed);


template<DubinsMove m>
PathShape<m> compute_params(const Pose3D& start, const Pose3D& end, const AircraftStats& stats)
{
    return compute_params<m>(start,end,stats.airspeed,stats.turn_radius,stats.climb);
}

/**
 * @brief Conversion from PathShape to DynamicPathShape (associated DubinsMove becomes dynamically typed)
 * 
 * @tparam m DubinsMove type
 * @param p Source PathShape
 * @return DynamicPathShape, equivalent to `p` 
 */
template<DubinsMove m>
DynamicPathShape to_dynamic_PathShape(const PathShape<m>& p)
{
    return DynamicPathShape(
        m,
        p.x,
        p.y,
        p.z,
        p.p1,
        p.p2,
        p.p3,
        p.p4
    );
}

/**
 * @brief Follow a Dubins move for a given duration (adapted for PathShape shorthand)
 * 
 * @tparam m        Type of move (Straight, Left turn, Right turn)
 * @param s         Structure describing the shape to follow (including speed and starting point)
 * @param duration  Move duration (s)
 * @return Pose3d   Resulting pose
 */
template<DubinsMove m>
[[gnu::pure]]
Pose3D follow_dubins(const PathShape<m>& s, double duration);

/**
 * @brief Follow a Dubins move for a given duration. Equivalent to DynamicPathShape::follow
 * 
 * @param s         Class describing the shape to follow (including speed and starting point)
 * @param duration  Move duration (s)
 * @return Pose3d   Resulting pose
 */
Pose3D follow_dubins(const DynamicPathShape& s, double duration);

/**
 * @brief Compute the initial pose for a PathShape
 * 
 * @tparam m        Type of move (Straight, Left turn, Right turn)
 * @param s         Structure describing the shape to follow (including speed and starting point)
 * @return Pose3d   Starting pose
 */
template<DubinsMove m>
[[gnu::pure]]
Pose3D initial_pose(const PathShape<m>& s)
{
    return follow_dubins(s,0.);
}

/**
 * @brief Compute an equivalent path shape by shifting the starting point to the location after the given duration
 * 
 * @tparam m        Type of move (Straight, Left turn, Right turn)
 * @param s         Structure describing the shape to follow (including speed and starting point)
 * @param duration  Duration of path following shifting the start
 * @return          New structure with shifted start
 */
template<DubinsMove m>
[[gnu::pure]]
PathShape<m> shift_start(const PathShape<m>& s, double duration);

DynamicPathShape shift_start(const DynamicPathShape& s, double duration);
