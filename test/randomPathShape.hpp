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
#include <random>

#include <Eigen/Dense>
#include "Aircraft.h"
#include "Primitives.hpp"

template<DubinsMove m>
PathShape<m> generate_random_shape(int seed, double range, double max_speed)
{
    std::default_random_engine gen(seed); // Some seeded RNG 

    range = std::abs(range);
    max_speed = std::abs(max_speed);

    std::uniform_real_distribution<double> dis_pos(-range, range);
    std::uniform_real_distribution<double> dis_speedn(max_speed/10, max_speed);
    std::uniform_real_distribution<double> dis_angle(-M_PI,M_PI);

    PathShape<m> output;
    output.x = dis_pos(gen);
    output.y = dis_pos(gen);
    output.z = dis_pos(gen);

    output.p3 = dis_speedn(gen)/10 * (dis_pos(gen) > 0 ? 1 : -1);

    double speed_n = dis_speedn(gen);

    double angle = dis_angle(gen);

    if (m == STRAIGHT)
    {
        output.p1 = speed_n * std::cos(angle);
        output.p2 = speed_n * std::sin(angle);
    }
    else
    {
        output.p1 = dis_speedn(gen);
        output.p2 = speed_n/output.p1 * ((m == LEFT) ? 1 : -1);
        output.p4 = angle;
    }

    return output;
}

DynamicPathShape generate_random_dynamic_shape(DubinsMove m, int seed, double range, double max_speed)
{
    std::default_random_engine gen(seed); // Some seeded RNG 

    range = std::abs(range);
    max_speed = std::abs(max_speed);

    std::uniform_real_distribution<double> dis_pos(-range, range);
    std::uniform_real_distribution<double> dis_speedn(max_speed/10, max_speed);
    std::uniform_real_distribution<double> dis_angle(-M_PI,M_PI);

    DynamicPathShape output;

    output.m = m;
    output.x = dis_pos(gen);
    output.y = dis_pos(gen);
    output.z = dis_pos(gen);

    output.p3 = dis_speedn(gen)/10 * (dis_pos(gen) > 0 ? 1 : -1);

    double speed_n = dis_speedn(gen);

    double angle = dis_angle(gen);

    if (output.m == STRAIGHT)
    {
        output.p1 = speed_n * std::cos(angle);
        output.p2 = speed_n * std::sin(angle);
    }
    else
    {
        output.p1 = dis_speedn(gen);
        output.p2 = speed_n/output.p1 * ((m == LEFT) ? 1 : -1);
        output.p4 = angle;
    }

    return output;
}

AircraftStats generate_random_aircraft_stats(int seed, double min_radius, double max_radius, double min_speed, double max_speed, double max_acc)
{
    std::default_random_engine gen(seed); // Some seeded RNG 

    std::uniform_real_distribution<double> dis_turn_radius(min_radius, max_radius);
    std::uniform_real_distribution<double> dis_speedn(min_speed, max_speed);

    AircraftStats output;

    output.id = seed;
    output.airspeed = dis_speedn(gen);
    output.target_airspeed = dis_speedn(gen);
    output.turn_radius = dis_turn_radius(gen);
    output.climb = dis_speedn(gen)/10;
    output.max_acceleration = max_acc;
    output.min_acceleration = -max_acc;

    return output;
}

[[gnu::const]]
inline Eigen::Vector2d angle_vector(double angle)
{
    return Eigen::Vector2d(std::cos(angle),std::sin(angle));
}

template<DubinsMove m1, DubinsMove m2, size_t samples>
std::pair<double,double> sample_temporal_XY_dist(const PathShape<m1>& s1, const PathShape<m2>& s2,
    const AircraftStats& s1_stats, const AircraftStats& s2_stats, double duration)
{
    static_assert(samples > 1);
    Pose3D p1,p2;

    PathShape<m1> s1_cs = s1;
    PathShape<m2> s2_cs = s2;

    path_set_planar_speed(s1_cs,1.);
    path_set_planar_speed(s2_cs,1.);

    double t = 0.;
    p1 = follow_dubins(s1_cs,t);
    p2 = follow_dubins(s2_cs,t);

    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;

    double min_dist = dx*dx + dy*dy;
    double min_loc = 0.;
    
    for(size_t i = 1; i < samples; i++)
    {
        t = i*duration/(samples-1);

        double l1  = length_traveled(s1_stats,t,duration);
        double l2  = length_traveled(s2_stats,t,duration);
        p1 = follow_dubins(s1_cs,l1);
        p2 = follow_dubins(s2_cs,l2);

        dx = p1.x - p2.x;
        dy = p1.y - p2.y;

        double dist = dx*dx + dy*dy;
        if (dist < min_dist)
        {
            min_dist = dist;
            min_loc = t;
        }
    }

    return {min_loc,sqrt(min_dist)};
}

template<size_t samples>
std::pair<double,double> sample_temporal_XY_dist(const DynamicPathShape& s1, const DynamicPathShape& s2,
    const AircraftStats& s1_stats, const AircraftStats& s2_stats, double duration)
{
    static_assert(samples > 1);
    Pose3D p1,p2;

    DynamicPathShape s1_cs = s1;
    DynamicPathShape s2_cs = s2;

    path_set_planar_speed(s1_cs,1.);
    path_set_planar_speed(s2_cs,1.);

    double t = 0.;
    p1 = follow_dubins(s1_cs,t);
    p2 = follow_dubins(s2_cs,t);

    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;

    double min_dist = dx*dx + dy*dy;
    double min_loc = 0.;
    
    for(size_t i = 1; i < samples; i++)
    {
        t = i*duration/(samples-1);

        double l1  = length_traveled(s1_stats,t,duration);
        double l2  = length_traveled(s2_stats,t,duration);
        p1 = follow_dubins(s1_cs,l1);
        p2 = follow_dubins(s2_cs,l2);

        dx = p1.x - p2.x;
        dy = p1.y - p2.y;

        double dist = dx*dx + dy*dy;
        if (dist < min_dist)
        {
            min_dist = dist;
            min_loc = t;
        }
    }

    return {min_loc,sqrt(min_dist)};
}

template<size_t samples>
std::pair<double,double> sample_temporal_XY_dist(const Dubins& s1, const Dubins& s2,
    const AircraftStats& s1_stats, const AircraftStats& s2_stats, double duration)
{
    static_assert(samples > 1);
    Pose3D p1,p2;

    p1 = s1.get_start();
    p2 = s2.get_start();

    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;

    double min_dist = dx*dx + dy*dy;
    double min_loc = 0.;

    std::vector<double> timestamps;
    
    for(size_t i = 0; i < samples; i++)
    {
        timestamps.push_back(i*duration/(samples-1));
    }

    auto s1_samples = s1.get_positions(timestamps,s1_stats,true);
    auto s2_samples = s2.get_positions(timestamps,s2_stats,true);

    for(size_t i = 0; i < samples; i++)
    {
        p1 = s1_samples[i];
        p2 = s2_samples[i];

        dx = p1.x - p2.x;
        dy = p1.y - p2.y;

        double dist = dx*dx + dy*dy;
        if (dist < min_dist)
        {
            min_dist = dist;
            min_loc = i*duration/(samples-1);
        }
    }

    return {min_loc,sqrt(min_dist)};
}

Pose3D generate_pose_2D(int seed, double xy_limit)
{
    std::default_random_engine gen(seed); // Some seeded RNG 

    xy_limit    = std::abs(xy_limit);

    std::uniform_real_distribution<double> dis_pos(-xy_limit, xy_limit);
    std::uniform_real_distribution<double> dis_angle(-M_PI,M_PI);

    Pose3D output;
    output.x = dis_pos(gen);
    output.y = dis_pos(gen);
    output.z = 0.;
    output.theta = dis_angle(gen);

    return output;
}

Pose3D generate_pose(int seed, double xy_limit, double z_limit)
{
    std::default_random_engine gen(seed); // Some seeded RNG 

    xy_limit    = std::abs(xy_limit);
    z_limit     = std::abs(z_limit);

    std::uniform_real_distribution<double> dis_pos(-xy_limit, xy_limit);
    std::uniform_real_distribution<double> dis_alt(-z_limit, z_limit);
    std::uniform_real_distribution<double> dis_angle(-M_PI,M_PI);

    Pose3D output;
    output.x = dis_pos(gen);
    output.y = dis_pos(gen);
    output.z = dis_alt(gen);
    output.theta = dis_angle(gen);

    return output;
}