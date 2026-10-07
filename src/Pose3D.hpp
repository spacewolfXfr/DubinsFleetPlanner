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

#include <cmath>
#include <tuple>
#include <string>
#include <vector>

// -------------------- Pose and mouvements -------------------- //

typedef struct {
    double x    ; // X position
    double y    ; // Y position
    double z    ; // Z position
    double theta; // XY Orientation (in radiants, 0 is pure X, pi/2 is pure Y)
} Pose3D;

const Pose3D UndefPose{
    NAN,NAN,NAN,NAN
};

std::string pose_to_string(const Pose3D& p);

/**
 * @brief Apply a straight (XY) movement to the Pose
 * 
 * This function modifies the given pose
 * 
 * @param pose          Starting pose 
 * @param duration      Movement duration
 * @param speed         XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m 
 */
void move_straight(Pose3D* pose, double duration, double speed, double climb_rate);

/**
 * @brief Apply a straight (XY) movement to the Pose
 * 
 * This function returns a fresh pose without modifying the starting one
 * 
 * @param pose          Starting pose 
 * @param duration      Movement duration (s)
 * @param speed         XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m 
 * @return Pose3D       Resulting position
 */
Pose3D move_straight(const Pose3D& pose, double duration, double speed, double climb_rate);

/**
 * @brief Apply a left turn (XY) movement to the Pose
 * 
 * This function modifies the given pose
 * 
 * @param pose          Starting pose 
 * @param duration      Movement duration (s)
 * @param speed         XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m
 */
void turn_left(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius);

/**
 * @brief Apply a left turn (XY) movement to the Pose
 * 
 * This function returns a fresh pose without modifying the starting one
 * 
 * @param pose          Starting pose 
 * @param duration      Movement duration (s)
 * @param speed         XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m 
 * @return Pose3D       Resulting position
 */
Pose3D turn_left(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius);

/**
 * @brief Apply a right turn (XY) movement to the Pose
 * 
 * This function modifies the given pose
 * 
 * @param pose          Starting pose 
 * @param duration      Movement duration (s)
 * @param speed         XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m
 */
void turn_right(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius);

/**
 * @brief Apply a right turn (XY) movement to the Pose
 * 
 * This function returns a fresh pose without modifying the starting one
 * 
 * @param pose          Starting pose 
 * @param duration      Movement duration (s)
 * @param speed         XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m 
 * @return Pose3D       Resulting position
 */
Pose3D turn_right(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius);

/**
 * @brief  Compute the normalized equivalent of the given path planning problem.
 * 
 * That is, given the initial pose `start` and final pose `end`, compute the equivalent 
 * start at (0,0) oriented with angle `alpha` (radian) and end at (d,0) oriented with angle `beta`
 * 
 * @param start     Initial pose
 * @param end       Ending pose
 * @return std::tuple<double,double,double,double> (alpha, beta, d, theta), with theta being the rotation needed
 * to align the `end` on the X axis
 */
std::tuple<double,double,double,double> normalize_poses(const Pose3D& start, const Pose3D& end);

// ---------- Distance between two poses ---------- //

/**
 * @brief Compute the 3D Euclidean distance between two poses
 * 
 * @param p1 First pose
 * @param p2 Second pose
 * @return double 3D Euclidean distance between the points
 */
double pose_dist(const Pose3D& p1, const Pose3D& p2);

/**
 * @brief Compute the angle difference between two poses (i.e. the difference between the two theta values)
 * 
 * The difference is put in the [-PI,PI] angular domain
 * 
 * @param p1 First pose
 * @param p2 Second pose
 * @return double Angle difference, in [-PI,PI]
 */
double pose_aligned(const Pose3D& p1, const Pose3D& p2);

/**
 * @brief Compute the 2D Euclidean distance between two poses, using the XY components
 * 
 * @param p1 First pose
 * @param p2 Second pose
 * @return double 2D Euclidean distance between the XY projected points
 */
double pose_dist_XY(const Pose3D& p1, const Pose3D& p2);

// ---------- Minimal distance in set ---------- //

/**
 * @brief Generic function computing the minimal distance in a vector of points, given some distance function
 * 
 * @tparam F Distance function type between two Pose3D
 * @param poses Vector of poses
 * @return std::tuple<uint,uint,double> id0, id1, min_dist identifying the points and minimal distance in the given set 
 */
template<double (*distance_fun)(const Pose3D&, const Pose3D&)>
std::tuple<uint,uint,double> min_vec_poses_dist(const std::vector<Pose3D>& poses)
{
    uint N = poses.size();

    double min_dist = INFINITY;
    uint i0,j0;

    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            double dist = distance_fun(poses[i],poses[j]);
            if (dist < min_dist)
            {
                min_dist = dist;
                i0 = i;
                j0 = j;
            }
        }
    }

    return {i0,j0,min_dist};
}

[[gnu::pure]]
inline std::tuple<uint,uint,double> min_vec_poses_dist_XY(const std::vector<Pose3D>& poses)
{
    return min_vec_poses_dist<pose_dist_XY>(poses);
}

/**
 * @brief Generic function computing the minimal distance in an array of points, given some distance function
 * 
 * @tparam F Distance function between two Pose3D
 * @tparam N Number of poses
 * @param poses Array of poses
 * @return std::tuple<uint,uint,double> id0, id1, min_dist identifying the points and minimal distance in the given set 
 */
template<std::regular_invocable<Pose3D,Pose3D> F, uint N>
std::tuple<uint,uint,double> min_poses_dist(F f, const std::array<Pose3D,N>& poses)
{
    double min_dist = INFINITY;
    uint i0,j0;

    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            double dist = f(poses[i],poses[j]);
            if (dist < min_dist)
            {
                min_dist = dist;
                i0 = i;
                j0 = j;
            }
        }
    }

    return {i0,j0,min_dist};
}

/**
 * @brief Generic function computing the minimal distance in a dynamic-length array of points, given some distance function
 * 
 * @tparam F Distance function between two Pose3D
 * @param poses Array of poses
 * @param N Number of poses
 * @return std::tuple<uint,uint,double> id0, id1, min_dist identifying the points and minimal distance in the given set 
 */
template<std::regular_invocable<Pose3D,Pose3D> F>
std::tuple<uint,uint,double> min_poses_dist(F f, const Pose3D* poses, uint N)
{
    double min_dist = INFINITY;
    uint i0,j0;

    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            double dist = f(poses[i],poses[j]);
            if (dist < min_dist)
            {
                min_dist = dist;
                i0 = i;
                j0 = j;
            }
        }
    }

    return {i0,j0,min_dist};
}

template<uint N>
[[gnu::pure]]
inline std::tuple<uint,uint,double> min_poses_dist_XY(const std::array<Pose3D,N>& poses)
{
    return min_poses_dist(pose_dist_XY,poses);
}

// ---------- Poses span ---------- //


/**
 * @brief Returns min_x,max_x,min_y,max_y,min_z,max_z of the given poses
 * 
 * @param poses 
 * @return std::tuple<float,float,float,float,float,float> 
 */
std::tuple<float,float,float,float,float,float> poses_span(const std::vector<Pose3D>& poses);