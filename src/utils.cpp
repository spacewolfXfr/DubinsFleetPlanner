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

#include "utils.hpp"

// -------------------- General maths -------------------- //

/**
 * @brief Reduce an angle in radian to [0,2*Pi]
 * 
 * @param x Input angle
 * @return double Reduced equivalent value in [0,2*Pi]
 */
[[gnu::const]]
double mod_2pi(double x) 
{
    double output = fmod(x,2*M_PI);
    return (output > 0) ? output : output + 2*M_PI;
}

/**
 * @brief Reduce an angle to its central form, i.e. into the interval [-Pi,Pi]
 * 
 * @param x Angle
 * @return double Equivalent to x in [-pi,pi]
 */
[[gnu::const]]
double central_angle(double x)
{
    double output = fmod(x,2*M_PI);

    if (output > M_PI)
    {
        return output - 2*M_PI;
    }

    if (output < -M_PI)
    {
        return output + 2*M_PI;
    }

    return output;
}   

// -------------------- Pose and mouvements -------------------- //

[[gnu::pure]]
std::string pose_to_string(const Pose3D& p)
{
    return "x    : " + std::to_string(p.x) + "\n"
        +  "y    : " + std::to_string(p.y) + "\n"
        +  "z    : " + std::to_string(p.z) + "\n"
        +  "theta: " + std::to_string(p.theta) + "\n";
}

void move_straight(Pose3D* pose, double duration, double speed, double climb_rate)
{
    double x_dir = std::cos(pose->theta);
    double y_dir = std::sin(pose->theta);

    pose->x += x_dir*duration*speed;
    pose->y += y_dir*duration*speed;
    pose->z += climb_rate*duration;
}

[[gnu::pure]]
Pose3D move_straight(const Pose3D& pose, double duration, double speed, double climb_rate)
{
    Pose3D output(pose);
    move_straight(&output,duration,speed,climb_rate);
    return output;
}

void turn_left(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius)
{
    double len   = speed*duration;  // Distance travelled
    double alpha = len/turn_radius; // Angle travelled (in radian)

    pose->x += turn_radius*( std::sin(pose->theta+alpha) - std::sin(pose->theta));
    pose->y += turn_radius*(-std::cos(pose->theta+alpha) + std::cos(pose->theta));
    pose->theta = mod_2pi(pose->theta+alpha);

    pose->z += climb_rate*duration;
}

[[gnu::pure]]
Pose3D turn_left(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius)
{
    Pose3D output(pose);
    turn_left(&output,duration,speed,climb_rate,turn_radius);
    return output;
}

void turn_right(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius)
{
    double len = speed*duration;    // Distance travelled
    double alpha = len/turn_radius; // Angle travelled (in radian)

    pose->x += turn_radius*(-std::sin(pose->theta-alpha) + std::sin(pose->theta));
    pose->y += turn_radius*( std::cos(pose->theta-alpha) - std::cos(pose->theta));
    pose->theta = mod_2pi(pose->theta-alpha);

    pose->z += climb_rate*duration;
}

[[gnu::pure]]
Pose3D turn_right(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius)
{
    Pose3D output(pose);
    turn_right(&output,duration,speed,climb_rate,turn_radius);
    return output;
}

[[gnu::pure]]
std::tuple<double,double,double,double> normalize_poses(const Pose3D& start, const Pose3D& end)
{
    double dx       = end.x - start.x;
    double dy       = end.y - start.y;
    double theta    = std::atan2(dy,dx);
    
    double alpha    = start.theta - theta;
    double beta     = end.theta - theta;
    double d        = std::sqrt(dx*dx+dy*dy);

    return {alpha,beta,d,theta};
}

[[gnu::pure]]
double pose_dist(const Pose3D& p1, const Pose3D& p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}

[[gnu::pure]]
double pose_aligned(const Pose3D& p1, const Pose3D& p2)
{
    double dtheta = p1.theta - p2.theta;
    double output = central_angle(mod_2pi(dtheta));
    return output;
}

[[gnu::pure]]
double pose_dist_XY(const Pose3D& p1, const Pose3D& p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx*dx+dy*dy);
}

std::tuple<float,float,float,float,float,float> poses_span(const std::vector<Pose3D>& poses)
{
    float min_x = INFINITY;
    float max_x = -INFINITY;
    float min_y = INFINITY;
    float max_y = -INFINITY;
    float min_z = INFINITY;
    float max_z = -INFINITY;

    for(const Pose3D& p : poses)
    {
        if (p.x < min_x) { min_x = p.x; }
        if (p.x > max_x) { max_x = p.x; }

        if (p.y < min_y) { min_y = p.y; }
        if (p.y > max_y) { max_y = p.y; }

        if (p.z < min_z) { min_z = p.z; }
        if (p.z > max_z) { max_z = p.z; }
    }

    return {min_x,max_x,min_y,max_y,min_z,max_z};
}