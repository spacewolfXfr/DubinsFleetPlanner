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

typedef struct {
    int id;             // A numeric ID
    double airspeed;    // Current airspeed, in [length]/[time]
    double target_airspeed; // Airspeed at trajectory end, in [length]/[time]
    double turn_radius; // Minimal turn radius, in [length]
    double climb;       // Climb rate, in [altitude]/s (with [altitude] some unit for altitude; meters above sea level, feets above ground...)
    double min_acceleration; // Minimal acceleration (horizontal, along the path). Default to zero.
    double max_acceleration; // Maximal acceleration (horizontal, along the path). Default to zero.
} AircraftStats;

/**
 * @brief Compute the length traveled by an aircraft at a given time, assuming linearly changing speed according to the aircraft's stats.
 * 
 * @param stats Aircraft stats
 * @param t     Time
 * @param trajectory_duration Total duration of the followed trajectory
 * @return double Length traveled at time t
 */
double length_traveled(const AircraftStats& stats, double t, double trajectory_duration);

/**
 * @brief Compute the speed of an aircraft at a given time, assuming linearly changing speed according to the aircraft's stats.
 * 
 * @param stats Aircraft stats
 * @param t     Time
 * @param trajectory_duration Total duration of the followed trajectory
 * @return double Speed along the trajectory at time t
 */
double speed_at_time(const AircraftStats& stats, double t, double trajectory_duration);


/**
 * @brief Compute the time required to travel a given length, assuming linearly changing speed according to the aircraft's stats.
 * 
 * @param stats Aircraft stats
 * @param length Length to travel
 * @param trajectory_duration Total duration of the followed trajectory
 * @return double Time at which the aircraft has traveled the given length
 */
double time_to_travel_length(const AircraftStats& stats, double length, double trajectory_duration);
