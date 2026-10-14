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

#include <stdexcept>
#include "Aircraft.h"

#include "ProjectHeader.h"

double length_traveled(const AircraftStats& stats, double t, double trajectory_duration)
{
    double duration = trajectory_duration;
    return t*t*(stats.target_airspeed-stats.airspeed)/(2.*duration) + stats.airspeed*t;
}

double speed_at_time(const AircraftStats& stats, double t, double trajectory_duration)
{
    double duration = trajectory_duration;
    return (stats.target_airspeed-stats.airspeed)*t/duration + stats.airspeed;
}

double time_to_travel_length(const AircraftStats& stats, double length, double trajectory_duration)
{
    double duration = trajectory_duration;
    double a = (stats.target_airspeed-stats.airspeed)/(2.*duration);
    double b = stats.airspeed;
    double c = -length;

    if (abs(a) < DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        if (abs(b) < DubinsFleetPlanner_ZERO_TOLERANCE)
        {
            throw std::runtime_error("No valid solution for time_to_travel_length: zero speed and zero acceleration");
        }
        return -c/b; // Linear case
    }

    double discriminant = b*b - 4*a*c;

    if (discriminant < 0)
    {
        throw std::runtime_error("No real solution for time_to_travel_length");
    }

    double sqrt_discriminant = std::sqrt(discriminant);

    double t1 = (-b + sqrt_discriminant) / (2*a);
    double t2 = (-b - sqrt_discriminant) / (2*a);

    // We want the positive root that is within the duration
    if (t1 >= 0 && t1 <= duration)
        return t1;
    else if (t2 >= 0 && t2 <= duration)
        return t2;
    else
        throw std::runtime_error("No valid time solution within trajectory duration");
}