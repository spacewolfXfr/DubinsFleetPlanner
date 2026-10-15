#pragma once

#include <cmath>

/**
 * @brief Class for speed profile. May be upgraded for profiles more complex than linear (constant acceleration)
 * 
 */
class SpeedProfile
{
public:
    double base_speed; // Base speed (at t=0)
    double acceleration; // Acceleration (constant)

    SpeedProfile(double _base_speed, double _acceleration) : base_speed(base_speed), acceleration(acceleration) {}
    SpeedProfile() : base_speed(1.), acceleration(0.) {}
    SpeedProfile(double start_speed, double end_speed, double duration) : base_speed(start_speed), acceleration((end_speed - start_speed) / duration) {}

    static SpeedProfile from_length(double start_speed, double end_speed, double length);

    double get_speed_at_time(double time) const;

    double get_length_at_time(double time) const;

    double get_time_at_length(double length) const;

    void shift_by_time(double time);
    SpeedProfile shifted_by_time(double time) const;
};