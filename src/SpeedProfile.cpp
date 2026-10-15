#include "SpeedProfile.hpp"

SpeedProfile SpeedProfile::from_length(double start_speed, double end_speed, double length)
{
    return SpeedProfile(start_speed, (end_speed*end_speed-start_speed*start_speed)/(2*length));
}

double SpeedProfile::get_speed_at_time(double time) const
{
    return base_speed + acceleration * time;
}

double SpeedProfile::get_length_at_time(double time) const
{
    return 0.5 * acceleration * time * time + base_speed * time;
}

double SpeedProfile::get_time_at_length(double length) const
{
    // Solve the equation: length = speed*t + 0.5*acceleration*t^2
    if (acceleration == 0)
    {
        return length / base_speed;
    }
    else
    {
        double discriminant = base_speed * base_speed + 2 * acceleration * length;
        if (discriminant < 0)
        {
            return NAN;
        }
        else
        {
            // Find the two possible solutions
            double t1 = (-base_speed + std::sqrt(discriminant)) / acceleration;
            double t2 = (-base_speed - std::sqrt(discriminant)) / acceleration;

            if (t1 >= 0)
            {
                return t1;
            }
            else if (t2 >= 0)
            {
                return t2;
            }
            else
            {
                return NAN;
            }
        }
    }
}


void SpeedProfile::shift_by_time(double time)
{
    double new_speed = get_speed_at_time(time);
    base_speed = new_speed;
}

SpeedProfile SpeedProfile::shifted_by_time(double time) const
{
    SpeedProfile output(*this);
    output.shift_by_time(time);
    return output;
}