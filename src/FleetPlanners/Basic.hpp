#pragma once

#include "AbstractFleetPlanner.hpp"

/**
 * @brief Plannification based only on the 6+2 basic Dubins paths
 * 
 */
class BasicDubinsFleetPlanner : public AbstractFleetPlanner
{
private:
    std::vector<std::unique_ptr<Dubins>> generate_possible_paths(const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
    {
        return fit_possible_baseDubins(stats.climb,stats.turn_radius,start,end,target_len,precision_tol);
    }

    uint max_path_num() const
    {
        return NumberOfBaseDubins;
    }

public:
    BasicDubinsFleetPlanner(double _prec, double _max_r_dur) :
        AbstractFleetPlanner(_prec,_max_r_dur) {}

    BasicDubinsFleetPlanner(double _prec, double _max_r_dur, int verb) :
        AbstractFleetPlanner(_prec,_max_r_dur,verb) {}

    BasicDubinsFleetPlanner(double _prec, double _max_r_dur, int verb, std::ostream& output_stream) :
        AbstractFleetPlanner(_prec,_max_r_dur,verb, output_stream) {}
};