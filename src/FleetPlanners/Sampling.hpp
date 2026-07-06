#pragma once

#include <random>

#include "AbstractFleetPlanner.hpp"

// ******************** Planners subclasses ******************** //

/**
 * @brief Plannification based on any previous paths generator and sampling randomly a middlepoint
 * 
 */
class SamplingDubinsFleetPlanner : public AbstractFleetPlanner
{
private:
    uint samples_num; // Number of samples to generate between the min and max points
    double min_x,max_x,min_y,max_y,min_z,max_z; // Bounding box for the sampling
    mutable std::mt19937 rng; // Random number generator
    std::unique_ptr<AbstractFleetPlanner> underlying_planner; // Underlying planner to use for generating paths


    std::vector<std::unique_ptr<Dubins>> generate_possible_paths(const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
    {
        auto generated = underlying_planner->generate_possible_paths(start,end,stats,target_len);
        for(uint i = 0; i < samples_num; i++)
        {
            Pose3D sample_pt;
            sample_pt.x = std::uniform_real_distribution<double>(min_x,max_x)(rng);
            sample_pt.y = std::uniform_real_distribution<double>(min_y,max_y)(rng);
            sample_pt.z = std::uniform_real_distribution<double>(min_z,max_z)(rng);
            sample_pt.theta = std::uniform_real_distribution<double>(-M_PI,M_PI)(rng);

            double start_dist   = pose_dist(sample_pt,start);
            double end_dist     = pose_dist(sample_pt,end);

            if (start_dist + end_dist > target_len)
            {
                continue; // Sample point is too far away to be reachable
            }

            double r1 = start_dist / (start_dist + end_dist);
            double r2 = end_dist / (start_dist + end_dist);

            auto start_to_sample = underlying_planner->generate_possible_paths(start, sample_pt, stats, r1*target_len);
            auto sample_to_end = underlying_planner->generate_possible_paths(sample_pt, end, stats, r2*target_len);

            for(const auto& p1 : start_to_sample)
            {
                for(const auto& p2 : sample_to_end)
                {
                    auto joined = join_paths(*p1,*p2,precision_tol);
                    if (joined.is_valid())
                    {
                        generated.push_back(std::make_unique<Dubins>(joined));
                    }
                }
            }

        }

        return generated;

    }

    uint max_path_num() const
    {
        uint generated_path_num = underlying_planner->max_path_num();
        return samples_num*generated_path_num*generated_path_num;
    } 

public:
    SamplingDubinsFleetPlanner(double _prec, double _max_r_dur,
        std::unique_ptr<AbstractFleetPlanner>&& _underlying_planner, uint _samples_num, Pose3D _min_pt, Pose3D _max_pt)
        : AbstractFleetPlanner(_prec,_max_r_dur), underlying_planner(std::move(_underlying_planner)), samples_num(_samples_num), 
        min_x(_min_pt.x), max_x(_max_pt.x), min_y(_min_pt.y), max_y(_max_pt.y), min_z(_min_pt.z), max_z(_max_pt.z) {}

    SamplingDubinsFleetPlanner(double _prec, double _max_r_dur,
        std::unique_ptr<AbstractFleetPlanner>&& _underlying_planner, uint _samples_num, Pose3D _min_pt, Pose3D _max_pt, int verb)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb), underlying_planner(std::move(_underlying_planner)), samples_num(_samples_num), 
        min_x(_min_pt.x), max_x(_max_pt.x), min_y(_min_pt.y), max_y(_max_pt.y), min_z(_min_pt.z), max_z(_max_pt.z) {}

    SamplingDubinsFleetPlanner(double _prec, double _max_r_dur,
        std::unique_ptr<AbstractFleetPlanner>&& _underlying_planner, uint _samples_num, Pose3D _min_pt, Pose3D _max_pt, int verb, std::ostream& output_stream)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb, output_stream), underlying_planner(std::move(_underlying_planner)), samples_num(_samples_num), 
        min_x(_min_pt.x), max_x(_max_pt.x), min_y(_min_pt.y), max_y(_max_pt.y), min_z(_min_pt.z), max_z(_max_pt.z) {}

    void set_rng_seed(uint seed)
    {
        rng.seed(seed);
    }

};
