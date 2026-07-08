#pragma once

#include "AbstractFleetPlanner.hpp"

// ******************** Helper functions ******************** //

static std::vector<std::unique_ptr<Dubins>> generate_both_straight_and_radius_fitted(double climb, double turn_radius,
    const Pose3D& start, const Pose3D& end, 
    double target_len, double tol, const std::vector<double>& ratios)
{
    std::vector<std::unique_ptr<Dubins>> output = generate_line_extended_base(start,end,climb,turn_radius,target_len,tol,ratios);
    std::vector<std::unique_ptr<Dubins>> curvature_extended = fit_possible_baseDubins(climb,turn_radius,start,end,target_len,tol);

    output.insert(output.end(),std::move_iterator(curvature_extended.begin()),std::move_iterator(curvature_extended.end()));

    return output;
}

static std::vector<std::unique_ptr<Dubins>> generate_both_straight_and_radius_fitted_fixed_ratios(double climb, double turn_radius,
    const Pose3D& start, const Pose3D& end, 
    double target_len, double tol)
{
    std::vector<std::unique_ptr<Dubins>> output = generate_line_extended_base(start,end,climb,turn_radius,target_len,tol,{0.,0.5,1.});
    std::vector<std::unique_ptr<Dubins>> curvature_extended = fit_possible_baseDubins(climb,turn_radius,start,end,target_len,tol);

    output.insert(output.end(),std::move_iterator(curvature_extended.begin()),std::move_iterator(curvature_extended.end()));

    return output;
}

// ******************** Planners subclasses ******************** //

/**
 * @brief Plannification based on any previous paths generator extended by basic primitives (S,L,R) of fixed lengths
 * 
 */
template<Dubins::FittedPathGeneratorFunction generator_function, uint generated_path_num>
class ExtendedDubinsFleetPlanner : public AbstractFleetPlanner
{
private:
    std::vector<double> start_lengths,end_lengths; // Finite list of possible lengths for starting and ending extra path

    std::vector<std::unique_ptr<Dubins>> generate_possible_paths(const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
    {
        return generate_extended_fitted_dubins<generator_function>(start,end,start_lengths,end_lengths,stats.climb,stats.turn_radius,target_len,precision_tol);
    }

    uint max_path_num() const
    {
        bool zero_in_starts = false;
        bool zero_in_ends   = false;

        for(double val : start_lengths)
        {
            if (abs(val) < precision_tol)
            {
                zero_in_starts = true;
                break;
            }
        }

        for(double val : end_lengths)
        {
            if (abs(val) < precision_tol)
            {
                zero_in_ends = true;
                break;
            }
        }

        uint nonzero_starts = start_lengths.size()  + ((zero_in_starts) ? (-1) : 0);
        uint nonzero_ends   = end_lengths.size()    + ((zero_in_ends)   ? (-1) : 0);

        uint nbr_of_possibilities = DubinsMoveNum*nonzero_starts*generated_path_num*nonzero_ends*DubinsMoveNum; // Non-zero paths

        if (zero_in_starts)
        {
            nbr_of_possibilities += generated_path_num*nonzero_ends*DubinsMoveNum; // Starts with 0, ends with non-zero
        }

        if (zero_in_ends)
        {
            nbr_of_possibilities += DubinsMoveNum*nonzero_starts*generated_path_num; // Ends with 0, starts with non-zero
        }

        if (zero_in_starts && zero_in_ends)
        {
            nbr_of_possibilities += generated_path_num; // Starts and ends with 0
        }

        return nbr_of_possibilities;
    } 

public:
    ExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens)
        : AbstractFleetPlanner(_prec,_max_r_dur), start_lengths(_slens), end_lengths(_elens) {}

    ExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens, int verb)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb), start_lengths(_slens), end_lengths(_elens) {}

    ExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens, int verb, std::ostream& output_stream)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb, output_stream), start_lengths(_slens), end_lengths(_elens) {}

    std::string my_name() const override
    {
        return "ExtendedDubinsFleetPlanner";
    }

};

/**
 * @brief Plannification based on any previous paths generator extended by straights only (S) of fixed lengths
 * 
 */
template<Dubins::FittedPathGeneratorFunction generator_function, uint generated_path_num>
class StraightExtendedDubinsFleetPlanner : public AbstractFleetPlanner
{
private:
    std::vector<double> start_lengths,end_lengths; // Finite list of possible lengths for starting and ending extra path

    std::vector<std::unique_ptr<Dubins>> generate_possible_paths(const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
    {
        return generate_straight_extended_fitted_dubins<generator_function>(start,end,start_lengths,end_lengths,stats.climb,stats.turn_radius,target_len,precision_tol);
    }

    uint max_path_num() const
    {
        return start_lengths.size()*generated_path_num*end_lengths.size();
    }
    
    /**
     * @brief Generate the list of shortest paths with specified endlengths. It is a list as it considers the shortest path *for each type of path*
     * 
     * @param start     Starting pose
     * @param end       Ending pose 
     * @param stats     Aircraft statistics
     * @param wind_x    Wind, X component (TODO: integrate properly)
     * @param wind_y    Wind, Y component (TODO: integrate properly)
     * @return std::vector<std::unique_ptr<Dubins>> 
     */
    std::vector<std::unique_ptr<Dubins>> generate_shortest_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double wind_x, double wind_y) const override
        {
            std::vector<std::unique_ptr<Dubins>> output;

            for(double sl : start_lengths)
            {
                Pose3D shifted_start = follow_dubins<STRAIGHT>(start,sl, 1., 0., stats.turn_radius);        //TODO: Verticality 
                DynamicPathShape shape_start(STRAIGHT, start, shifted_start, 1., stats.turn_radius, 0.);    //TODO: Verticality 

                for(double el : end_lengths)
                {
                    Pose3D shifted_end = follow_dubins<STRAIGHT>(end,-el, 1., 0., stats.turn_radius);       //TODO: Verticality 
                    DynamicPathShape shape_end(STRAIGHT, shifted_end, end, 1., stats.turn_radius, 0.);      //TODO: Verticality

                    auto possible = list_possible_baseDubins(stats.climb,stats.turn_radius,shifted_start,shifted_end,wind_x,wind_y);

                    for(auto& b : possible)
                    {
                        std::vector<DynamicPathShape> extended_shapes = {shape_start};
                        extended_shapes.insert(
                            extended_shapes.end(),
                            b->get_all_sections().begin(),
                            b->get_all_sections().end());

                        extended_shapes.push_back(shape_end);

                        output.push_back(std::make_unique<Dubins>(start,end,extended_shapes));
                    }     
                }
            }

            return output;
        }

public:
    StraightExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens)
        : AbstractFleetPlanner(_prec,_max_r_dur), start_lengths(_slens), end_lengths(_elens) {}

    StraightExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens, int verb)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb), start_lengths(_slens), end_lengths(_elens) {}

    StraightExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens, int verb, std::ostream& output_stream)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb, output_stream), start_lengths(_slens), end_lengths(_elens) {}

    std::string my_name() const override
    {
        return "StraightExtendedDubinsFleetPlanner";
    }
};



/**
 * @brief Plannification based on the 8 basic Dubins paths extended by basic primitives
 * 
 */
class BaseExtendedDubinsFleetPlanner : public AbstractFleetPlanner
{
private:
    std::vector<double> start_lengths,end_lengths; // Finite list of possible lengths for starting and ending extra path

    std::vector<std::unique_ptr<Dubins>> generate_possible_paths(const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
    {
        return generate_all_extended_from_fitted_baseDubins(start,end,start_lengths,end_lengths,stats.climb,stats.turn_radius,target_len,precision_tol);
    }

    uint max_path_num() const
    {
        bool zero_in_starts = false;
        bool zero_in_ends   = false;

        for(double val : start_lengths)
        {
            if (abs(val) < precision_tol)
            {
                zero_in_starts = true;
                break;
            }
        }

        for(double val : end_lengths)
        {
            if (abs(val) < precision_tol)
            {
                zero_in_ends = true;
                break;
            }
        }

        uint nonzero_starts = start_lengths.size()  + ((zero_in_starts) ? (-1) : 0);
        uint nonzero_ends   = end_lengths.size()    + ((zero_in_ends)   ? (-1) : 0);

        uint nbr_of_possibilities = DubinsMoveNum*nonzero_starts*NumberOfBaseDubins*nonzero_ends*DubinsMoveNum; // Non-zero paths

        if (zero_in_starts)
        {
            nbr_of_possibilities += NumberOfBaseDubins*nonzero_ends*DubinsMoveNum; // Starts with 0, ends with non-zero
        }

        if (zero_in_ends)
        {
            nbr_of_possibilities += DubinsMoveNum*nonzero_starts*NumberOfBaseDubins; // Ends with 0, starts with non-zero
        }

        if (zero_in_starts && zero_in_ends)
        {
            nbr_of_possibilities += DubinsMoveNum; // Starts and ends with 0
        }

        return nbr_of_possibilities;
    } 

public:
    BaseExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens)
        : AbstractFleetPlanner(_prec,_max_r_dur), start_lengths(_slens), end_lengths(_elens) {}

    BaseExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens, int verb)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb), start_lengths(_slens), end_lengths(_elens) {}

    BaseExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens, int verb, std::ostream& output_stream)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb, output_stream), start_lengths(_slens), end_lengths(_elens) {}


    std::string my_name() const override
    {
        return "BaseExtendedDubinsFleetPlanner";
    }
};


/**
 * @brief Plannification based on the 8 basic Dubins paths using the minimal turn radius,
 * extended by well-chosen straight at the start and end
 * 
 */
class LineExtendedDubinsFleetPlanner : public AbstractFleetPlanner
{
private:
    std::vector<double> ratios;

    std::vector<std::unique_ptr<Dubins>> generate_possible_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
    {
        return generate_both_straight_and_radius_fitted(stats.climb,stats.turn_radius,start,end,target_len,precision_tol, this->ratios);
    }

    uint max_path_num() const
    {
        return 3*8+8; // Three types of extensions, as well as no extensions for the 8 curvature adjusted Dubins
    } 

public:
    LineExtendedDubinsFleetPlanner(double _prec, double _max_r_dur, const std::vector<double>& rs)
        : AbstractFleetPlanner(_prec,_max_r_dur),ratios(rs){}

    LineExtendedDubinsFleetPlanner(double _prec, double _max_r_dur, const std::vector<double>& rs,
        int verb)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb),ratios(rs){}

    LineExtendedDubinsFleetPlanner(double _prec, double _max_r_dur, const std::vector<double>& rs,
        int verb, std::ostream& output_stream)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb, output_stream),ratios(rs){}

    std::string my_name() const override
    {
        return "LineExtendedDubinsFleetPlanner";
    }
};
