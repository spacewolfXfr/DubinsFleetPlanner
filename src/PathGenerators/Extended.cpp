#include "PathGenerators/Extended.hpp"

#include "BaseDubins.hpp"

// ---------- AllExtendedPathsGenerator ---------- //


std::vector<std::unique_ptr<Dubins>> AllExtendedPathsGenerator::generate_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats) const
{
    std::vector<std::unique_ptr<Dubins>> output,tmp;

    auto unique_ptr_transfer = [&](std::vector<std::unique_ptr<Dubins>>& v)
    {
        for(std::unique_ptr<Dubins>& p : v)
        {
            output.push_back(std::move(p));
        }
    };

    // STRAIGHT first
    tmp = generate_extended_paths<STRAIGHT,STRAIGHT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_paths<STRAIGHT,LEFT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_paths<STRAIGHT,RIGHT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats);

    unique_ptr_transfer(tmp);

    // LEFT first
    tmp = generate_extended_paths<LEFT,STRAIGHT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_paths<LEFT,LEFT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_paths<LEFT,RIGHT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats);

    unique_ptr_transfer(tmp);

    // RIGHT first
    tmp = generate_extended_paths<RIGHT,STRAIGHT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_paths<RIGHT,LEFT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_paths<RIGHT,RIGHT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats);

    unique_ptr_transfer(tmp);

    return output;
}

uint AllExtendedPathsGenerator::max_path_num() const
{
    return start_lengths.size() * end_lengths.size() * underlying->max_path_num() * 9;
}


// ---------- AllExtendedFittedPathsGenerator ---------- //

std::vector<std::unique_ptr<Dubins>> AllExtendedFittedPathsGenerator::generate_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
{
    std::vector<std::unique_ptr<Dubins>> output,tmp;

    auto unique_ptr_transfer = [&](std::vector<std::unique_ptr<Dubins>>& v)
    {
        for(std::unique_ptr<Dubins>& p : v)
        {
            output.push_back(std::move(p));
        }
    };

    // STRAIGHT first
    tmp = generate_extended_fitted_paths<STRAIGHT,STRAIGHT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats, target_len);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_fitted_paths<STRAIGHT,LEFT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats, target_len);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_fitted_paths<STRAIGHT,RIGHT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats, target_len);

    unique_ptr_transfer(tmp);

    // LEFT first
    tmp = generate_extended_fitted_paths<LEFT,STRAIGHT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats, target_len);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_fitted_paths<LEFT,LEFT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats, target_len);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_fitted_paths<LEFT,RIGHT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats, target_len);

    unique_ptr_transfer(tmp);

    // RIGHT first
    tmp = generate_extended_fitted_paths<RIGHT,STRAIGHT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats, target_len);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_fitted_paths<RIGHT,LEFT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats, target_len);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_fitted_paths<RIGHT,RIGHT>(
        *underlying, start, end,
        start_lengths, end_lengths,
        stats, target_len);

    unique_ptr_transfer(tmp);

    return output;
}

uint AllExtendedFittedPathsGenerator::max_path_num() const
{
    return start_lengths.size() * end_lengths.size() * underlying->max_path_num() * 9;
}

// ---------- FittedStraightBaseDubins ---------- //

// Helper for generating length-fitted straight-extended paths
template<BasePathShapeGenerator generator_function>
std::vector<DynamicPathShape> generate_line_extended_pathshapes(
    const Pose3D& start, const Pose3D& end, double start_len, double end_len, double turn_radius, double tol)
{
    std::vector<DynamicPathShape> output;

    Pose3D shifted_start = move_straight(start,start_len,1.,0.);
    Pose3D shifted_end = move_straight(end,-end_len,1.,0.);

    DynamicPathShape start_shape(STRAIGHT,start,start_len,1.,turn_radius,0.);
    DynamicPathShape end_shape(STRAIGHT,shifted_end,end_len,1.,turn_radius,0.);

    std::vector<DynamicPathShape> midshapes = generator_function(shifted_start,shifted_end,turn_radius,tol);

    if (midshapes.size() == 0)
    {
        return {};
    }

    if (start_shape.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(start_shape);
    }
    for(auto& s : midshapes)
    {
        if (s.length > DubinsFleetPlanner_ZERO_TOLERANCE)
        {
            output.push_back(s);
        }
    }
    if (end_shape.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(end_shape);
    }

    return output;
}

std::vector<std::unique_ptr<Dubins>> FittedStraightBaseDubins::generate_paths(
    const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
{
    std::vector<std::unique_ptr<Dubins>> output;

    double turn_radius = stats.turn_radius;

    Pose3D norm_start(start),norm_end(end);
    norm_start.x /= turn_radius;
    norm_start.y /= turn_radius;

    norm_end.x /= turn_radius;
    norm_end.y /= turn_radius;

    for(double r : ratios)
    {
        double LSL_length = turn_radius*fit_shift_LSL_ratio(norm_start,norm_end,target_len/turn_radius,precision_tol,r);
        if (!std::isnan(LSL_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_LSL>(
                start,end,r*LSL_length,(1-r)*LSL_length,turn_radius,precision_tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
                assert(abs(output.back()->get_length() - target_len) < 1e-1);
            }
            
        }

        double RSR_length = turn_radius*fit_shift_RSR_ratio(norm_start,norm_end,target_len/turn_radius,precision_tol,r);
        if (!std::isnan(RSR_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_RSR>(
                start,end,r*RSR_length,(1-r)*RSR_length,turn_radius,precision_tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
                assert(abs(output.back()->get_length() - target_len) < 1e-1);
            }
        }

        double RSL_length = turn_radius*fit_shift_RSL_ratio(norm_start,norm_end,target_len/turn_radius,precision_tol,r);
        if (!std::isnan(RSL_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_RSL>(
                start,end,r*RSL_length,(1-r)*RSL_length,turn_radius,precision_tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
                assert(abs(output.back()->get_length() - target_len) < 1e-1);
            }
        }

        double LSR_length = turn_radius*fit_shift_LSR_ratio(norm_start,norm_end,target_len/turn_radius,precision_tol,r);
        if (!std::isnan(LSR_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_LSR>(
                start,end,r*LSR_length,(1-r)*LSR_length,turn_radius,precision_tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
                assert(abs(output.back()->get_length() - target_len) < 1e-1);
            }
        }

        double RLR_length = turn_radius*fit_shift_RLR_ratio(norm_start,norm_end,target_len/turn_radius,precision_tol,r);
        if (!std::isnan(RLR_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_RLR>(
                start,end,r*RLR_length,(1-r)*RLR_length,turn_radius,precision_tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
                assert(abs(output.back()->get_length() - target_len) < 1e-1);
            }
        }

        double LRL_length = turn_radius*fit_shift_LRL_ratio(norm_start,norm_end,target_len/turn_radius,precision_tol,r);
        if (!std::isnan(LRL_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_LRL>(
                start,end,r*LRL_length,(1-r)*LRL_length,turn_radius,precision_tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
                assert(abs(output.back()->get_length() - target_len) < 1e-1);
            }
        }

        double SLS_length = turn_radius*fit_shift_SLS_ratio(norm_start,norm_end,target_len/turn_radius,precision_tol,r);
        if (!std::isnan(SLS_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_SLS>(
                start,end,r*SLS_length,(1-r)*SLS_length,turn_radius,precision_tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
                assert(abs(output.back()->get_length() - target_len) < 1e-1);
            }
        }

        double SRS_length = turn_radius*fit_shift_SRS_ratio(norm_start,norm_end,target_len/turn_radius,precision_tol,r);
        if (!std::isnan(SRS_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_SRS>(
                start,end,r*SRS_length,(1-r)*SRS_length,turn_radius,precision_tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
                if (abs(output.back()->get_length() - target_len) > 1e-1)
                {
                    shapes = generate_line_extended_pathshapes<set_radius_SRS>(
                start,end,r*SRS_length,(1-r)*SRS_length,turn_radius,precision_tol);
                    output.push_back(
                        std::make_unique<Dubins>(start,end,shapes)
                    );
                }
                assert(abs(output.back()->get_length() - target_len) < 1e-1);
            }
        } 

    }

    return output;
}
