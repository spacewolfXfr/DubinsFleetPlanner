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

#include "ExtendedDubins.hpp"

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

std::vector<std::unique_ptr<Dubins>> generate_line_extended_base(const Pose3D& start, const Pose3D& end, 
    double climb, double turn_radius, double target_len, double tol, const std::vector<double>& ratios)
{
    std::vector<std::unique_ptr<Dubins>> output;

    Pose3D norm_start(start),norm_end(end);
    norm_start.x /= turn_radius;
    norm_start.y /= turn_radius;

    norm_end.x /= turn_radius;
    norm_end.y /= turn_radius;

    for(double r : ratios)
    {
        double LSL_length = turn_radius*fit_shift_LSL_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(LSL_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_LSL>(
                start,end,r*LSL_length,(1-r)*LSL_length,turn_radius,tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
            }
            
        }

        double RSR_length = turn_radius*fit_shift_RSR_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(RSR_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_RSR>(
                start,end,r*RSR_length,(1-r)*RSR_length,turn_radius,tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
            }
        }

        double RSL_length = turn_radius*fit_shift_RSL_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(RSL_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_RSL>(
                start,end,r*RSL_length,(1-r)*RSL_length,turn_radius,tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
            }
        }

        double LSR_length = turn_radius*fit_shift_LSR_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(LSR_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_LSR>(
                start,end,r*LSR_length,(1-r)*LSR_length,turn_radius,tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
            }
        }

        double RLR_length = turn_radius*fit_shift_RLR_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(RLR_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_RLR>(
                start,end,r*RLR_length,(1-r)*RLR_length,turn_radius,tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
            }
        }

        double LRL_length = turn_radius*fit_shift_LRL_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(LRL_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_LRL>(
                start,end,r*LRL_length,(1-r)*LRL_length,turn_radius,tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
            }
        }

        double SLS_length = turn_radius*fit_shift_SLS_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(SLS_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_SLS>(
                start,end,r*SLS_length,(1-r)*SLS_length,turn_radius,tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
            }
        }

        double SRS_length = turn_radius*fit_shift_SRS_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(SRS_length))
        {
            std::vector<DynamicPathShape> shapes = generate_line_extended_pathshapes<set_radius_SRS>(
                start,end,r*SRS_length,(1-r)*SRS_length,turn_radius,tol);

            if (shapes.size() > 0)
            {
                output.push_back(
                    std::make_unique<Dubins>(start,end,shapes)
                );
            }
        } 

    }

    return output;
}

std::vector<std::unique_ptr<Dubins>> generate_all_extended_from_fitted_baseDubins(const Pose3D& start, const Pose3D& end, 
    std::vector<double> start_lens, std::vector<double> end_lens,
    const AircraftStats& stats, double target_len, double tol)
{
    return generate_all_extended_fitted_dubins<fit_possible_baseDubins>(start,end,start_lens,end_lens,stats,target_len,tol);
}