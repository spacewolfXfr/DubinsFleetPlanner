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

#include <format>

#include "Dubins.hpp"
#include "BaseDubins.hpp"
#include "Pose3D.hpp"

// ---------- Generator functions ---------- //

// ----- Generic templates ----- //

/**
 * @brief Given an arbitrary fitted path generating function, extend the generated paths
 * 
 * @tparam beginMove    Type of the begin extension
 * @tparam endMove      Type of the end extension
 * @tparam gen_F_T      Generic type for the path generator function
 * @param generator_function    Path generator function
 * @param start     Path start
 * @param end       Path end
 * @param start_lens    Initial length extensions
 * @param end_lens      Final length extensions
 * @param stats         Aircraft stats
 * @param target_len    Target length
 * @param tol           Target fitter tolerance
 * @return std::vector<std::unique_ptr<Dubins>> 
 */
template <DubinsMove beginMove, DubinsMove endMove, Dubins::FittedPathGeneratorFunction generator_function>
std::vector<std::unique_ptr<Dubins>> generate_extended_fitted_dubins(
    const Pose3D& start, const Pose3D& end, std::vector<double> start_lens, std::vector<double> end_lens,
    const AircraftStats& stats, double target_len, double tol)
{
    std::vector<std::unique_ptr<Dubins>> output;

    for(double sl : start_lens)
    {

        Pose3D shift_start = follow_dubins<beginMove>(start,sl,1.,0., stats.turn_radius);
        DynamicPathShape shape_start(beginMove, start, shift_start, 1., stats.turn_radius, 0.);

        for(double el : end_lens)
        {
            Pose3D shift_end = follow_dubins<endMove>(end,-el,1.,0., stats.turn_radius);
            DynamicPathShape shape_end(beginMove, shift_end, end, 1., stats.turn_radius, 0.);

            if (target_len - sl - el < 0.)
            {
                continue;
            }

            auto generated = generator_function(stats, shift_start, shift_end, target_len-sl-el,tol);


            for(auto& b : generated)
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

/**
 * @brief Generate fitted extended Dubins path using a given generator function
 * 
 * The Extended Dubins paths are made by adding set paths after the start and before the end,
 * thus generating a new Dubins problem which is solved normally. 
 * 
 * @tparam generator_function A function generating a list of Dubins paths achieving a target length
 * @param start     Start pose
 * @param end       End pose
 * @param start_lens    Lengths for the added initial segment
 * @param end_lens      Lengths for the added final segment
 * @param climb         Climb ratio
 * @param turn_radius   Minimal turn radius
 * @param target_len    Target length
 * @param tol           Precision for target length
 * @return std::vector<std::unique_ptr<Dubins>>     List of generated paths
 */
template <Dubins::FittedPathGeneratorFunction generator_function>
std::vector<std::unique_ptr<Dubins>> generate_all_extended_fitted_dubins(
    const Pose3D& start, const Pose3D& end, std::vector<double> start_lens, std::vector<double> end_lens,
    const AircraftStats& stats, double target_len, double tol)
{
    std::vector<std::unique_ptr<Dubins>> output,tmp;

    auto unique_ptr_transfer = [&](std::vector<std::unique_ptr<Dubins>>& v)
    {
        for(std::unique_ptr<Dubins>& p : v)
        {
            output.push_back(std::move(p));
        }
    };

    tmp = generate_extended_fitted_dubins<STRAIGHT,STRAIGHT,generator_function>(
        start,end,start_lens,end_lens,stats,target_len,tol);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_fitted_dubins<STRAIGHT,LEFT,generator_function>(
        start,end,start_lens,end_lens,stats,target_len,tol);

    unique_ptr_transfer(tmp);
        
    tmp = generate_extended_fitted_dubins<STRAIGHT,RIGHT,generator_function>(
        start,end,start_lens,end_lens,stats,target_len,tol);

    unique_ptr_transfer(tmp);
    

    
    tmp = generate_extended_fitted_dubins<LEFT,STRAIGHT,generator_function>(
        start,end,start_lens,end_lens,stats,target_len,tol);

    unique_ptr_transfer(tmp);
    
    tmp = generate_extended_fitted_dubins<LEFT,LEFT,generator_function>(
        start,end,start_lens,end_lens,stats,target_len,tol);

    unique_ptr_transfer(tmp);
    
    tmp = generate_extended_fitted_dubins<LEFT,RIGHT,generator_function>(
        start,end,start_lens,end_lens,stats,target_len,tol);

    unique_ptr_transfer(tmp);



    tmp = generate_extended_fitted_dubins<RIGHT,STRAIGHT,generator_function>(
        start,end,start_lens,end_lens,stats,target_len,tol);

    unique_ptr_transfer(tmp);
    
    tmp = generate_extended_fitted_dubins<RIGHT,LEFT,generator_function>(
        start,end,start_lens,end_lens,stats,target_len,tol);

    unique_ptr_transfer(tmp);
    
    tmp = generate_extended_fitted_dubins<RIGHT,RIGHT,generator_function>(
        start,end,start_lens,end_lens,stats,target_len,tol);

    unique_ptr_transfer(tmp);
    

    return output;
}

template <Dubins::FittedPathGeneratorFunction generator_function>
std::vector<std::unique_ptr<Dubins>> generate_straight_extended_fitted_dubins(
    const Pose3D& start, const Pose3D& end, std::vector<double> start_lens, std::vector<double> end_lens,
    const AircraftStats& stats, double target_len, double tol)
{
    return generate_extended_fitted_dubins<STRAIGHT,STRAIGHT,generator_function>(
        start,end,start_lens,end_lens,stats,target_len,tol);
}

/**
 * @brief Generate Dubins bath with minimal turn radius and adjusted initial and final straight segments
 * 
 * @param start     Start pose
 * @param end       End pose
 * @param climb         Climb ratio (TODO: Handle verticality)
 * @param turn_radius   Minimal turn radius
 * @param target_len    Target length
 * @param tol           Precision for target length
 * @return std::vector<std::unique_ptr<Dubins>> List of generated paths
 */
std::vector<std::unique_ptr<Dubins>> generate_line_extended_base(const Pose3D& start, const Pose3D& end, 
    const AircraftStats& stats, double target_len, double tol, const std::vector<double>& ratios = {0.,0.5,1.});

// ----- Path generation functions ----- //

/**
 * @brief Generate fitted extended Dubins path from base ones
 * 
 * The Extended Dubins paths are made by adding set paths after the start and before the end,
 * thus generating a new Dubins problem which is solved normally. 
 * 
 * @param start     Start pose
 * @param end       End pose
 * @param start_lens    Lengths for the added initial segment
 * @param end_lens      Lengths for the added final segment
 * @param climb         Climb ratio (TODO: Handle verticality)
 * @param turn_radius   Minimal turn radius
 * @param target_len    Target length
 * @param tol           Precision for target length
 * @return std::vector<std::unique_ptr<Dubins>>     List of generated paths
 */
std::vector<std::unique_ptr<Dubins>> generate_all_extended_from_fitted_baseDubins(const Pose3D& start, const Pose3D& end, 
    std::vector<double> start_lens, std::vector<double> end_lens,
    const AircraftStats& stats, double target_len, double tol);