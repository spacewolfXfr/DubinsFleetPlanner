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


#include "BaseDubins.hpp"

#include "FleetPlanner.hpp"
#include "PathGenerators/all.hpp"

#include "ioUtils.hpp"

#include "randomPathShape.hpp"
#include "FleetDrawPrimitives.hpp"



#ifndef PLOTTING_SAMPLES
#define PLOTTING_SAMPLES 500
#endif

#ifndef TEST_POS_RANGE
#define TEST_POS_RANGE 5.
#endif

#ifndef TEST_SPEED_RANGE
#define TEST_SPEED_RANGE 2.
#endif

#ifndef TEST_PATH_DURATION
#define TEST_PATH_DURATION 5.5
#endif

#ifndef TEST_AIRCRAFT_NUM 
#define TEST_AIRCRAFT_NUM 11
#endif


int main()
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));


    // -------------------- Generate test cases -------------------- //
    double wind_x = 0.;//0.3;
    double wind_y = 0.;//-0.2;
    double min_sep = 1.1;

    constexpr uint N = TEST_AIRCRAFT_NUM;

    std::vector<std::array<Pose3D,N>> list_of_checkpoints;

    std::array<Pose3D,N> vchevron = generate_hchevron<N>(1.5,1.);
    rotate_around_poses(vchevron,Pose3D(0.,0.,0.,-M_PI_2));
    shift_poses(vchevron,Pose3D(0.,0.,0.,0.));

    for(uint i = 0; 3*i < N; i++)
    {
        list_of_checkpoints.push_back(std::array<Pose3D,N>(vchevron));
        shift_poses(vchevron,Pose3D(2*static_cast<double>(N),0.,0.,0.));
        std::rotate(vchevron.begin(),vchevron.begin()+3,vchevron.end());
    }
    

    std::array<AircraftStats,N> stats;
    for(uint i = 0; i < N; i++)
    {
        stats[i].id = i;
        stats[i].airspeed = 1.;
        stats[i].climb = 1.;
        stats[i].turn_radius = 1.;
    }

    std::vector<AircraftStats> stats_vec(stats.cbegin(),stats.cend());

    std::vector<double> delta_t(N-1);
    std::fill(delta_t.begin(),delta_t.end(),0.);

    // -------------------- Test solver -------------------- //

    BasePathsGenerator short_gen =  BasePathsGenerator(1e-3);
    BaseFittedPathsGenerator gen =  BaseFittedPathsGenerator(1e-3);

    auto solver = FleetPlanner(
        short_gen,
        gen,
        1e-3,5.
    );

    for(uint i = 0; i < list_of_checkpoints.size(); i++)
    {
        std::array<Pose3D,N>& starts_arr = list_of_checkpoints[i];
        std::array<Pose3D,N>& ends_arr = list_of_checkpoints[(i+1) % list_of_checkpoints.size()];

        std::vector<Pose3D> starts(starts_arr.cbegin(),starts_arr.cend());
        std::vector<Pose3D> ends(ends_arr.cbegin(),ends_arr.cend());


        ExtraPPResults extra;
        std::vector<Dubins> results;
        solver.solve<Dubins::are_XY_separated,Dubins::compute_XY_distance>(results,extra,starts,ends,stats_vec,min_sep,delta_t,wind_x,wind_y,500);

        if (results.size() != stats_vec.size())
        {
            std::cout << std::endl << "| ***** !! No solution found !! ***** |" << std::endl;
            exit(1);
        }

        std::ofstream output("/home/mael/Programming/DubinsFleetPlanner/" + std::to_string(i) + "_chained.json");
        if (output)
        {
            DubinsPP::OutputPrinter::print_paths_as_ModernJSON(output,results,stats_vec,min_sep,wind_x,wind_y);
            output.close();
        }
        else
        {
            std::cerr << "Could not create output file n° " << i << " ..." << std::endl;
        }
    }
}