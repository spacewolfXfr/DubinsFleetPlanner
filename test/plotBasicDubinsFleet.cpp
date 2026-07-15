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
#include "plotDubins.hpp"

#include "FleetPlanner.hpp"
#include "PathGenerators/all.hpp"

#include "ioUtils.hpp"
#include "ExtendedDubins.hpp"


#include "randomPathShape.hpp"

#ifndef PLOTTING_SAMPLES
#define PLOTTING_SAMPLES 500
#endif

#ifndef TEST_PRECISION
#define TEST_PRECISION 1e-3
#endif

#ifndef TEST_MAX_R_LENGTH
#define TEST_MAX_R_LENGTH 10
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
#define TEST_AIRCRAFT_NUM 5
#endif

#ifndef TEST_AIRPORT_RADIUS
#define TEST_AIRPORT_RADIUS 50. // In nautical miles (NM)
#endif

#ifndef TEST_AIRPORT_SPEED
#define TEST_AIRPORT_SPEED 4.166 // 250knots in NM/min
#endif

#ifndef TEST_AIRPORT_TURN
#define TEST_AIRPORT_TURN 1.33 // In NM, equivalent to 360° in 2min at 250knots
#endif

#ifndef TEST_AIRPORT_AIRSEP
#define TEST_AIRPORT_AIRSEP 5 // In NM
#endif

#ifndef TEST_AIRPORT_TIMESEP
#define TEST_AIRPORT_TIMESEP 1.5 // In minutes
#endif


#include "FleetDrawPrimitives.hpp"

int main()
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));


    // -------------------- Generate test cases -------------------- //
    double wind_x = 0.;//0.3;
    double wind_y = 0.2;
    double min_sep = TEST_AIRPORT_AIRSEP;

    constexpr uint N = TEST_AIRCRAFT_NUM;

    std::array<Pose3D,N> random_poses = generate_random<N>(static_cast<double>(N),1.,12);
    std::array<Pose3D,N> hchev_3 = generate_P_chevron<N,N/3>(min_sep*1.1,1.2);
    std::array<Pose3D,N> hchev_3_bis = generate_P_chevron<N,N/3>(min_sep*1.1,1.2);
    std::array<Pose3D,N> circ = generate_circle<N>(1.1*N*min_sep/(2*M_PI));
    std::array<Pose3D,N> hline = generate_hline<N>(min_sep*1.1);
    std::array<Pose3D,N> hline_bis = generate_hline<N>(min_sep*1.1);
    std::array<Pose3D,N> hchevron = generate_hchevron<N>(min_sep*1.1,1.);
    std::array<Pose3D,N> random_disk_inward = generate_random_safe_disk_inward<N>(0.9*TEST_AIRPORT_RADIUS,3*TEST_AIRPORT_RADIUS,
        0.,M_PI_4,TEST_AIRPORT_AIRSEP,
        100000,3);

    std::array<Pose3D,N> airport_ordinal_arrivals = generate_ordinals<N>(TEST_AIRPORT_RADIUS,0.,TEST_AIRPORT_AIRSEP*5);

    std::array<Pose3D,N> airport; airport.fill(Pose3D(0,0,0,M_PI_2));

    shift_poses(hline,Pose3D(0.,0*TEST_AIRPORT_RADIUS,0.,0.));
    shift_poses(hline_bis,Pose3D(0.,TEST_AIRPORT_RADIUS,0.,0.));
    shift_poses(hchevron,Pose3D(0.,TEST_AIRPORT_RADIUS,0.,0.));
    shift_poses(hchev_3,Pose3D(0.,TEST_AIRPORT_RADIUS,0.,0.));
    rotate_around_poses(hchev_3_bis,Pose3D(0.,0.,0.,M_PI_2));
    shift_poses(hchev_3_bis,Pose3D(TEST_AIRPORT_RADIUS,2*TEST_AIRPORT_RADIUS,0.,0.));

    std::array<Pose3D,N> vline = generate_hline<N>(1.5);
    rotate_around_poses(vline,Pose3D(0.,0.,0.,M_PI_2));
    shift_poses(vline,Pose3D(TEST_AIRPORT_RADIUS,2*TEST_AIRPORT_RADIUS,0.,0.));

    std::array<Pose3D,N> vchevron = generate_hchevron<N>(1.5,1.);
    rotate_around_poses(vchevron,Pose3D(0.,0.,0.,-M_PI_2));
    shift_poses(vchevron,Pose3D(TEST_AIRPORT_RADIUS*5,2*TEST_AIRPORT_RADIUS,0.,0.));

    std::array<AircraftStats,N> stats;
    for(uint i = 0; i < N; i++)
    {
        stats[i].id = i;
        stats[i].airspeed = TEST_AIRPORT_SPEED;
        stats[i].climb = 1.;
        stats[i].turn_radius = TEST_AIRPORT_TURN;
    }

    std::vector<AircraftStats> stats_vec(stats.cbegin(),stats.cend());


    std::string test_name("cirlce_to_chev_" + std::to_string(N));
    std::string file_format("json");

    std::array<Pose3D,N> starts = circ;
    std::array<Pose3D,N> ends   = hchevron;

    // Sort by distance to airport
    // auto sort_function = [&](const Pose3D& a, const Pose3D& b)
    // {
    //     return pose_dist_XY(a,airport[0]) <= pose_dist_XY(b,airport[0]);
    // };
    // std::sort(starts.begin(),starts.end(),sort_function);

    std::array<double,N-1> delta_t; 
    delta_t.fill(0.); 
    // delta_t.fill(TEST_AIRPORT_TIMESEP);

    std::vector<Pose3D> starts_vec(starts.cbegin(),starts.cend());
    std::vector<Pose3D> ends_vec(ends.cbegin(),ends.cend());
    std::vector<double> delta_t_vec(delta_t.cbegin(),delta_t.cend());


    // std::cout   << "Start: " << std::endl << pose_to_string(starts[0]) << std::endl
    //             << "End: " << std::endl << pose_to_string(ends[0]) << std::endl;


    // -------------------- Test solver -------------------- //

    BasePathsGenerator          shortest_gen = BasePathsGenerator(TEST_PRECISION);
    BaseFittedPathsGenerator    base_gen     = BaseFittedPathsGenerator(TEST_PRECISION);

    const auto& basic_planner = FleetPlanner(
        shortest_gen,base_gen,
        TEST_PRECISION,TEST_MAX_R_LENGTH
    );

    std::vector<double> extensions_lengths{
        0.,
        // TEST_AIRPORT_SPEED*0.1,
        // TEST_AIRPORT_SPEED*0.5,
        TEST_AIRPORT_SPEED*1.,
        // TEST_AIRPORT_SPEED*2.,
        // TEST_AIRPORT_SPEED*5.
    };


    ExtraPPResults extra;
    std::vector<Dubins> results;

    basic_planner.solve<Dubins::are_XY_separated,Dubins::compute_XY_distance>(results,extra,starts_vec,ends_vec,stats_vec,min_sep,delta_t_vec,wind_x,wind_y);
    
    if (results.size() != stats_vec.size())
    {
        std::cout << std::endl << "| ***** !! No solution found !! ***** |" << std::endl;

        exit(1);
    }

    // -------------------- Plotting --------------------

    Visualisation::Plot2D plot = Visualisation::init_plot();

    // ---------- Individual trajectories ---------- //

    double sampled_min_dist = Visualisation::plot_multiple_paths<PLOTTING_SAMPLES>(plot,results,stats_vec,wind_x,wind_y);

    // ---------- Minimal distance points ---------- //
    
    std::cout   << "Minimal sampled distance : " << sampled_min_dist << std::endl
                << "Minimal required distance: " << min_sep << std::endl;
    
    if (sampled_min_dist < min_sep)
    {
        std::cout << "***** WARNING: Minimal Sampled distance below threshold!!! *****" << std::endl << std::endl;
    }

    // ---------- Display ---------- //

    sciplot::Figure fig     = {{plot}};
    sciplot::Canvas canvas  = {{fig}};
    canvas.size(1920,1080);
    canvas.show();

    // ---------- Output ---------- //

    std::ofstream output("/home/mael/Programming/DubinsFleetPlanner/" + test_name + "." + file_format);
    if (output)
    {
        if (file_format == "csv")
        {
            DubinsPP::OutputPrinter::print_paths_as_CSV(output,results,stats_vec,wind_x,wind_y,PLOTTING_SAMPLES);
        }
        else if (file_format == "json")
        {
            DubinsPP::OutputPrinter::print_paths_as_ModernJSON(output,results,stats_vec,sampled_min_dist,wind_x,wind_y);
        }
        else
        {
            std::cerr << "Unknown file format: " << file_format << std::endl << "Nothing written..." << std::endl;
        }

        output.close();
    }
    else
    {
        std::cerr << "Could not create output file..." << std::endl;
    }

    

}