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
#include "ConflictDetection.hpp"

#include "randomPathShape.hpp"



#ifndef PLOTTING_SAMPLES
#define PLOTTING_SAMPLES 2000
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

#ifndef TEST_PRECISION
#define TEST_PRECISION DubinsFleetPlanner_PRECISION
#endif


int main()
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));

    // -------------------- Generate test cases -------------------- //

    PathShape<STRAIGHT> s = generate_random_shape<STRAIGHT> (0, TEST_POS_RANGE, TEST_SPEED_RANGE);
    PathShape<LEFT>     l = generate_random_shape<LEFT>     (1, TEST_POS_RANGE, TEST_SPEED_RANGE);
    PathShape<RIGHT>    r = generate_random_shape<RIGHT>    (2, TEST_POS_RANGE, TEST_SPEED_RANGE);
    const AircraftStats test_stats{0, 1., 1., 1., 0., 0., 0.};
    std::cout << "Random shapes generation done" << std::endl;

    std::cout << "Path s speed: " << path_planar_speed(s) << std::endl; 
    std::cout << "Path l speed: " << path_planar_speed(l) << std::endl; 
    std::cout << "Path r speed: " << path_planar_speed(r) << std::endl; 

    // -------------------- Compute groundtruth by sampling -------------------- //

    std::pair<double,double> sampled_sl = sample_temporal_XY_dist<STRAIGHT,LEFT,samples>  (s,l,TEST_PATH_DURATION);
    std::cout << "Sampling SL done" << std::endl;
    Pose3D sampled_sl_pose1 = follow_dubins(s,sampled_sl.first);
    Pose3D sampled_sl_pose2 = follow_dubins(l,sampled_sl.first);

    std::pair<double,double> sampled_sr = sample_temporal_XY_dist<STRAIGHT,RIGHT,samples> (s,r,TEST_PATH_DURATION);
    std::cout << "Sampling SR done" << std::endl;
    Pose3D sampled_sr_pose1 = follow_dubins(s,sampled_sr.first);
    Pose3D sampled_sr_pose2 = follow_dubins(r,sampled_sr.first);

    std::pair<double,double> sampled_lr = sample_temporal_XY_dist<LEFT,RIGHT,samples>     (l,r,TEST_PATH_DURATION);
    std::cout << "Sampling LR done" << std::endl;
    Pose3D sampled_lr_pose1 = follow_dubins(l,sampled_lr.first);
    Pose3D sampled_lr_pose2 = follow_dubins(r,sampled_lr.first);


    // -------------------- Test solver -------------------- //

    std::pair<double,double> test_sl_no_der = temporal_XY_dist<STRAIGHT,LEFT,false>  (s,l,test_stats,test_stats,TEST_PATH_DURATION,TEST_PRECISION);
    Pose3D test_sl_no_der_pose1 = follow_dubins(s,test_sl_no_der.first);
    Pose3D test_sl_no_der_pose2 = follow_dubins(l,test_sl_no_der.first);

    std::pair<double,double> test_sr_no_der = temporal_XY_dist<STRAIGHT,RIGHT,false> (s,r,test_stats,test_stats,TEST_PATH_DURATION,TEST_PRECISION);
    Pose3D test_sr_no_der_pose1 = follow_dubins(s,test_sr_no_der.first);
    Pose3D test_sr_no_der_pose2 = follow_dubins(r,test_sr_no_der.first);

    std::pair<double,double> test_lr_no_der = temporal_XY_dist<LEFT,RIGHT,false>     (l,r,test_stats,test_stats,TEST_PATH_DURATION,TEST_PRECISION);
    Pose3D test_lr_no_der_pose1 = follow_dubins(l,test_lr_no_der.first);
    Pose3D test_lr_no_der_pose2 = follow_dubins(r,test_lr_no_der.first);

    std::cout   << "STRAIGHT-LEFT case    :\nLocations (sampled VS test): " 
                << sampled_sl.first << "  VS  "
                << test_sl_no_der.first << std::endl
                << "Distances: "
                << sampled_sl.second << "  VS  "
                << test_sl_no_der.second << std::endl << std::endl;

    std::cout   << "STRAIGHT-RIGHT case   :\nLocations (sampled VS test): " 
                << sampled_sr.first << "  VS  "
                << test_sr_no_der.first << std::endl
                << "Distances: "
                << sampled_sr.second << "  VS  "
                << test_sr_no_der.second << std::endl << std::endl;

    std::cout   << "LEFT-RIGHT case       :\nLocations (sampled VS test): " 
                << sampled_lr.first << "  VS  "
                << test_lr_no_der.first << std::endl
                << "Distances: "
                << sampled_lr.second << "  VS  "
                << test_lr_no_der.second << std::endl << std::endl;


    // -------------------- Plotting --------------------

    Visualisation::Plot2D plot = Visualisation::init_plot();

    // ---------- Individual trajectories ---------- //

    Visualisation::plot_path<PLOTTING_SAMPLES,STRAIGHT>(plot,s,TEST_PATH_DURATION)
        .label("STRAIGHT Path")
        .lineColor(Visualisation::colorlist[0]);
    Visualisation::plot_pose(plot,initial_pose(s))
        .label("STRAIGHT Init")
        .lineColor(Visualisation::colorlist[0]);


    Visualisation::plot_path<PLOTTING_SAMPLES,LEFT>(plot,l,TEST_PATH_DURATION)
        .label("LEFT Path")
        .lineColor(Visualisation::colorlist[1]);
    Visualisation::plot_pose(plot,initial_pose(l))
        .label("LEFT Init")
        .lineColor(Visualisation::colorlist[1]);


    Visualisation::plot_path<PLOTTING_SAMPLES,RIGHT>(plot,r,TEST_PATH_DURATION)
        .label("RIGHT Path")
        .lineColor(Visualisation::colorlist[2]);
    Visualisation::plot_pose(plot,initial_pose(r))
        .label("RIGHT Init")
        .lineColor(Visualisation::colorlist[2]);

    // ---------- Minimal distance points ---------- //
    
    plot.drawCurve(
        std::valarray{sampled_sl_pose1.x,sampled_sl_pose2.x},
        std::valarray{sampled_sl_pose1.y,sampled_sl_pose2.y})
        .label("STAIGHT-LEFT sampled points")
        .lineColor("black")
        .dashType(2);
        
    plot.drawCurve(
        std::valarray{test_sl_no_der_pose1.x,test_sl_no_der_pose2.x},
        std::valarray{test_sl_no_der_pose1.y,test_sl_no_der_pose2.y})
        .label("STAIGHT-LEFT test points")
        .lineColor("black")
        .dashType(3);


    plot.drawCurve(
        std::valarray{sampled_sr_pose1.x,sampled_sr_pose2.x},
        std::valarray{sampled_sr_pose1.y,sampled_sr_pose2.y})
        .label("STRAIGHT-RIGHT sampled points")
        .lineColor("grey")
        .dashType(2);
        
    plot.drawCurve(
        std::valarray{test_sr_no_der_pose1.x,test_sr_no_der_pose2.x},
        std::valarray{test_sr_no_der_pose1.y,test_sr_no_der_pose2.y})
        .label("STRAIGHT-RIGHT test points")
        .lineColor("grey")
        .dashType(3);


    plot.drawCurve(
        std::valarray{sampled_lr_pose1.x,sampled_lr_pose2.x},
        std::valarray{sampled_lr_pose1.y,sampled_lr_pose2.y})
        .label("LEFT-RIGHT sampled points")
        .lineColor("blue")
        .dashType(2);
        
    plot.drawCurve(
        std::valarray{test_lr_no_der_pose1.x,test_lr_no_der_pose2.x},
        std::valarray{test_lr_no_der_pose1.y,test_lr_no_der_pose2.y})
        .label("LEFT-RIGHT test points")
        .lineColor("blue")
        .dashType(3);

    // ---------- Display ---------- //

    sciplot::Figure fig     = {{plot}};
    sciplot::Canvas canvas  = {{fig}};
    canvas.size(1920,1080);
    canvas.show();

}