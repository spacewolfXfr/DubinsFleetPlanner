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

#include <iostream>

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

#define SQUARE(x) ((x)*(x))

template<DubinsMove m>
static inline void print_PathShape(const PathShape<m>& s)
{
    std::cout   << "Path type: " << get_DubinsMove_name(m) << std::endl
                << "x  : " << s.x    << std::endl
                << "y  : " << s.y    << std::endl
                << "z  : " << s.z    << std::endl
                << "p1 : " << s.p1   << std::endl
                << "p2 : " << s.p2   << std::endl
                << "p3 : " << s.p3   << std::endl
                << "p4 : " << s.p4   << std::endl << std::endl;
}

int main()
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));

    // -------------------- Specific test cases -------------------- //
    
    PathShape<LEFT> t1 = {
        5.3918161949513541,
        9.3388995947284368,
        0.,
        14.656615736110123,
        0.068228574590808014,
        0.,
        -1.1287833647256464
    };

    PathShape<RIGHT> t2 = {
        15.351664777357195,
        4.1009780772792794,
        0.,
        3.3516647773571915,
        -0.29835919354336687,
        0.,
        2.3038346126325147
    };

    double t12_duration = 18.251246765857772;

    auto s1 = t1;
    auto s2 = t2;
    constexpr auto s1_type = LEFT;
    constexpr auto s2_type = RIGHT;
    auto duration = t12_duration;
    auto known_distance = sampled_geometric_XY_dist<s1_type,s2_type,2000>(s1,s2,duration);;

    // -------------------- Generate test cases -------------------- //

// TURN,0.023982,0.558854,4.249035,7.961535,0.259252,TURN,0.693138,0.440454,2.608158,4.389598,0.156868,0.263430

    // double arc1_x       = 0.023982;
    // double arc1_y       = 0.558854;
    // double arc1_start   = 4.249035;
    // double arc1_end     = 7.961535;
    // double arc1_radius  = 0.259252;

    // #define IS_LINE
    // double line1_ax     = 0.544649;
    // double line1_ay     = 0.780315;
    // double line1_bx     = 0.306364;
    // double line1_by     = 0.221958;
    // double line1_len    = sqrt(SQUARE(line1_ax-line1_bx) + SQUARE(line1_ay-line1_by));

    // double arc2_x       = 0.693138;
    // double arc2_y       = 0.440454;
    // double arc2_start   = 2.608158;
    // double arc2_end     = 4.389598;
    // double arc2_radius  = 0.156868;
// 
    // double known_distance = 0.263430;
    // double duration = 5.;
// 
// #ifndef IS_LINE
    // assert(arc1_end > arc1_start);
// #endif
    // assert(arc2_end > arc2_start);

    // -------------------- Compare to groundtruth -------------------- //



// #ifdef IS_LINE

//     PathShape<STRAIGHT> s1{line1_ax,line1_ay,0.,line1_bx-line1_ax,line1_by-line1_ay,0.,0.};
//     path_set_planar_speed(s1,line1_len/duration);

//     Pose3D s1_end = follow_dubins(s1,duration);
//     assert(pose_dist_XY(s1_end,Pose3D(line1_bx,line1_by,0.,0.)) < 1e-6);
// #else

//     PathShape<LEFT>     s1    {arc1_x,arc1_y,0.,arc1_radius,1.,0.,arc1_start};
//     PathShape<RIGHT>    s1_bis{arc1_x,arc1_y,0.,arc1_radius,1.,0.,arc1_end};
//     double s1_length = (arc1_end-arc1_start)*arc1_radius;
//     path_set_planar_speed(s1,s1_length/duration);
//     path_set_planar_speed(s1_bis,s1_length/duration);

//     Pose3D s1_end = follow_dubins(s1,duration);
//     Pose3D s1_bis_end = follow_dubins(s1_bis,duration);

//     assert(pose_dist(s1_end,initial_pose(s1_bis)) < 1e-6);
//     assert(pose_dist(s1_bis_end,initial_pose(s1)) < 1e-6);
// #endif

//     PathShape<LEFT>     s2    {arc2_x,arc2_y,0.,arc2_radius,1.,0.,arc2_start};
//     PathShape<RIGHT>    s2_bis{arc2_x,arc2_y,0.,arc2_radius,1.,0.,arc2_end};
//     double s2_length = (arc2_end-arc2_start)*arc2_radius;
//     path_set_planar_speed(s2,s2_length/duration);
//     path_set_planar_speed(s2_bis,s2_length/duration);

//     Pose3D s2_end = follow_dubins(s2,duration);
//     Pose3D s2_bis_end = follow_dubins(s2_bis,duration);

//     assert(pose_dist(s2_end,initial_pose(s2_bis)) < 1e-6);
//     assert(pose_dist(s2_bis_end,initial_pose(s2)) < 1e-6);



    double distance = std::get<0>(geometric_XY_dist(s1,s2,duration));
    std::cout   << "Known distance   : " << known_distance  << std::endl
                << "Computed distance: " << distance << std::endl;

#ifdef S1_BIS
    double distance_bisstd = std::get<0>(geometric_XY_dist(s1_bis,s2,duration));
    std::cout   << "Bis-std distance : " << distance_bisstd << std::endl;
#endif

#ifdef S2_BIS
    double distance_stdbis = std::get<0>(geometric_XY_dist(s1,s2_bis,duration));
    std::cout   << "Std-bis distance : " << distance_stdbis << std::endl;
#endif

#if defined(S1_BIS) && defined(S2_BIS)
    double distance_bisbis = std::get<0>(geometric_XY_dist(s1_bis,s2_bis,duration));
    std::cout   << "Bis-bis distance : " << distance_bisbis << std::endl;
#endif

    

    // -------------------- Plotting -------------------- //

    Visualisation::Plot2D plot = Visualisation::init_plot();

    // ---------- As geometric objects ---------- //

    // Visualisation::plot_line(plot,line1_ax,line1_ay,line1_bx,line1_by)
    //     .label("Path 1- Geometry")
    //     .lineColor(Visualisation::colorlist[9]);

    // Visualisation::plot_arc(plot,arc1_x,arc1_y,arc1_radius,arc1_start,arc1_end
    //     ,1,Visualisation::colorlist[10]);

    // ---------- As primitive paths ---------- //

    Visualisation::plot_path<PLOTTING_SAMPLES,s1_type>(plot,s1,duration)
        .label("Path 1")
        .lineColor(Visualisation::colorlist[0])
        .dashType(4);

    
    Visualisation::plot_pose(plot,initial_pose(s1))
        .label("Path 1 Init")
        .lineColor(Visualisation::colorlist[0]);

#ifdef S1_BIS
    Visualisation::plot_path<PLOTTING_SAMPLES,s1_bis_type>(plot,s1_bis,duration)
        .label("Path 1bis")
        .lineColor(Visualisation::colorlist[1])
        .dashType(4);

    
    Visualisation::plot_pose(plot,initial_pose(s1_bis))
        .label("Path 1bis Init")
        .lineColor(Visualisation::colorlist[1]);
#endif

    Visualisation::plot_path<PLOTTING_SAMPLES,s2_type>(plot,s2,duration)
        .label("Path 2")
        .lineColor(Visualisation::colorlist[2])
        .dashType(4);

    
    Visualisation::plot_pose(plot,initial_pose(s2))
        .label("Path 2 Init")
        .lineColor(Visualisation::colorlist[2]);

#ifdef S2_BIS
    Visualisation::plot_path<PLOTTING_SAMPLES,s2_bis_type>(plot,s2_bis,duration)
        .label("Path 2bis")
        .lineColor(Visualisation::colorlist[3])
        .dashType(4);

    
    Visualisation::plot_pose(plot,initial_pose(s2_bis))
        .label("Path 2bis Init")
        .lineColor(Visualisation::colorlist[3]);
#endif

    // ---------- Display ---------- //

    sciplot::Figure fig     = {{plot}};
    sciplot::Canvas canvas  = {{fig}};
    canvas.size(1920,1080);
    canvas.show();

}