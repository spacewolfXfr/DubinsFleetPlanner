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

#ifndef TEST_MIN_TURN_RADIUS
#define TEST_MIN_TURN_RADIUS 0.6
#endif


int main()
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));

    // -------------------- Generate test case -------------------- //

    uint i = 10;

    Pose3D p2_start = generate_pose_2D(5*i+1, TEST_POS_RANGE);
    Pose3D p2_end   = generate_pose_2D(5*i+2, TEST_POS_RANGE);

    Pose3D p1_start = generate_pose_2D(5*i+3, TEST_POS_RANGE);
    Pose3D p1_end   = generate_pose_2D(5*i+4, TEST_POS_RANGE);

    std::default_random_engine gen(5*i); // Some seeded RNG 
    std::uniform_real_distribution<double> dis_speedn(TEST_SPEED_RANGE/10, TEST_SPEED_RANGE);

    double p2_speed = dis_speedn(gen);
    double p2_vspeed = 0.;
    double p1_speed = dis_speedn(gen);
    double p1_vspeed = 0.;

    std::vector<std::unique_ptr<Dubins>> p1_possibilities = list_possible_baseDubins(1.,TEST_MIN_TURN_RADIUS,p1_start,p1_end);


    // -------------------- Plotting --------------------

    Visualisation::Plot2D plot = Visualisation::init_plot();

    // ---------- Main paths ---------- //

    uint d_index = 0;
    for(auto& d : p1_possibilities)
    {
        // Visualisation::plot_path<PLOTTING_SAMPLES>(plot,*d)
        //     .label(d->get_type_abbr())
        //     .lineColor(Visualisation::colorlist[d_index]);

        std::vector<double> junctions = d->get_junction_locs();
        junctions.push_back(d->get_length());

        for(int i = 0; i < junctions.size(); i++)
        {
            double start_l;
            if (i == 0)
            {
                start_l = 0.;
            }
            else
            {
                start_l = junctions[i-1]; 
            }

            double end_l = junctions[i];

            Pose3D start_pos = d->get_position_at_length(start_l);
            Pose3D end_pos   = d->get_position_at_length(end_l);

            DubinsMove curr_type = d->get_section_type_at_length((start_l+end_l)/2);
            switch (curr_type)
            {
            case STRAIGHT:
                Visualisation::plot_path<PLOTTING_SAMPLES,STRAIGHT>(plot,
                    compute_params<STRAIGHT>(start_pos,end_pos,p1_speed,TEST_MIN_TURN_RADIUS,p1_vspeed),
                    (end_l-start_l)/p1_speed)
                        .label(get_DubinsMove_name(curr_type))
                        .lineColor(Visualisation::colorlist[d_index])
                        .dashType(2);
                break;

            case LEFT:
                Visualisation::plot_path<PLOTTING_SAMPLES,LEFT>(plot,
                    compute_params<LEFT>(start_pos,end_pos,p1_speed,TEST_MIN_TURN_RADIUS,p1_vspeed),
                    (end_l-start_l)/p1_speed)
                        .label(get_DubinsMove_name(curr_type))
                        .lineColor(Visualisation::colorlist[d_index])
                        .dashType(2);
                break;

            case RIGHT:
                Visualisation::plot_path<PLOTTING_SAMPLES,RIGHT>(plot,
                    compute_params<RIGHT>(start_pos,end_pos,p1_speed,TEST_MIN_TURN_RADIUS,p1_vspeed),
                    (end_l-start_l)/p1_speed)
                        .label(get_DubinsMove_name(curr_type))
                        .lineColor(Visualisation::colorlist[d_index])
                        .dashType(2);
                break;
            
            default:
                break;
            }
        }

        d_index++;
    }

    // ---------- Display ---------- //

    sciplot::Figure fig     = {{plot}};
    sciplot::Canvas canvas  = {{fig}};
    canvas.size(1920,1080);
    canvas.show();

}