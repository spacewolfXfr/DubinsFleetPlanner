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

#include "plotDubins.hpp"

#include "BaseDubins.hpp"
#include "ExtendedDubins.hpp"

#ifndef PLOTTING_SAMPLES
#define PLOTTING_SAMPLES 200
#endif

int main()
{
    Visualisation::Plot2D plot = Visualisation::init_plot();

    Pose3D start{0.,0.,0.,0.};
    Pose3D end{1.,2.,0.,M_PI/2};

    AircraftStats stats{0,1.,1.,1.};

    double target_l = 15;
    double tol = 1e-6;

    auto candidates = fit_possible_baseDubins(stats,start,end, target_l, tol);
    auto more_candidates = generate_line_extended_base(start,end,stats.climb,stats.turn_radius, target_l, tol,
        {0,0.5,1.});

    for(uint i = 0; i < candidates.size(); i++)
    {
        auto color = Visualisation::get_color(i);
        auto& d = candidates[i];
        if (d->is_valid())
        {
            Visualisation::plot_path<PLOTTING_SAMPLES>(plot,*d)
                .label(d->get_type_abbr() + std::string(" ") + std::to_string(d->get_length()))
                .lineColor(color);
            // Visualisation::plot_junctions(plot,*d)
            //     .label(d->get_type_abbr())
            //     .lineColor(color);
        }
    }

    for(uint i = 0; i < more_candidates.size(); i++)
    {
        auto color = Visualisation::get_color(i);
        auto& d = more_candidates[i];
        if (d->is_valid())
        {
            Visualisation::plot_path<PLOTTING_SAMPLES>(plot,*d)
                .label(d->get_type_abbr() + std::string(" ") + std::to_string(d->get_length()))
                .lineColor(color)
                .lineType(3);
            // Visualisation::plot_junctions(plot,*d)
            //     .label(d->get_type_abbr())
            //     .lineColor(color);
        }
    }

    Visualisation::plot_pose(plot,start).label("Start").lineColor("dark-grey");
    Visualisation::plot_pose(plot,end).label("End").lineColor("black");

    sciplot::Figure fig     = {{plot}};
    sciplot::Canvas canvas  = {{fig}};
    canvas.size(1920,1080);
    canvas.show();

}