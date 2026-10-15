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

#include <sciplot/sciplot.hpp>

#include <format>
#include <tuple>
#include <valarray>
#include <map>

#include "utils.hpp"
#include "Primitives.hpp"
#include "Dubins.hpp"
#include "BaseDubins.hpp"


// Colormap from https://tsitsul.in/blog/coloropt/
// xgfs_normal12 = [(235, 172, 35), (184, 0, 88), (0, 140, 249), (0, 110, 0), (0, 187, 173), (209, 99, 230), (178, 69, 2), (255, 146, 135), (89, 84, 214), (0, 198, 248), (135, 133, 0), (0, 167, 108), (189, 189, 189)]
// xgfs_normal12_hex = {#ebac23,#b80058,#008cf9,#006e00,#00bbad,#d163e6,#b24502,#ff9287,#5954d6,#00c6f8,#878500,#00a76c,#bdbdbd}

namespace Visualisation
{

    const std::vector<std::string> colorlist{
        "#EBAC23",
        "#B80058",
        "#008CF9",
        "#006E00",
        "#00BBAD",
        "#D163E6",
        "#B24502",
        "#FF9287",
        "#5954d6",
        "#00c6f8",
        "#878500",
        "#00a76c",
        "#bdbdbd"
    };

    inline const std::string& get_color(uint i)
    {
        return colorlist[i % colorlist.size()];
    }


    using namespace sciplot;

    DrawSpecs& plot_pose(Plot2D& plot, const Pose3D& pose)
    {
        return plot.drawWithVecs("vectors",
            std::valarray<double>{pose.x},
            std::valarray<double>{pose.y},
            std::valarray<double>{std::cos(pose.theta)},
            std::valarray<double>{std::sin(pose.theta)});
    }

    template<unsigned samples>
    DrawSpecs& plot_path(Plot2D& plot, const Dubins& path)
    {
        static_assert(samples > 1);
        
        std::valarray<double> xs(samples),ys(samples);
        double len = path.get_length();
        auto lens = linspace(0,len,samples);
        std::vector<double> lens_vec{std::begin(lens),std::end(lens)};
        std::vector<Pose3D> poses = path.get_positions_at_lengths(lens_vec,true);

        for(unsigned i = 0; i < samples; i++)
        {
            xs[i] = poses[i].x;
            ys[i] = poses[i].y;
        }

        return plot.drawCurve(xs,ys);
    }

    template<unsigned samples>
    DrawSpecs& plot_path(Plot2D& plot, const Dubins& path, const AircraftStats& stats, double wind_x, double wind_y)
    {
        static_assert(samples > 1);
        
        std::valarray<double> xs(samples),ys(samples);
        double duration = path.get_duration();
        auto times = linspace(0,duration,samples);
        std::vector<double> time_vec{std::begin(times),std::end(times)};
        std::vector<Pose3D> poses = path.get_positions_at_times(time_vec,true);

        for(unsigned i = 0; i < samples; i++)
        {
            xs[i] = poses[i].x + wind_x*time_vec[i];
            ys[i] = poses[i].y + wind_y*time_vec[i];
        }

        return plot.drawCurve(xs,ys);
    }

    template<unsigned samples, DubinsMove m>
    DrawSpecs& plot_path(Plot2D& plot, const PathShape<m>& path, double duration)
    {
        static_assert(samples > 1);

        std::valarray<double> xs(samples),ys(samples);

        for(unsigned i = 0; i < samples; i++)
        {
            Pose3D pose = follow_dubins(path,i*duration/(samples-1));
            xs[i] = pose.x;
            ys[i] = pose.y;
        }

        return plot.drawCurve(xs,ys);
    }

    DrawSpecs& plot_line(Plot2D& plot, double ax, double ay, double bx, double by)
    {
        std::valarray<double> xs(2),ys(2);
        xs[0] = ax;
        xs[1] = bx;
        ys[0] = ay;
        ys[1] = by;

        return plot.drawCurveWithPoints(xs,ys);
    }

    void plot_arc(Plot2D& plot, double cx, double cy, double radius, double rad_init, double rad_end,
        uint obj_id=0, std::string color="black")
    {
        double deg_init = rad_init*180/M_PI;
        double deg_end  = rad_end*180/M_PI;

        plot.gnuplot(std::string("set obj ") + std::to_string(obj_id) + std::string(" circle at ") + 
            std::to_string(cx) + std::string(",") + std::to_string(cy) +
            std::string(" size ") + std::to_string(radius) + std::string(" arc [") +
            std::to_string(deg_init) + std::string(":") + std::to_string(deg_end) + std::string("] fc \"") +
            color + std::string("\" nowedge"));

        // set obj 2 circle at  4,60  size 3 arc [0:180] fc "web-green" nowedge

    }

    DrawSpecs& plot_junctions(Plot2D& plot, Dubins& path)
    {
        std::vector<Pose3D> locs = path.get_junction_points();
        size_t num = locs.size();
        std::valarray<double> xs(num), ys(num);
        for(unsigned i = 0; i < num; i++)
        {
            xs[i] = locs[i].x;
            ys[i] = locs[i].y;
        }

        return plot.drawPoints(xs,ys);
    }

    template<uint samples>
    double plot_multiple_paths(Plot2D& plot, 
        const std::vector<Dubins>& dubins, const std::vector<AircraftStats>& stats,
        double wind_x, double wind_y)
    {
        static_assert(samples > 1);
        assert(dubins.size() == stats.size());
        uint N = dubins.size();

        double max_time = 0.;

        // Plot all paths
        for(uint i = 0; i < N; i++)
        {
            const auto& d = dubins[i];
            max_time = std::max(max_time, d.get_duration());
            Visualisation::plot_path<samples>(plot,d, stats[i], wind_x, wind_y)
                .label(std::to_string(i) + std::string(": ") + d.get_type_abbr() + std::format(" ( {:.3f} )",d.get_length()))
                .lineColor(Visualisation::get_color(i));

            Visualisation::plot_pose(plot,d.get_start())
                .labelNone()//.label(std::to_string(i) + std::string(": Start"))
                .lineColor(Visualisation::get_color(i));

            Pose3D airref_end = d.get_end();
            airref_end.x += wind_x*d.get_duration();
            airref_end.y += wind_y*d.get_duration();
            Visualisation::plot_pose(plot,airref_end)
                .labelNone()//.label(std::to_string(i) + std::string(": End"))
                .lineColor(Visualisation::get_color(i));
        }

        // Sample for minimal distance
        double timestep = max_time/(samples-1);
        double min_dist = INFINITY;
        double min_dist_time;
        uint ac_id_1,ac_id_2;
        for(uint i = 0; i < samples; i++)
        {

            double ctime = i*timestep;
            std::vector<Pose3D> poses(N);
            for(uint j = 0; j < N; j++)
            {
                if (ctime > dubins[j].get_duration())
                {
                    poses[j] = UndefPose;
                }
                else
                {
                    poses[j] = dubins[j].get_position_at_time(ctime);
                }
            }

            std::tuple<uint,uint,double> min_dist_loc = min_vec_poses_dist_XY(poses);

            double curr_min_dist = std::get<2>(min_dist_loc);

            if (min_dist > curr_min_dist)
            {
                min_dist = curr_min_dist;
                min_dist_time = ctime;
                ac_id_1 = std::get<0>(min_dist_loc);
                ac_id_2 = std::get<1>(min_dist_loc);
            }
        }


        Pose3D p1 = dubins[ac_id_1].get_position_at_time(min_dist_time);
        Pose3D p2 = dubins[ac_id_2].get_position_at_time(min_dist_time);

        Visualisation::plot_line(plot,
                    p1.x+wind_x*min_dist_time,
                    p1.y+wind_y*min_dist_time,
                    p2.x+wind_x*min_dist_time,
                    p2.y+wind_y*min_dist_time)
                .dashType(3)
                .lineColor("black")
                .pointType(13)
                .pointSize(1.2)
                .label("Min distance: " + std::to_string(min_dist) + " ( " + std::to_string(ac_id_1) + " , " +std::to_string(ac_id_2) + " )");

        assert(pose_dist_XY(p1,p2) == min_dist);

        return min_dist;
    }

    Plot2D init_plot()
    {
        // Create a Plot object
        Plot2D plot;

        // Set the x and y labels
        plot.xlabel("x");
        plot.ylabel("y");

        // // Set x and y ranges
        // plot.xrange(xmin,xmax); 
        // plot.yrange("","*"); // Autorange
        plot.gnuplot("set size ratio -1"); // Equal aspect ratio

        // Set the legend to be on the bottom along the horizontal
        plot.legend()
            .atOutsideBottom()
            .displayHorizontal();

        return plot;
    }

}