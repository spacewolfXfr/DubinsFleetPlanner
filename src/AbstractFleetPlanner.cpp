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

#include "AbstractFleetPlanner.hpp"

// ---------- Helpful for solving ---------- //

double maxmin_dubins_traveltime(
    const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
    const std::vector<AircraftStats>& stats, const std::vector<double>& delta_t,
    double wind_x, double wind_y, bool reallocate
)
{

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(starts.size() <= ends.size());
    assert(starts.size() == stats.size());
    assert(starts.size() == delta_t.size()+1);
#endif

    uint N = starts.size();
    double min_travel_time;
    double delta_sum = 0.;
    for(uint i = 0; i < N; i++)
    {
        const Pose3D& s        = starts[i];
        const AircraftStats& p = stats[i];

        double this_ac_best_traveltime = INFINITY;

        if (reallocate)
        {
            for(const Pose3D& e : ends)
            {
                std::unique_ptr<Dubins> dd = shortest_possible_baseDubins(
                    p.climb,
                    p.turn_radius,
                    s,e,
                    wind_x,wind_y
                );

                this_ac_best_traveltime = std::min(
                    this_ac_best_traveltime,
                    dd->get_length()/p.airspeed
                );
            }
        }
        else
        {
            const Pose3D& e = ends[i];
    
            std::unique_ptr<Dubins> dd = shortest_possible_baseDubins(
                p.climb,
                p.turn_radius,
                s,e,
                wind_x,wind_y
            );

            this_ac_best_traveltime = dd->get_length()/p.airspeed;
        }

        if (i == 0)
        {
            min_travel_time = this_ac_best_traveltime;
        }
        else
        {
            min_travel_time = std::max(
                min_travel_time,
                this_ac_best_traveltime - delta_sum);
            
            delta_sum += delta_t[i-1];
        }
        
    }

    return min_travel_time;
}


std::vector<double> compute_arrival_times(const std::vector<double>& dts, double time_ref)
{
    uint N = dts.size()+1;
    std::vector<double> output(N);

    output[0] = time_ref;
    for(uint i = 1; i < N; i++)
    {
        output[i] = output[i-1] + dts[i-1];
    }

    return output;
}

std::tuple<uint,uint> count_min_number_of_valid_paths(const ListOfPossibilities& all_paths)
{
    uint output = all_paths[0].size();
    uint output_loc = 0;

    uint i = 0;

    for(const std::vector<std::unique_ptr<Dubins>>& v : all_paths)
    {
        uint count = 0;
        for(auto& d : v)
        {
            if (d->is_valid())
            {
                count++;
            }
        }

        if (count < output)
        {
            output = count;
            output_loc = i;
        }

        i++;
    }

    return {output,output_loc};
}

std::set<uint> acs_without_paths(const ListOfPossibilities& all_paths)
{
    std::set<uint> output;
    uint ac_id = 0;
    for(const std::vector<std::unique_ptr<Dubins>>& v : all_paths)
    {
        uint count = 0;
        for(auto& d : v)
        {
            if (d->is_valid())
            {
                count++;
            }
        }

        if (count == 0)
        {
            output.insert(ac_id);
        }

        ac_id++;
    }
    
    return output;
}

std::vector<std::unique_ptr<Dubins>> AbstractFleetPlanner::generate_shortest_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double wind_x, double wind_y) const
    {
        return list_possible_baseDubins(stats.climb,stats.turn_radius,start,end,wind_x,wind_y);
    }


// ---------- Smart sampling ---------- //

void print_samples(const std::list<SampleCase>& l)
{
    for(const SampleCase& c : l)
    {
        std::cout << c.val << " ; ";
    }
    std::cout << std::endl;
}

std::list<SampleCase> generate_linspace(double start, double end, uint samples, bool include_start, bool include_end)
{
    assert(end > start);
    std::list<SampleCase> output;

    double delta = end-start;

    if (include_start && include_end)
    {
        assert(samples >= 2);

        double step = delta/(samples-1);
        for(uint i = 0; i < samples; i++)
        {
            output.push_back(SampleCase{start+i*step});
        }
        return output;
    }
    else if (include_start && !include_end)
    {
        assert(samples >= 1);

        double step = delta/(samples);
        for(uint i = 0; i < samples; i++)
        {
            output.push_back(SampleCase{start+i*step});
        }
        return output;
    }
    else if (!include_start && include_end)
    {
        assert(samples >= 1);

        double step = delta/(samples);
        for(uint i = 1; i <= samples; i++)
        {
            output.push_back(SampleCase{start+i*step});
        }
        return output;
    }
    else // (!include_start && !include_end)
    {
        double step = delta/(samples+1);
        for(uint i = 1; i <= samples; i++)
        {
            output.push_back(SampleCase{start+i*step});
        }
        return output;
    }
}

bool weave_samples(std::list<SampleCase>& l, uint weave, double min_weave_delta)
{
    assert(l.size() > 1);
    // print_samples(l);

    auto next = l.begin(); next++;
    auto iter = l.begin();
    bool output = false;

    while(next != l.end())
    {
        if (next->val - iter->val >= min_weave_delta)
        {
            std::set<uint> intersect;
            std::set_intersection(
                iter->NoPathsACs.begin(),iter->NoPathsACs.end(),
                next->NoPathsACs.begin(),next->NoPathsACs.end(),
                std::inserter(intersect,intersect.begin())
            );

            if (intersect.size() == 0)
            {
                auto new_values = generate_linspace(iter->val,next->val,weave,false,false);
                l.splice(next,new_values);
                output = true;
            }
        }
        iter = next;
        next++;
    }

    return output;
}