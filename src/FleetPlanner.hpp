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

#include <vector>
#include <tuple>
#include <list>
#include <set>
#include <functional>

#include <iostream>
#include <memory>

#include <boost/chrono.hpp>

#include "PathGenerator.hpp"
#include "ConflictList.hpp"
#include "ioUtils.hpp"
#include "Pose3D.hpp"

/*************************************************************************************/
/*************************************************************************************/

/**                                     Helpers                                     **/

/*************************************************************************************/
/*************************************************************************************/

// ---------- Imports and typedefs ---------- //

namespace chrono = boost::chrono;

namespace oprint = DubinsPP::OutputPrinter;

// ---------- Metadata processing ---------- //

class ExtraPPResults
{
public:
    std::string case_name;
    bool success = false;
    bool false_positive = false;
    uint iterations = 0;
    chrono::nanoseconds duration;
    uint threads = 0;
    uint possible_paths_num = 0;
    double initial_path_time = 0;
    double final_path_time = 0;
    double worst_improvement_rate = 0;

    // Stdout-friendly formatting of the results
    std::string format() const;

    static const std::string CSV_header()
    {
        return "Test input;Success;False positive;Iterations;Duration(ns);Threads;Possible paths;Initial guessed time;Final obtained time;Worst rate(u/s)";
    }

    // CSV row format
    std::string as_CSV() const;
};

// ---------- Helpful for solving ---------- //

/**
 * @brief For each aircraft, compute the shortest path. Then return the maximal time of all minima. 
 * 
 * @param starts    Starting poses
 * @param ends      Ending poses
 * @param stats     Stats for each aircraft
 * @param delta_t   Arrival time differences
 * @param wind_x    Wind, x component
 * @param wind_y    Wind, y component
 * @param reallocate If true, each aircraft will seek the shortest path to any end, instead of its allocated one.
 * @return double   Max of min travel times
 */
double maxmin_dubins_traveltime(
    const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
    const std::vector<AircraftStats>& stats, const std::vector<double>& delta_t,
    double wind_x, double wind_y, bool reallocate
);

std::vector<double> compute_arrival_times(const std::vector<double>& dts, double time_ref);

/**
 * @brief Return one of the aircraft with the least amount of possible paths
 * 
 * @param all_paths List of all paths per aircraft
 * @return std::tuple<uint,uint> (number,ac_id) for an aircraft with minimal amount of available paths
 */
std::tuple<uint,uint> count_min_number_of_valid_paths(const ListOfPossibilities& all_paths);

/**
 * @brief Return the set of aircraft which do not have a possible path
 * 
 * @param all_paths List of all paths per aircraft
 * @return std::set<uint> set of ac_id, for which every individual has no path available
 */
std::set<uint> acs_without_paths(const ListOfPossibilities& all_paths);

/**
 * @brief Remove from the given list of paths the ones colliding with the given set of obstacle paths
 * 
 * @tparam distance_function    Function computing the distance between two path (usually underline the separation one)
 * @param all_paths             Candidate paths to be filtered
 * @param stats                 Aircraft stats associated to candidate paths
 * @param obstacle_paths        Obstacle paths
 * @param obstacle_stats        Stats of obstacle aircraft paths
 * @param obstacle_delays       Delays of obstacle aircraft paths
 * @param min_sep               Minimal separation desired (non-negative)
 */
template<Dubins::DubinsDistanceFunction distance_function>
void filter_colliding_paths(ListOfPossibilities& all_paths, const std::vector<AircraftStats> &stats,
    const std::vector<Dubins>& obstacle_paths, const std::vector<AircraftStats>& obstacle_stats,
    const std::vector<double>& obstacle_delays,
    double min_sep)
{
    for(uint i = 0; i < stats.size(); i++)
    {
        const AircraftStats& s = stats[i];
        std::vector<std::unique_ptr<Dubins>>& path_list = all_paths[i];
        std::vector<std::unique_ptr<Dubins>> filtered_paths;
        
        for(std::unique_ptr<Dubins>& p_ptr : path_list)
        {
            bool conflict_free = true;

            for(uint j = 0; j < obstacle_stats.size(); j++)
            {
                const Dubins& p = *p_ptr;
                const Dubins& c = obstacle_paths[j];
                const double delay = obstacle_delays[j];
                const AircraftStats& cs = obstacle_stats[j];
                
                std::pair<double,double> res;

                if (delay > DubinsFleetPlanner_PRECISION)
                {
                    if (delay > p.get_duration())
                    {
                        continue;
                    }
                    const Dubins& p_followed = p.follow_for_duration(delay);
                    res = generic_compute_distance<distance_function>(p_followed,c,min_sep);
                }
                else
                {
                    res = generic_compute_distance<distance_function>(p,c,min_sep);
                }

                if (res.second < min_sep)
                {
                    conflict_free = false;
                    break;
                }
            }

            if (conflict_free)
            {
                filtered_paths.push_back(std::move(p_ptr));
            }
        }

        all_paths[i] = std::move(filtered_paths);
    }
}

/**
 * @brief Remove from the list of possibilities the paths which get geometrically close to the given obstacles.
 * DOES NOT CONSIDER VERTICALITY.
 * 
 * @param all_paths Candidate paths to be filtered
 * @param stats Aircraft stats associated to candidate paths
 * @param obstacles Obstacle elements
 * @param min_sep   Minimal separation desired (non-negative)
 * @param wind_x    X-component of the wind vector
 * @param wind_y    Y-component of the wind vector
 */
void filter_2D_geometric_colliding_paths(ListOfPossibilities& all_paths, const std::vector<AircraftStats>& stats, const std::vector<DynamicPathShape>& obstacles, double min_sep, double wind_x = 0., double wind_y = 0.);

// ---------- Smart sampling ---------- //

struct SampleCase
{
    double val;
    bool done       = false;
    bool success    = false;
    std::set<uint> NoPathsACs;
};

void print_samples(const std::list<SampleCase>& l);

/**
 * @brief Generates a list of sample cases evenly spaced between start and end
 * 
 * @param start Starting value
 * @param end   Ending value
 * @param samples Number of samples to generate
 * @param include_start If true, include the start value in the list of samples. Default is true.
 * @param include_end   If true, include the end value in the list of samples. Default is true.
 * @return std::list<SampleCase> 
 */
std::list<SampleCase> generate_linspace(double start, double end, uint samples, bool include_start = true, bool include_end = true);

/**
 * @brief Assuming the input is sorted (by val), weave points, that is add points between the existing ones. 
 * Returns whether points have been successfully added or not.
 * 
 * @param q     List of SampleCase, sorted by val
 * @param weave Number of points to add between each two samples
 * @param min_weave_delta   Minimal value between two iterations for weaving between them.
 * @return true     New points have been added
 * @return false    No new samples
 */
bool weave_samples(std::list<SampleCase>& l, uint weave, double min_weave_delta);

class FleetPlanner 
{
protected:

    /*******************************************************************************/
    /*                                                                             */
    /*                              Solver attributes                              */
    /*                                                                             */
    /*******************************************************************************/

    double precision_tol;               // Precision when looking for collisions
    double maximal_relative_duration;   // Maximum distance (as a multiplicative factor from the lowest possible distance)
    mutable bool first_log = true;      // When logging, state whether it is the first log or not
    FittedPathsGenerator& path_gen;
    PathsGenerator& shortest_path_gen;
    std::vector<std::reference_wrapper<const FittedPathsGenerator>> path_gens_by_ac = {};
    std::vector<std::reference_wrapper<const PathsGenerator>> shortest_path_gens_by_ac = {};

public:
    FleetPlanner(PathsGenerator &_shortest_path_gen, FittedPathsGenerator &_path_gen, double _prec, double _max_r_dur, int verb = DubinsFleetPlanner_VERBOSE, std::ostream& output_stream = std::cout) : 
        shortest_path_gen(_shortest_path_gen), path_gen(_path_gen),
        precision_tol(_prec), maximal_relative_duration(_max_r_dur), verbosity(verb), datalog(output_stream) {}

    void set_aircraft_generators(
        const std::vector<std::reference_wrapper<const PathsGenerator>>& shortest_gens,
        const std::vector<std::reference_wrapper<const FittedPathsGenerator>>& fitted_gens)
    {
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(shortest_gens.size() == fitted_gens.size());
#endif
        shortest_path_gens_by_ac = shortest_gens;
        path_gens_by_ac = fitted_gens;
    }

    std::string my_name() const
    {
        return path_gen.my_name();
    }

    /*******************************************************************************/
    /*                                                                             */
    /*                               Path generation                               */
    /*                                                                             */
    /*******************************************************************************/

public:
    /**
     * @brief Function generating possible paths given a situation, relying on provided path generator
     * 
     * @param start     Starting pose
     * @param end       Ending pose 
     * @param stats     Aircraft statistics
     * @param target_len    Length of the paths to generate
     * @return std::vector<std::unique_ptr<Dubins>> List of possible paths with the given length
     */
    std::vector<std::unique_ptr<Dubins>> generate_possible_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
    {
        return path_gen.generate_paths(start,end,stats,target_len);
    }

    std::vector<std::unique_ptr<Dubins>> generate_possible_paths(
        uint ac_index,
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
    {
        if (path_gens_by_ac.size() > ac_index)
        {
            return path_gens_by_ac[ac_index].get().generate_paths(start,end,stats,target_len);
        }
        return generate_possible_paths(start,end,stats,target_len);
    }

    /**
     * @brief Maximum number of different paths returned by `generate_possible_paths` 
     * 
     * @return uint 
     */
    uint max_path_num() const
    {
        if (path_gens_by_ac.size() > 0)
        {
            uint output = 0;
            for(const auto& gen_ref : path_gens_by_ac)
            {
                output = std::max(output,gen_ref.get().max_path_num());
            }
            return output;
        }
        return path_gen.max_path_num();
    } 

    /**
     * @brief Generate the list of shortest paths. It is a list as it considers the shortest path *for each type of path*
     * 
     * @param start     Starting pose
     * @param end       Ending pose 
     * @param stats     Aircraft statistics
     * @param wind_x    Wind, X component (TODO: integrate properly)
     * @param wind_y    Wind, Y component (TODO: integrate properly)
     * @return std::vector<std::unique_ptr<Dubins>> 
     */
    std::vector<std::unique_ptr<Dubins>> generate_shortest_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double wind_x, double wind_y) const
    {
        return shortest_path_gen.generate_paths(start,end,stats);
    }

    std::vector<std::unique_ptr<Dubins>> generate_shortest_paths(
        uint ac_index,
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double wind_x, double wind_y) const
    {
        if (shortest_path_gens_by_ac.size() > ac_index)
        {
            return shortest_path_gens_by_ac[ac_index].get().generate_paths(start,end,stats);
        }
        return generate_shortest_paths(start,end,stats,wind_x,wind_y);
    }


    /**********************************************************************************/
    /*                                                                                */
    /*          Find a collision-free solution given a list of possibilities          */
    /*                                                                                */
    /**********************************************************************************/

    /******************** Mono-thread solution finding ********************/

protected:
    /**
     * @brief Recursively look for a valid combination of paths
     * 
     * This is akin to a depth-first search in the tree of possible combinations, with 'naive' branch-and-bound (i.e. there no special heuristic
     * for selecting better branches)
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @param all_paths List of all possible path for each agent
     * @param stats     Statistics of each agent
     * @param durations Trip duration for each aircraft
     * @param min_sep   Minimal distance required
     * @param current_index Index of the aircraft currently being studied
     * @param choices   Array of selected paths; considered filled for i=0 to current_index (excluded)
     * @param conflicts_memo Hashmap storing known conflict values
     * @return true A valid combination has been found, as is stored in 'choices'
     * @return false No valid combination found
     */
    template<Dubins::DubinsSeparationFunction separation_function>
    bool recursive_pathfinder(const ListOfPossibilities& all_paths, const std::vector<AircraftStats>& stats,
        double min_sep, uint current_index, 
        std::vector<int>& choices, std::map<Conflict_T,bool>& conflicts_memo) const
    {
    #if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(all_paths.size() == choices.size());
        assert(all_paths.size() == stats.size());
    #endif
        uint N = all_paths.size();

        // Edge case handling
        if (current_index >= N)
        {
            return true;
        }

        for(uint i = 0; i < all_paths[current_index].size(); i++)
        {
            // Take a possible path
            auto& dubin = all_paths[current_index][i];
            
            // if it is not valid, skip it
            if (!dubin->is_valid())
            {
                continue;
            }
            else // Otherwise, test distances against the ones already set
            {
                bool conflict_with_existing = false;

                for(uint past_index = 0; past_index < current_index; past_index++)
                {
                    // Check if there is a conflict with a set path
                    auto& other = all_paths[past_index][choices[past_index]];

                    // Look first in the memo dictionnary
                    Conflict_T ccase = std::make_tuple(
                        current_index,i,
                        past_index,choices[past_index]);

                    
                    auto case_it = conflicts_memo.find(ccase);
                    if (case_it == conflicts_memo.end()) // If not found, compute and store
                    {
                        double duration = std::min(
                            dubin->get_duration(),
                            other->get_duration());

                        conflict_with_existing = !separation_function(*dubin,*other,duration,min_sep,precision_tol,DubinsDistDefaultRec);
                        conflicts_memo.insert({ccase,conflict_with_existing});
                    }
                    else    // Otherwise, simply retrieve the value
                    {
                        conflict_with_existing = case_it->second;
                    }

                    // If there is, stop trying with this path
                    if (conflict_with_existing)
                    {
                        break;
                    }
                }

                // Found a conflict between set paths and current; try the next one
                if (conflict_with_existing)
                {
                    continue;
                }
                else // If not, set it and go down in recursion
                {
                    choices[current_index] = i;
                    bool successful_attribution = recursive_pathfinder<separation_function>(
                        all_paths,stats,min_sep,current_index+1,choices,conflicts_memo);

                    // Early return
                    if (successful_attribution)
                    {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    /**
     * @brief Recursively look for a valid combination of paths
     * 
     * This is an entry point for the 'recursive_pathfinder' method
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @param all_paths List of all possible path for each agent
     * @param stats     Statistics of each agent
     * @param min_sep   Minimal distance required
     * @return std::vector<Dubins>
     */
    template<Dubins::DubinsSeparationFunction separation_function>
    std::vector<Dubins> pathfinder_separated(
        ListOfPossibilities& all_paths, const std::vector<AircraftStats>& stats, double min_sep) const
    {
        uint current_index = 0;
        std::vector<int> choices(stats.size());
        std::map<Conflict_T,bool> conflicts_memo;

        bool success = recursive_pathfinder<separation_function>(all_paths,stats,min_sep,current_index,choices,conflicts_memo);
        if (success)
        {
            std::vector<Dubins> output(all_paths.size());

            for(uint i = 0; i < all_paths.size(); i++)
            {
                output[i] = *all_paths[i][choices[i]];
            }

            return output;
        }
        else
        {
            return {};
        }
    }

    /******************** Multi-thread solution finding ********************/




    /******************** Standardized entry points ********************/


    /**
     * @brief Generic function for monothread solution finding (may be overloaded)
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @param all_paths List of all possible path for each agent
     * @param stats     Statistics of each agent
     * @param min_sep   Minimal distance required
     * @return std::vector<Dubins>
     */
    template<Dubins::DubinsSeparationFunction separation_function>
    std::vector<Dubins> find_solution(
        ListOfPossibilities& possibilites, const std::vector<AircraftStats>& stats, double min_sep) const
    {
        return pathfinder_separated<separation_function>(possibilites,stats,min_sep);
    }

    /**
     * @brief Generic function for multithread solution finding (may be overloaded)
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @tparam distance_function    Function computing the distance between two path (usually underline the separation one)
     * @param all_paths List of all possible path for each agent
     * @param stats     Statistics of each agent
     * @param min_sep   Minimal distance required
     * @param threads   Number of threads to use
     * @return std::vector<Dubins>
     */
    template<Dubins::DubinsSeparationFunction separation_function, Dubins::DubinsDistanceFunction distance_function>
    std::vector<Dubins> find_solution_parallel(
        ListOfPossibilities& possibilites, const std::vector<AircraftStats>& stats, double min_sep, uint max_pathnum, 
        uint threads, bool duration_based_cost,
        bool list_conflicts=true, double timetag=0.) const
    {
        SharedListOfPossibilities shared_list;
        for(auto& vd: possibilites)
        {
            shared_list.push_back(make_shared(vd));
        }
        
        std::vector<Conflict_T> conflicts;

        // If the minimum separation is too small, assume it is null and return no conflicts
        if (min_sep < DubinsFleetPlanner_PRECISION) 
        {
            return find_pathplanning_LP_solution(shared_list,stats,conflicts,max_pathnum,threads,duration_based_cost,verbosity);
        } 

        if (list_conflicts)
        {
            std::vector<RichConflict_T> r_conflicts = generic_parallel_compute_distances<distance_function>(
                threads,shared_list,stats,min_sep,
                Conflict_Map_T(),true);

            if (verbosity >= DubinsFleetPlanner_VERYVERY_VERBOSE)
            {
                oprint::append_rich_conflicts(datalog,timetag,r_conflicts,shared_list,stats,!first_log);
                first_log = false;
            }
            
            // std::cout << "Going from " << r_conflicts.size() << " rich conflicts to ";


            conflicts = drop_conflict_details(r_conflicts,min_sep);

            // std::cout << conflicts.size() << " std ones " << std::endl;

            
        }
        else
        {
            conflicts = generic_parallel_compute_separations<separation_function>(threads,shared_list,stats,min_sep);
        }

        return find_pathplanning_LP_solution(shared_list,stats,conflicts,max_pathnum,threads,duration_based_cost,verbosity);
    }

    /**
     * @brief List all possible paths for a fleet given their arrival times
     * 
     * @param starts    Starting poses
     * @param ends      Ending poses (ground referential)
     * @param stats     Aircraft statistics
     * @param times     Time of arrival for each aircraft
     * @param wind_x    Wind, X component
     * @param wind_y    Wind, Y component
     * @return ListOfPossibilities List of possible paths per aircraft
     */
    ListOfPossibilities list_all_possibilities(
        const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
        const std::vector<AircraftStats>& stats, const std::vector<double>& times, double wind_x, double wind_y) const
    {
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(starts.size() == ends.size());
        assert(starts.size() == stats.size());
        assert(starts.size() == times.size());
#endif
        uint N = starts.size();

        ListOfPossibilities output(N);

        for(uint i = 0; i < N; i++)
        {
            // Modify target according to estimated travel time
            Pose3D base_end = ends[i];
            double target_time = times[i];
            base_end.x -= target_time*wind_x;
            base_end.y -= target_time*wind_y;
            double target_len = target_time*(stats[i].airspeed+stats[i].target_airspeed)/2.0;

            // List all possibilities for aircraft i
            output[i] = generate_possible_paths(i,starts[i],base_end,stats[i],target_len);
            for(const auto& el : output[i])
            {
                if (el->is_valid())
                {
                    assert(abs(el->get_length() - target_len) < DubinsFleetPlanner_PRECISION);
                    assert(abs(el->get_duration() - target_time) < DubinsFleetPlanner_PRECISION);
                }
            }
        }

        return output;
    }

    /**
     * @brief List all possible paths for a fleet given their possible travel times.
     * 
     * @param starts    Starting poses
     * @param ends      Ending poses (ground referential)
     * @param stats     Aircraft statistics
     * @param timeslots Several possible times of arrival for each aircraft
     * @param wind_x    Wind, X component
     * @param wind_y    Wind, Y component
     * @param allow_shortest_paths If true, adds as possible paths the shortest ones (durations will most likely be out of the timeslots).
     * @return ListOfPossibilities List of possible paths per aircraft
     */
    ListOfPossibilities list_all_possibilities(
        const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
        const std::vector<AircraftStats>& stats, const std::vector<std::vector<double>>& timeslots, double wind_x, double wind_y) const
    {
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(starts.size() == ends.size());
        assert(starts.size() == stats.size());
        assert(starts.size() == timeslots.size());
#endif
        uint N = starts.size();

        ListOfPossibilities output(N);

        for(uint i = 0; i < N; i++)
        {
            
            Pose3D base_end = ends[i];
            std::vector<double> possible_times = timeslots[i];
            for(double target_time : possible_times)
            {
                // Modify target according to estimated travel time
                base_end.x -= target_time*wind_x;
                base_end.y -= target_time*wind_y;
            double target_len = target_time*(stats[i].airspeed+stats[i].target_airspeed)/2.0;
                // Add possibilities to aircraft i
                auto possibilities = generate_possible_paths(i,starts[i],base_end,stats[i],target_len);

                output[i].insert(output[i].end(),
                    std::make_move_iterator(possibilities.begin()),
                    std::make_move_iterator(possibilities.end()));
            }
        }

        return output;
    }

    /**
     * @brief List all possible paths for a fleet given their arrival times, but allow aircraft to reach any arrival point.
     * 
     * Two aircraft will never be able to select the same arrival point, since arriving simultaneously will
     * create a collision.
     * 
     * @param starts    Starting poses
     * @param ends      Ending poses (ground referential)
     * @param stats     Aircraft statistics
     * @param times     Time of arrival for each aircraft
     * @param wind_x    Wind, X component
     * @param wind_y    Wind, Y component
     * @return ListOfPossibilities List of possible paths per aircraft
     */
    ListOfPossibilities list_all_possibilities_with_reallocation(
        const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
        const std::vector<AircraftStats>& stats, const std::vector<double>& times, double wind_x, double wind_y) const
    {
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(starts.size() <= ends.size());
        assert(starts.size() == stats.size());
#endif
        uint N = starts.size();

        ListOfPossibilities output(N);

        for(uint i = 0; i < N; i++)
        {
            double target_time = times[i];
            for(Pose3D end : ends)
            {
                // Modify target according to estimated travel time
                end.x -= target_time*wind_x;
                end.y -= target_time*wind_y;
            double target_len = target_time*(stats[i].airspeed+stats[i].target_airspeed)/2.0;
                // Add possibilities to aircraft i
                auto possibilities = generate_possible_paths(i,starts[i],end,stats[i],target_len);

                output[i].insert(output[i].end(),
                    std::make_move_iterator(possibilities.begin()),
                    std::make_move_iterator(possibilities.end()));
                
            }
        }

        return output;
    }


public:
    int verbosity;
    std::ostream& datalog;

    /**
     * @brief  Solve the pathfinding problem for given target times
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @tparam distance_function    Function computing the distance between two path (usually underline the separation one)
     * @param res       A reference for the solution (if it exists)
     * @param extra     Extra info regarding the solving process
     * @param starts    List of starting poses 
     * @param ends      List of ending poses
     * @param stats     Statistics for aircraft
     * @param min_sep   Minimal required separation
     * @param times     Target time for each aircraft
     * @param wind_x    X component of the wind vector. Default to no wind.
     * @param wind_y    Y component of the wind vector. Default to no wind.
     * @param reallocate If true, allow aircraft to swap endpoints. Default to false.
     * @param threads   Number of threads to use for solving. 
     *                      If nonpositive, disable threading
     *                      If 1, set the number of threads to std::thread::hardware_concurrency()
     *                      Otherwise, use the given value
     * @param obstacle_paths Paths acting as obstacles (representing other aircraft)
     * @param obstacle_stats Flight characteristics of the obstacle aircraft
     * @param obstacle_delays Delays before which each obstacle path is active (same size as obstacle_paths)
     * @param geometric_obstacles  Geometric obstacles to avoid (2D only)
     * @return std::set<uint> The set of aircraft IDs with no possible paths
     */
    template<Dubins::DubinsSeparationFunction separation_function, Dubins::DubinsDistanceFunction distance_function>
    std::set<uint> solve_once(
        std::vector<Dubins>& res,
        ExtraPPResults& extra,
        const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
        const std::vector<AircraftStats>& stats, double min_sep,
        const std::vector<double>& times,
        double wind_x = 0., double wind_y = 0.,
        bool reallocate = false,
        int threads = 0,
        const std::vector<Dubins>& obstacle_paths = {}, const std::vector<AircraftStats>& obstacle_stats = {},
        const std::vector<double>& obstacle_delays = {},
        const std::vector<DynamicPathShape>& geometric_obstacles = {}
    ) const
    {
        // Timing
        chrono::thread_clock clk;
        auto start = clk.now();

        // Init variables
        uint N = starts.size();
        if (threads == 0)
        {
            threads = std::thread::hardware_concurrency();
        }

        extra.threads = std::max(threads,0);
        extra.possible_paths_num = max_path_num();
        min_sep = std::abs(min_sep);
         
        // Check safety
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(starts.size() == ends.size());
        assert(starts.size() == stats.size());
        assert(starts.size() == times.size());
        assert(obstacle_paths.size() == obstacle_stats.size());

        for(uint i = 0; i < N; i++)
        {
            for(uint j = i+1; j < N; j++)
            {
                assert(pose_dist_XY(starts[i],starts[j]) > min_sep);
            }
        }
#endif

        ListOfPossibilities possibilities;
        if (reallocate)
        {
            possibilities = list_all_possibilities_with_reallocation(starts,ends,stats,times,wind_x,wind_y);
        }
        else
        {
            possibilities = list_all_possibilities(starts,ends,stats,times,wind_x,wind_y);
        }

        if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
        {
            std::cout << "Before dynamic obstacles filtering, there are " << possibilities.size() << " aircraft with the following number of paths : ";
            for(const auto& ac_possibilities : possibilities)
            {
                std::cout << ac_possibilities.size() << ", ";
            }
            std::cout << std::endl;
        }

        filter_colliding_paths<distance_function>(possibilities,stats,obstacle_paths,obstacle_stats,obstacle_delays,min_sep);

        if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
        {
            std::cout << "After dynamic obstacles filtering, there are " << possibilities.size() << " aircraft with the following number of paths : ";
            for(const auto& ac_possibilities : possibilities)
            {
                std::cout << ac_possibilities.size() << ", ";
            }
            std::cout << std::endl;
        }

        std::set<uint> nopaths_acs_indexes = acs_without_paths(possibilities);

        if (nopaths_acs_indexes.size() == 0)
        {
            filter_2D_geometric_colliding_paths(possibilities,stats,geometric_obstacles,precision_tol,wind_x,wind_y);

            if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
            {
                std::cout << "After 2D geometric filtering, there are " << possibilities.size() << " aircraft with the following number of paths : ";
                for(const auto& ac_possibilities : possibilities)
                {
                    std::cout << ac_possibilities.size() << ", ";
                }
                std::cout << std::endl;
            }

            nopaths_acs_indexes = acs_without_paths(possibilities);
        }

        std::vector<Dubins> sol = {};

        if (nopaths_acs_indexes.size() == 0)
        {
            if (threads > 1)
            {
                uint max_p_num = 0;
                for(const auto& ac_possibilities : possibilities)
                {
                    max_p_num = std::max(max_p_num,static_cast<uint>(ac_possibilities.size()));
                }
                
                sol = find_solution_parallel<separation_function,distance_function>(possibilities,stats,min_sep,max_p_num,threads,
                    false,
                    verbosity >= DubinsFleetPlanner_VERY_VERBOSE,
                    *times.begin());
            }
            else
            {
                sol = find_solution<separation_function>(possibilities,stats,min_sep);
            }
        } 
        else
        {
            if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
            {
                std::cout   << "AC ids : ";
                for(uint i : nopaths_acs_indexes)
                {
                    std::cout << stats[i].id << ", ";
                } 
                std::cout << " have no path available..." << std::endl;
            }
        }

        auto end = clk.now();

        extra.success = sol.size() == stats.size();
        extra.duration = end - start;

        res = sol;
        return nopaths_acs_indexes;
    }

    /**
     * @brief Solve the pathfinding problem using Dubins paths with known differences between time of arrivals, over a range of
     * possible times
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @tparam distance_function    Function computing the minimal distance location and value between two trajectories
     * @param res           Passed by ref, where to put the result
     * @param extra         Extra information regarding the solving process
     * @param starts        List of starting poses 
     * @param ends          List of ending poses
     * @param stats         Statistics for aircraft
     * @param min_sep       Minimal required separation
     * @param delta_t       Wanted time separation between arrivals.
     * @param wind_x        X component of the wind vector. Default to no wind.
     * @param wind_y        Y component of the wind vector. Default to no wind.
     * @param reallocate If true, allow aircraft to swap endpoints. Default to false.
     * @param max_iters     Maximal number of iterations before aborting research. Default to 300
     * @param weave_iters   Number of iterations to add between two failed samples. Default to 2
     * @param min_weave_delta   Minimal value between two iterations for weaving between them. Default to 1.
     * @param threads       Number of threads to use for solving. 
     *                          If nonpositive, disable threading
     *                          If 1, set the number of threads to std::thread::hardware_concurrency()
     *                          Otherwise, use the given value
     * @param obstacle_paths Paths acting as obstacles (representing other aircraft)
     * @param obstacle_stats Flight characteristics of the obstacle aircraft
     * @param obstacle_delays Delays before which each obstacle path is active (same size as obstacle_paths)
     * @param geometric_obstacles  Geometric obstacles to avoid (2D only)
     */
    template<Dubins::DubinsSeparationFunction separation_function, Dubins::DubinsDistanceFunction distance_function>
    void solve(
        std::vector<Dubins>& res,
        ExtraPPResults& extra,
        const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
        const std::vector<AircraftStats>& stats, double min_sep,
        const std::vector<double>& delta_t,
        double wind_x = 0., double wind_y = 0.,
        bool reallocate = false,
        uint max_iters = 300, uint weave_iters = 2,
        double min_weave_delta =1., double min_weave_frac =0.0001,
        int threads = -1,
        int max_time_s = 60,
        const std::vector<Dubins>& obstacle_paths = {}, const std::vector<AircraftStats>& obstacle_stats = {},
        const std::vector<double>& obstacle_delays = {},
        const std::vector<DynamicPathShape>& geometric_obstacles = {}
    ) const
    {
        // Timing
        chrono::milliseconds max_dur_s(static_cast<long int>(max_time_s*1000));
        auto start = chrono::thread_clock::now();
        auto max_date = start + max_dur_s;

        // Variables preprocessing
        uint N = starts.size();
        if (threads == 0)
        {
            threads = std::thread::hardware_concurrency();
        }        
        
        extra.threads = std::max(threads,0);
        extra.possible_paths_num = max_path_num();

        if (verbosity >= DubinsFleetPlanner_VERBOSE)
        {
            std::cout << "Using " << extra.threads << " threads" << std::endl;
        }

        min_sep = std::abs(min_sep);
         
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(starts.size() == ends.size());
        assert(starts.size() == stats.size());
        assert(starts.size() == delta_t.size() +1);
        assert(obstacle_paths.size() == obstacle_stats.size());

        for(uint i = 0; i < N; i++)
        {
            for(uint j = i+1; j < N; j++)
            {
                assert(pose_dist_XY(starts[i],starts[j]) > min_sep);
            }
        }
#endif


        double min_time = maxmin_dubins_traveltime(starts,ends,stats,delta_t,wind_x,wind_y,reallocate);

        extra.initial_path_time = min_time;

        double max_time = min_time * maximal_relative_duration;

        min_weave_delta = std::max(min_weave_delta,min_weave_frac * max_time);

        std::list<SampleCase> samples = generate_linspace(min_time,max_time,weave_iters,true,true);
        auto iter = samples.begin();

        uint iter_count = 0;
        std::vector<Dubins> best_sol;
        double best_time = max_time;

        auto solve_timepoint = start;
        double solver_rate = INFINITY;

        while(iter_count < max_iters && samples.size() > 1 && chrono::thread_clock::now() < max_date)
        {
            iter_count++;

            // Remove cases already done
            while(iter != samples.end() && iter->done)
            {
                iter++;
            }

            // If nothing left, regenerate
            if (iter == samples.end())
            {
                if (verbosity >= DubinsFleetPlanner_VERYVERY_VERBOSE)
                {
                    std::cout << "Adding new samples, going from " << samples.size() << " to ";
                }

                if (samples.size() <= 1)
                {
                    break;
                }
                bool new_samples = weave_samples(samples,weave_iters,min_weave_delta);

                if (verbosity >= DubinsFleetPlanner_VERYVERY_VERBOSE)
                {
                    std::cout << samples.size() << std::endl;
                }

                if (!new_samples)
                {
                    if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
                    {
                        std::cout << "No new samples added; terminating early" << std::endl;
                    }
                    break;
                }

                iter = samples.begin();
                continue;
            }

            // Found a valid sample to test!

            double target_time = iter->val;

            if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
            {
                std::cout   << "Starting iteration number " << iter_count 
                            << " (target time: " << target_time << " )" << std::endl;
            }

            // Compute arrival time for each AC and solve
            std::vector<double> times = compute_arrival_times(delta_t,target_time);


            std::vector<Dubins> sol;
            std::set<uint> nopaths_acs;
            nopaths_acs = solve_once<separation_function,distance_function>(
                sol,
                extra,starts,ends,
                stats,
                min_sep,times,
                wind_x,wind_y,
                reallocate,
                threads,
                obstacle_paths,obstacle_stats,obstacle_delays,
                geometric_obstacles);

            iter->NoPathsACs = nopaths_acs;
            iter->done = true;

            if (sol.size() == stats.size())
            {
                iter->success = true;

                best_sol = sol;

                auto now = chrono::thread_clock::now();
                chrono::nanoseconds dt = now - solve_timepoint;
                double rate = (best_time - target_time)/(static_cast<double>(dt.count())/1e9);

                solver_rate = std::min(solver_rate,rate);

                solve_timepoint = now;
                best_time = target_time;

                if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
                {
                    std::cout << "Found a solution at " << best_time << std::endl;
                }

                // Remove all values after the best solution so far
                auto next = iter;
                next++;
                
                if (verbosity >= DubinsFleetPlanner_VERYVERY_VERBOSE)
                {
                    std::cout << "Dropping points from " << samples.size();
                }

                samples.erase(next,samples.end());

                if (verbosity >= DubinsFleetPlanner_VERYVERY_VERBOSE)
                {
                    std::cout << " to " << samples.size();
                }

                // Weave
                if (samples.size() <= 1)
                {
                    break;
                }
                bool new_samples = weave_samples(samples,weave_iters,min_weave_delta);

                if (verbosity >= DubinsFleetPlanner_VERYVERY_VERBOSE)
                {
                    std::cout << " then weaved to " << samples.size() << std::endl;
                }

                // Reset iterator
                iter = samples.begin();
            }
            else
            {
                iter->success = false;
            }
            iter++;
        }

        extra.duration = chrono::thread_clock::now() - start;

        if (verbosity >= DubinsFleetPlanner_VERBOSE && extra.duration > max_dur_s)
        {
            std::cout << "WARNING: Timeout stop!" << std::endl;
        }

        extra.success = best_sol.size() == stats.size();

        extra.worst_improvement_rate = solver_rate;
        extra.iterations        = iter_count;
        extra.final_path_time   = best_time;
        res = best_sol;
    }

    /**
     * @brief Solve the pathfinding problem using Dubins paths with fixed arrival times
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @tparam distance_function    Function computing the minimal distance location and value between two trajectories
     * @param res           Passed by ref, where to put the result
     * @param extra         Extra information regarding the solving process
     * @param starts        List of starting poses 
     * @param ends          List of ending poses
     * @param stats         Statistics for aircraft
     * @param timeslots     For each aircraft, its possible arrival times (should be sorted by preferred arrival time)
     * @param delta_t       Wanted time separation between arrivals.
     * @param wind_x        X component of the wind vector. Default to no wind.
     * @param wind_y        Y component of the wind vector. Default to no wind.
     * @param threads       Number of threads to use for solving. 
     *                          If negative, disable threading
     *                          If 0, set the number of threads to std::thread::hardware_concurrency()
     *                          Otherwise, use the given value
     * @param add_shortest  If true, adds shortest possible paths for each aircraft. Note that with this enabled, a pth may have a duration outside of the allocated timeslots. 
     * @param obstacle_paths Paths acting as obstacles (representing other aircraft)
     * @param obstacle_stats Flight characteristics of the obstacle aircraft
     * @param obstacle_delays Delays before which each obstacle path is active (same size as obstacle_paths)
     * @param geometric_obstacles  Geometric obstacles to avoid (2D only)
     * @return std::set<uint> The set of aircraft IDs with no possible paths
     */
    template<Dubins::DubinsSeparationFunction separation_function, Dubins::DubinsDistanceFunction distance_function>
    std::set<uint> solve_with_timeslots(
        std::vector<Dubins>& res,
        ExtraPPResults& extra,
        const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
        const std::vector<AircraftStats>& stats, double min_sep,
        const std::vector<std::vector<double>>& timeslots,
        double wind_x = 0., double wind_y = 0.,
        int threads = -1,
        bool add_shortest = false,
        const std::vector<Dubins>& obstacle_paths = {}, const std::vector<AircraftStats>& obstacle_stats = {},
        const std::vector<double>& obstacle_delays = {},
        const std::vector<DynamicPathShape>& geometric_obstacles = {}
    ) const
    {
        // Timing
        chrono::process_real_cpu_clock clk;
        auto start = clk.now();

        // Init variables
        uint N = starts.size();
        if (threads == 0)
        {
            threads = std::thread::hardware_concurrency();
        }

        extra.threads = std::max(threads,0);
        extra.possible_paths_num = max_path_num();
        min_sep = std::abs(min_sep);

        
        // Check safety
    #if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(starts.size() == ends.size());
        assert(starts.size() == stats.size());
        assert(starts.size() == timeslots.size());
        assert(obstacle_paths.size() == obstacle_stats.size());
    #endif

        double maxmin_ts = 0.;
        size_t max_timeslots = timeslots[0].size();
        for(auto ts : timeslots)
        {
            maxmin_ts = std::max(maxmin_ts,*std::min_element(ts.begin(),ts.end()));
            max_timeslots = std::max(max_timeslots,ts.size());
        }
        extra.initial_path_time = maxmin_ts;

        ListOfPossibilities possibilities = list_all_possibilities(starts,ends,stats,timeslots,wind_x,wind_y);

        if (add_shortest)
        {
            for(uint i = 0; i < N; i++)
            {
                auto& paths = possibilities[i];
                const AircraftStats& stat = stats[i];
                const Pose3D& start = starts[i];
                const Pose3D& end   = ends[i];

                auto shortests = generate_shortest_paths(i,start,end,stat,wind_x,wind_y);
                paths.insert(
                    paths.end(),
                    std::move_iterator(shortests.begin()),
                    std::move_iterator(shortests.end())
                );
            }
        }

        if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
        {
            std::cout << "Involved Aircraft IDs: ";
            for(const auto& stat : stats)
            {
                std::cout << stat.id << ", ";
            }
            std::cout << std::endl;
            std::cout << "Before dynamic obstacles filtering, there are " << possibilities.size() << " aircraft with the following number of paths : ";
            for(const auto& ac_possibilities : possibilities)
            {
                std::cout << ac_possibilities.size() << ", ";
            }
            std::cout << std::endl;
        }

        filter_colliding_paths<distance_function>(possibilities,stats,obstacle_paths,obstacle_stats,obstacle_delays,min_sep);

        filter_2D_geometric_colliding_paths(possibilities,stats,geometric_obstacles,precision_tol,wind_x,wind_y);

        if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
        {
            std::cout << "After filtering, there are " << possibilities.size() << " aircraft with the following number of paths : ";
            for(const auto& ac_possibilities : possibilities)
            {
                std::cout << ac_possibilities.size() << ", ";
            }
            std::cout << std::endl;
        }

        std::set<uint> nopaths_acs_indexes = acs_without_paths(possibilities);

        std::vector<Dubins> sol = {};

        if (nopaths_acs_indexes.size() == 0)
        {
            if (threads > 1)
            {
                uint max_p_num = 0;
                for(const auto& ac_possibilities : possibilities)
                {
                    max_p_num = std::max(max_p_num,static_cast<uint>(ac_possibilities.size()));
                }

                sol = find_solution_parallel<separation_function,distance_function>(
                    possibilities,stats,min_sep,max_p_num,threads,true,
                    verbosity >= DubinsFleetPlanner_VERY_VERBOSE);
            }
            else
            {
                sol = find_solution<separation_function>(possibilities,stats,min_sep);
            }
        } 
        else
        {
            if (verbosity >= DubinsFleetPlanner_VERBOSE)
            {
                std::cout   << "AC ids : ";
                for(uint i : nopaths_acs_indexes)
                {
                    std::cout << stats[i].id << ", ";
                } 
                std::cout << " have no path available..." << std::endl;
            }
        }

        auto end = clk.now();

        extra.success = sol.size() == stats.size();
        extra.duration = end - start;
        extra.iterations = 1;
        for (const auto& tslots : timeslots)
        {
            extra.iterations *= tslots.size();
        }

        if (extra.success)
        {
            uint i = 0;
            double max_duration = 0.;
            for(const auto& d : sol)
            {
                const auto& s = stats[i];
                max_duration = std::max(max_duration,d.get_duration());
                i++;
            }
            extra.final_path_time = max_duration;
        }

        res = sol;
        return nopaths_acs_indexes;
    }
    
};
