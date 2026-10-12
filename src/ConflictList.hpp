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

#include "ProjectHeader.h"

#include <array>
#include <vector>
#include <thread>
#include <future>
#include <optional>

#include <highs/Highs.h>

#include "Aircraft.h"
#include "Dubins.hpp"
#include "Primitives.hpp"

// ---------- Util functions ---------- //

/**
 * @brief Represent a conflict in the form (ac_id1,path_id1,ac_id2,path_id2)
 * 
 */
typedef std::tuple<unsigned short, unsigned short, unsigned short, unsigned short> Conflict_T;

/**
 * @brief Represent a conflict with its information in the form
 *  (ac_id1,path_id1,ac_id2,path_id2,location,value)
 * 
 */
typedef std::tuple<unsigned short, unsigned short, unsigned short, unsigned short, double, double> RichConflict_T;

/**
 * @brief Convert a vector of RichConflict_T into a vector of Conflict_T while dropping irrelevant conflicts (distance higher than given bound)
 * 
 * @param vec Vector of RichConflict_T
 * @param min_dist Minimal separation distance
 * @return std::vector<Conflict_T> 
 */
std::vector<Conflict_T> drop_conflict_details(const std::vector<RichConflict_T>& vec, double min_dist);

/**
 * @brief Log the characteristics of a conflict (location,distance)
 * 
 */
typedef std::map<Conflict_T,std::pair<double,double>> Conflict_Map_T;

typedef std::vector<std::vector<std::unique_ptr<Dubins>>> ListOfPossibilities;
typedef std::vector<std::vector<std::shared_ptr<Dubins>>> SharedListOfPossibilities;

uint number_of_valid_paths(const ListOfPossibilities& list);

// ---------- Separation functions ---------- //

// ----- Generic functions ----- //

/**
 * @brief Given a separation function, possible paths, aircraft characteristics and a minimal separation,
 * compute the list of conflicting paths
 * 
 * @tparam separation_function Given two paths with their aircarft stats and a minimum separation, returns if there is a conflict or not
 * @param list_of_possibilites  List of possible paths for each aircraft
 * @param stats                 Aircraft stats
 * @param sep                   Minimal separation required
 * @return std::vector<Conflict_T>  List of conflicting aircraft + paths combination
 */
template<Dubins::DubinsSeparationFunction separation_function>
std::vector<Conflict_T> generic_compute_separations(const ListOfPossibilities& list_of_possibilites, 
    const std::vector<AircraftStats>& stats, double sep);

/**
 * @brief Given a separation function, possible paths, aircraft characteristics and a minimal separation,
 * compute the list of conflicting paths using multiple threads
 * 
 * @tparam separation_function  Given two paths with their aircarft stats and a minimum separation, returns if there is a conflict or not
 * @param THREADS               Number of threads to use
 * @param list_of_possibilites  List of possible paths for each aircraft
 * @param stats                 Aircraft stats
 * @param sep                   Minimal separation required
 * @return std::vector<Conflict_T>  List of conflicting aircraft + paths combination
 */
template<Dubins::DubinsSeparationFunction separation_function>
std::vector<Conflict_T> generic_parallel_compute_separations(uint THREADS, const SharedListOfPossibilities& list_of_possibilites, 
    const std::vector<AircraftStats>& stats, double sep);

    
// ---------- Distance functions ---------- //

/**
 * @brief Given a distance function, possible paths, aircraft characteristics and a minimal separation,
 * compute the list of conflicting paths with the conflict location and evaluation
 * 
 * @tparam distance_function    Given two paths with their aircarft stats and a minimum separation, 
 *  returns the closest point separating them with associated distance
 * @param list_of_possibilites  List of possible paths for each aircraft
 * @param stats                 Aircraft stats
 * @param sep                   Minimal separation required
 * @param map                   A map containing hints for the solver trying to find conflicts
 * @param all_values            Whether to returns all evaluations or only the ones generating conflicts
 * @return std::vector<RichConflict_T>  List of conflicting aircraft + paths combination with the location and distance
 */
template<Dubins::DubinsDistanceFunction distance_function>
std::vector<RichConflict_T> generic_compute_distances(
    const ListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats,
    double sep, const Conflict_Map_T& map, bool all_values);

/**
 * @brief Given a distance function, possible paths, aircraft characteristics and a minimal separation,
 * compute the list of conflicting paths with the conflict location and evaluation using multiple threads
 * 
 * @tparam distance_function    Given two paths with their aircarft stats and a minimum separation, 
 *  returns the closest point separating them with associated distance
 * @param THREADS               Number of threads to use
 * @param list_of_possibilites  List of possible paths for each aircraft
 * @param stats                 Aircraft stats
 * @param sep                   Minimal separation required
 * @param map                   A map containing hints for the solver trying to find conflicts
 * @param all_values            Whether to returns all evaluations or only the ones generating conflicts
 * @return std::vector<RichConflict_T>  List of conflicting aircraft + paths combination with the location and distance
 */
template<Dubins::DubinsDistanceFunction distance_function>
std::vector<RichConflict_T> generic_parallel_compute_distances(
    uint THREADS, const SharedListOfPossibilities& list_of_possibilites, 
    const std::vector<AircraftStats>& stats, double sep,
    const Conflict_Map_T& map, bool all_values);

/**
 * @brief Solve the path finding problem using HiGHS ILP solver
 * 
 * We denote N the number of aircraft and P the number of paths per aircraft.
 * The decisions variables are the x_{ik} with 0 <= i < N and 0 <= p < P describing
 * if aircraft i uses path p.
 * 
 * We have some general constraints:
 * - Variables are 01 (a path is either chosen or not):
 *     Forall i in [0,N-1], forall p in [0,P-1]  x_{ip} in {0,1}
 *     (i.e. x_{ip} is an integer with 0 <= x_{ip} <= 1)
 * - Each aircraft must be allocated to one and only one path:
 *     Forall i, sum_{p=0}^{P-1} x_{ip} = 1
 * 
 * We also have some specific constraints, defined using additional data:
 * - There a conflict matrix M with dimension N x N x P x P, such that M_{ijpq} is 1 if
 *   there is a conflict between path p of aircraft i and path q of aircraft j, and 0 otherwise.
 *   The associated 'collision-free' constraints are written as:
 *     Forall i,j and forall p,q M_{ijpq} * (x_{ip} + x_{jq}) <= 1
 *   (Note: the indices are flattened, assuming all aircraft have the same number of possible paths)
 * 
 * @param possibilites  List of possibles paths per aircraft
 * @param stats         Caracteristics of each aircraft
 * @param conflicts     List of conflicts (paths that cannot be taken together)
 * @param max_path_num  Maximum number of paths for an aircraft
 * @param THREADS       Number of threads to use. 0 choose automatically. Default to 0
 * @param duration_based_costs  If true, the cost of each path is the time taken to realize it. Thus the objective becomes minimizing the sum of times.
 *                              If false, costs are all set to 1.
 * @param preset_model  A preset HiGHS model to quickly setup
 * @return std::vector<Dubins>
 */
std::vector<Dubins> find_pathplanning_LP_solution(
    SharedListOfPossibilities& possibilites, 
    const std::vector<AircraftStats>& stats, 
    const std::vector<Conflict_T>& conflicts,
    uint max_path_num, int THREADS, bool duration_based_costs, int verbosity);



// ================================================================================== //
// =====                                                                        ===== //
// =====                            Implementation                              ===== //
// =====                                                                        ===== //
// ================================================================================== //


// ---------------------------------------- Util functions ---------------------------------------- //

std::vector<Conflict_T> drop_conflict_details(const std::vector<RichConflict_T>& vec, double min_dist);

uint number_of_valid_paths(ListOfPossibilities& list);


template<Dubins::DubinsSeparationFunction separation_function>
static inline bool generic_check_compatibility(const Dubins& d1, const Dubins& d2, const AircraftStats& s1, const AircraftStats& s2, double sep)
{
    double duration = std::min(d1.get_duration(s1),d2.get_duration(s2));
    return d1.is_valid() && d2.is_valid() && separation_function(d1,d2,s1,s2,duration,sep,DubinsFleetPlanner_PRECISION,DubinsDistDefaultRec);
}

template<Dubins::DubinsDistanceFunction distance_function>
static inline std::pair<double,double> generic_compute_distance(
    const Dubins& d1, const Dubins& d2, 
    const AircraftStats& s1, const AircraftStats& s2, 
    double sep,
    std::optional<double> hotstart = std::nullopt)
{
    double duration = std::min(d1.get_duration(s1),d2.get_duration(s2));
    if (!(d1.is_valid() && d2.is_valid()))
    {
        return std::make_pair(0.,INFINITY);
    }
    else
    {
        return distance_function(d1,d2,s1,s2,duration,sep,DubinsFleetPlanner_PRECISION,hotstart,DubinsDistDefaultRec);
    }
}



// ---------------------------------------- Single thread functions ---------------------------------------- //

// -------------------- Only separation -------------------- //

template<Dubins::DubinsSeparationFunction separation_function>
std::vector<Conflict_T> generic_compute_separations(const ListOfPossibilities& list_of_possibilites, 
    const std::vector<AircraftStats>& stats, double sep)
{
    uint N = list_of_possibilites.size();

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(list_of_possibilites.size() == stats.size());
#endif

    std::vector<Conflict_T> output;

    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            const std::vector<std::unique_ptr<Dubins>>& d1_paths = list_of_possibilites[i];
            const std::vector<std::unique_ptr<Dubins>>& d2_paths = list_of_possibilites[j];

            const AircraftStats& d1_stats = stats[i];
            const AircraftStats& d2_stats = stats[j];

            uint P1 = d1_paths.size();
            uint P2 = d2_paths.size();

            for(uint p1 = 0; p1 < P1; p1++)
            {
                for(uint p2 = 0; p2 < P2; p2++)
                {
                    const Dubins& d1 = *d1_paths[p1];
                    const Dubins& d2 = *d2_paths[p2];

                    if (!generic_check_compatibility<separation_function>(d1,d2,d1_stats,d2_stats,sep))
                    {
                        output.push_back({
                            i,p1,j,p2
                        });
                    }
                }
            }

        }
    }

    return output;
}


template<Dubins::DubinsSeparationFunction separation_function>
std::vector<Conflict_T> generic_compute_separations(const SharedListOfPossibilities& list_of_possibilites, 
    const std::vector<AircraftStats>& stats, double sep)
{
    uint N = list_of_possibilites.size();

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(list_of_possibilites.size() == stats.size());
#endif

    std::vector<Conflict_T> output;

    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            const std::vector<std::shared_ptr<Dubins>>& d1_paths = list_of_possibilites[i];
            const std::vector<std::shared_ptr<Dubins>>& d2_paths = list_of_possibilites[j];

            const AircraftStats& d1_stats = stats[i];
            const AircraftStats& d2_stats = stats[j];

            uint P1 = d1_paths.size();
            uint P2 = d2_paths.size();

            for(uint p1 = 0; p1 < P1; p1++)
            {
                for(uint p2 = 0; p2 < P2; p2++)
                {
                    const Dubins& d1 = *d1_paths[p1];
                    const Dubins& d2 = *d2_paths[p2];

                    if (!generic_check_compatibility<separation_function>(d1,d2,d1_stats,d2_stats,sep))
                    {
                        output.push_back({
                            i,p1,j,p2
                        });
                    }
                }
            }

        }
    }

    return output;
}

// -------------------- Separation and distance -------------------- //


template<Dubins::DubinsDistanceFunction distance_function>
std::vector<RichConflict_T> generic_compute_distances(
    const ListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats,
    double sep, const Conflict_Map_T& map, bool all_values)
{
    uint N = list_of_possibilites.size();

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(list_of_possibilites.size() == stats.size());
#endif

    std::vector<RichConflict_T> output;

    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            const std::vector<std::unique_ptr<Dubins>>& d1_paths = list_of_possibilites[i];
            const std::vector<std::unique_ptr<Dubins>>& d2_paths = list_of_possibilites[j];

            const AircraftStats& d1_stats = stats[i];
            const AircraftStats& d2_stats = stats[j];

            uint P1 = d1_paths.size();
            uint P2 = d2_paths.size();

            for(uint p1 = 0; p1 < P1; p1++)
            {
                for(uint p2 = 0; p2 < P2; p2++)
                {
                    const Dubins& d1 = *d1_paths[p1];
                    const Dubins& d2 = *d2_paths[p2];

                    std::optional<double> hotstart;
                    auto search = map.find({i,p1,j,p2});
                    if (search == map.end())
                    {
                        hotstart = std::nullopt;
                    }
                    else
                    {
                        hotstart = search->second.first;
                    }

                    std::pair<double,double> cvals = generic_compute_distance<distance_function>(d1,d2,d1_stats,d2_stats,sep,hotstart);
                    if (all_values)
                    {
                        output.push_back({
                            i,p1,j,p2,cvals.first,cvals.second
                        });
                    }
                    else
                    {
                        if (cvals.second < sep)
                        {
                            output.push_back({
                                i,p1,j,p2,cvals.first,cvals.second
                            });
                        }
                    }
                }
            }

        }
    }

    return output;
}

template<Dubins::DubinsDistanceFunction distance_function>
std::vector<RichConflict_T> generic_compute_distances(
    const SharedListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats,
    double sep, const Conflict_Map_T& map, bool all_values)
{
    uint N = list_of_possibilites.size();

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(list_of_possibilites.size() == stats.size());
#endif

    std::vector<RichConflict_T> output;

    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            const std::vector<std::unique_ptr<Dubins>>& d1_paths = list_of_possibilites[i];
            const std::vector<std::unique_ptr<Dubins>>& d2_paths = list_of_possibilites[j];

            const AircraftStats& d1_stats = stats[i];
            const AircraftStats& d2_stats = stats[j];

            uint P1 = d1_paths.size();
            uint P2 = d2_paths.size();

            for(uint p1 = 0; p1 < P1; p1++)
            {
                for(uint p2 = 0; p2 < P2; p2++)
                {
                    const Dubins& d1 = *d1_paths[p1];
                    const Dubins& d2 = *d2_paths[p2];

                    std::optional<double> hotstart;
                    auto search = map.find({i,p1,j,p2});
                    if (search == map.end())
                    {
                        hotstart = std::nullopt;
                    }
                    else
                    {
                        hotstart = search->second.first;
                    }

                    std::pair<double,double> cvals = generic_compute_distance<distance_function>(d1,d2,d1_stats,d2_stats,sep,hotstart);
                    if (all_values)
                    {
                        output.push_back({
                            i,p1,j,p2,cvals.first,cvals.second
                        });
                    }
                    else
                    {
                        if (cvals.second < sep)
                        {
                            output.push_back({
                                i,p1,j,p2,cvals.first,cvals.second
                            });
                        }
                    }
                }
            }

        }
    }

    return output;
}

// ---------------------------------------- Multi thread functions ---------------------------------------- //

// -------------------- Only separation -------------------- //

template<Dubins::DubinsSeparationFunction separation_function>
std::vector<Conflict_T> _subtask_compute_separations(
    const SharedListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats,
    double sep, uint start_i, uint start_j, uint ac_num)
{

    uint N = list_of_possibilites.size();
    uint done = 0;

    uint i = start_i;
    uint j = start_j;

    std::vector<Conflict_T> output;

    while(i < N)
    {
        while(j < N)
        {
            const std::vector<std::shared_ptr<Dubins>>& d1_paths = list_of_possibilites[i];
            const std::vector<std::shared_ptr<Dubins>>& d2_paths = list_of_possibilites[j];

            const AircraftStats& d1_stats = stats[i];
            const AircraftStats& d2_stats = stats[j];

            uint P1 = d1_paths.size();
            uint P2 = d2_paths.size();

            for(uint p1 = 0; p1 < P1; p1++)
            {
                for(uint p2 = 0; p2 < P2; p2++)
                {
                    const Dubins& d1 = *d1_paths[p1];
                    const Dubins& d2 = *d2_paths[p2];

                    if (!generic_check_compatibility<separation_function>(d1,d2,d1_stats,d2_stats,sep))
                    {
                        output.push_back({
                            i,p1,j,p2
                        });
                    }
                }
            }

            done++;
            if (done >= ac_num)
            {
                return output;
            }

            j++;
        }

        i++;
        j = i+1;
    }

    return output;
}


template<Dubins::DubinsSeparationFunction separation_function>
std::vector<Conflict_T> generic_parallel_compute_separations(uint THREADS, const SharedListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats, double sep)
{
    assert(THREADS > 0);

    uint N = list_of_possibilites.size();

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(list_of_possibilites.size() == stats.size());
#endif

    uint tasks_to_do = N*(N-1)/2;
    std::vector<Conflict_T> output;

    uint tasks_per = tasks_to_do/THREADS;
    uint thread_started = 0;
    uint tasks_given = 0;

    uint start_i,start_j;

    std::vector<std::future<std::vector<Conflict_T>>> futures;

    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            if (tasks_given >= tasks_per)
            {
                futures.push_back(
                    std::async(
                        std::launch::async,
                        _subtask_compute_separations<separation_function>,
                        list_of_possibilites,stats,sep,
                        start_i,start_j,tasks_given
                    )
                );
                tasks_to_do -= tasks_given;
                thread_started++;
                tasks_given = 0;
            }

            if (tasks_given == 0)
            {
                start_i = i;
                start_j = j;
            }

            tasks_given++;

        }
    }

    if (tasks_given > 0)
    {
        output = _subtask_compute_separations<separation_function>(
            list_of_possibilites,stats,
            sep,start_i,start_j,tasks_given);
    }

    for(auto& res : futures)
    {
        res.wait();
        auto res_val = res.get();
        output.insert(output.end(),res_val.begin(), res_val.end());
    }

    return output;
}

// -------------------- Separation and distance -------------------- //

template<Dubins::DubinsDistanceFunction distance_function>
std::vector<RichConflict_T> _subtask_compute_distances(
    const SharedListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats,
    double sep, uint start_i, uint start_j, uint ac_num, const Conflict_Map_T& map, bool all_values)
{

    uint N = list_of_possibilites.size();
    uint done = 0;

    uint i = start_i;
    uint j = start_j;

    std::vector<RichConflict_T> output;

    while(i < N)
    {
        while(j < N)
        {
            const std::vector<std::shared_ptr<Dubins>>& d1_paths = list_of_possibilites[i];
            const std::vector<std::shared_ptr<Dubins>>& d2_paths = list_of_possibilites[j];

            const AircraftStats& d1_stats = stats[i];
            const AircraftStats& d2_stats = stats[j];

            uint P1 = d1_paths.size();
            uint P2 = d2_paths.size();

            for(uint p1 = 0; p1 < P1; p1++)
            {
                for(uint p2 = 0; p2 < P2; p2++)
                {
                    const Dubins& d1 = *d1_paths[p1];
                    const Dubins& d2 = *d2_paths[p2];

                    std::optional<double> hotstart;
                    auto search = map.find({i,p1,j,p2});
                    if (search == map.end())
                    {
                        hotstart = std::nullopt;
                    }
                    else
                    {
                        hotstart = search->second.first;
                    }

                    std::pair<double,double> cvals = generic_compute_distance<distance_function>(d1,d2,d1_stats,d2_stats,sep,hotstart);

                    if (all_values)
                    {
                        output.push_back({
                            i,p1,j,p2,cvals.first,cvals.second
                        });
                    }
                    else
                    {
                        if (cvals.second < sep)
                        {
                            output.push_back({
                                i,p1,j,p2,cvals.first,cvals.second
                            });
                        }
                    }
                }
            }

            done++;
            if (done >= ac_num)
            {
                return output;
            }

            j++;
        }

        i++;
        j = i+1;
    }

    return output;
}


template<Dubins::DubinsDistanceFunction distance_function>
std::vector<RichConflict_T> generic_parallel_compute_distances(
    uint THREADS, const SharedListOfPossibilities& list_of_possibilites, 
    const std::vector<AircraftStats>& stats, double sep,
    const Conflict_Map_T& map, bool all_values)
{
    assert(THREADS > 0);

    uint N = list_of_possibilites.size();

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(list_of_possibilites.size() == stats.size());
#endif

    uint tasks_to_do = N*(N-1)/2;
    std::vector<RichConflict_T> output;

    uint tasks_per = tasks_to_do/THREADS;
    uint thread_started = 0;
    uint tasks_given = 0;

    uint start_i = 0;
    uint start_j = 1;

    std::vector<std::future<std::vector<RichConflict_T>>> futures;

    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            if (tasks_given >= tasks_per)
            {
                futures.push_back(
                    std::async(
                        std::launch::async,
                        _subtask_compute_distances<distance_function>,
                        list_of_possibilites,stats,sep,
                        start_i,start_j,tasks_given,
                        map, all_values
                    )
                );
                tasks_to_do -= tasks_given;
                thread_started++;
                tasks_given = 0;
            }

            if (tasks_given == 0)
            {
                start_i = i;
                start_j = j;
            }

            tasks_given++;

        }
    }

    if (tasks_given > 0)
    {
        output = _subtask_compute_distances<distance_function>(
            list_of_possibilites,stats,
            sep,start_i,start_j,tasks_given,
            map, all_values);
    }

    for(auto& res : futures)
    {
        res.wait();
        auto res_val = res.get();
        output.insert(output.end(),res_val.begin(), res_val.end());
    }

    return output;
}
