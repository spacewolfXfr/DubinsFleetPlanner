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
#include <array>
#include <cmath>
#include <string>
#include <memory>
#include <optional>

#include "ProjectHeader.h"

#include "utils.hpp"
#include "Aircraft.h"
#include "Primitives.hpp"
#include "Fitting.hpp"
#include "Pose3D.hpp"

const uint DubinsDistDefaultRec = 6;

class Dubins
{
protected:
    Pose3D start; // Start pose (x,y,z, XY orientation)
    Pose3D end;   // End pose (x,y,z, XY orientation)

    std::vector<DynamicPathShape> sections;  // Parts of the path
    std::vector<double> junctions_locs = {}; // Location (in length) of junctions
    std::vector<Pose3D> junctions = {};      // Poses at junctions

    double length; // Path length. Is NAN if the path is not possible.

    bool valid = false; // Whether the desired path is possible or not

    /**
     * @brief Recompute the different values deduced from the problem if the start, end or control ever change
     */
    void recompute();

    /**
     * @brief Compute and return the path length.
     *
     * @return double Path length
     */
    double _compute_length()
    {
        length = 0.;
        for (const DynamicPathShape &s : sections)
        {
            length += s.length;
        }
        return length;
    }

public:
    /**
     * @brief Ensure that the path ends where it is supposed to
     *
     * This updates the 'valid' attribute, setting it to false if the end is not correct
     *
     * @return true The path ends correctly
     * @return false Either the path is already invalid, or it does not ends correctly
     */
    bool check_valid_end()
    {
        bool out = false;
        if (valid)
        {
            Pose3D end_check = get_position(length);
            out = pose_dist(end, end_check) < DubinsFleetPlanner_PRECISION;
            valid &= out;
        }
        return out;
    }

    /**
     * @brief Same as `_compute_length`, but also set the `length` and `valid` attributes
     *
     * @return double
     */
    double compute_length()
    {
        length = _compute_length();
        valid = (std::isfinite(length)) && (length >= 0);
        return length;
    }

    /************************ Constructors ************************/

    Dubins() : start(UndefPose),end(UndefPose),sections({}),length(NAN) {}

    Dubins(const Pose3D &_start, const Pose3D &_end, std::vector<DynamicPathShape> _sections)
        : start(_start), end(_end), sections(_sections)
    {
        recompute();
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(pose_dist(_sections.front().initial_pose(), _start) < DubinsFleetPlanner_PRECISION);
        assert(pose_dist(_sections.back().final_pose(), _end) < DubinsFleetPlanner_PRECISION);

        assert(abs(pose_aligned(_sections.front().initial_pose(), _start)) < DubinsFleetPlanner_PRECISION);
        assert(abs(pose_aligned(_sections.back().final_pose(), _end)) < DubinsFleetPlanner_PRECISION);
#endif
    }

    /************************ Getters ************************/

    Pose3D get_start() const { return start; }
    Pose3D get_end() const { return end; }

    double get_length() const { return length; }
    bool is_valid() const { return valid; }
    double get_duration(const AircraftStats stats) const
    {
        return 2*length/(stats.airspeed + stats.target_airspeed);
    }

    /**
     * @brief Return the length spent in turns in this path.
     * 
     * It is strictly equal to the total length minus the length of straits
     * 
     * @return double 
     */
    double turning_length() const
    {
        double output = 0.;
        for(const DynamicPathShape& s : sections)
        {
            if (s.is_turning())
            {
                output += s.length;
            }
        }
        return output;
    }

    /**
     * @brief Return the sum of absolute values of the changes in angle along this path.
     * This can be seen as "How much turning are we doing along this path" 
     * 
     * @return double 
     */
    double turning_angle() const
    {
        double output = 0.;
        for(const DynamicPathShape& s : sections)
        {
            output += s.turning_angle();
        }
        return output;
    }

    /**
     * @brief Get an abridged description of the path by using its components types
     *
     * @return const std::string
     */
    const std::string get_type_abbr() const
    {
        std::string output = "";
        for (const DynamicPathShape &s : sections)
        {
            output += get_DubinsMove_name(s.m)[0];
        }
        return output;
    }

    /**
     * @brief Get all sections types
     *
     * @return const std::vector<DubinsMove>
     */
    std::vector<DubinsMove> get_all_section_types() const
    {
        std::vector<DubinsMove> output;
        for (const DynamicPathShape &s : sections)
        {
            output.push_back(s.m);
        }
        return output;
    }

    /**
     * @brief Get the underlying section list
     * 
     * @return const std::vector<DynamicPathShape>& 
     */
    const std::vector<DynamicPathShape> &get_all_sections() const
    {
        return sections;
    }

    size_t section_count() const
    {
        return sections.size();
    }

    /**
     * @brief Get the position at the given length
     *
     * @param len Length parameter
     * @return Pose3D
     */
    Pose3D get_position(double len) const;

    /**
     * @brief Get the position at the given time, assuming linearly changing speed according to the aircraft's stats.
     *
     * @param time
     * @param stats Aircraft stats
     * @return Pose3D
     */
    Pose3D get_position(double time, const AircraftStats& stats) const
    {
        double duration = get_duration(stats);
        return get_position(length_traveled(stats, time, duration));
    }

    /**
     * @brief Get positions at several lengths
     *
     * @param lens Length abscissas
     * @param sorted If True, assume lengths are sorted (speed up computations)
     * @return const std::vector<Pose3D>
     */
    const std::vector<Pose3D> get_positions(std::vector<double> &lens, bool sorted = false) const;

    /**
     * @brief Get positions at several times, assuming linearly changing speed according to the aircraft's stats.
     *
     * @param times
     * @param stats Aircraft stats
     * @param sorted If True, assume times are sorted (speed up computations)
     * @return const std::vector<Pose3D>
     */
    const std::vector<Pose3D> get_positions(std::vector<double> &times, const AircraftStats& stats, bool sorted = false) const
    {
        double duration = get_duration(stats);

        std::vector<double> lens(times);
        for (double &e : lens)
        {
            e = length_traveled(stats, e, duration);
        }
        return get_positions(lens, sorted);
    }

    /**
     * @brief Get the path shape at the given length (object representing the FULL section)
     *
     * @param len
     * @return DynamicPathShape
     */
    DynamicPathShape get_pathshape(double len) const
    {
        uint i = 0;
        uint N = sections.size();

        while (i < N - 1 && len > junctions_locs[i])
        {
            i++;
        }

        return sections[i];
    }

    /**
     * @brief Get the path shape of the given section
     *
     * @param i Section index
     * @return DynamicPathShape
     */
    DynamicPathShape get_pathshape(uint i) const
    {
        return sections[i];
    }

    /**
     * @brief Get the location (in length coordinate) of the junction points between shapes
     *
     * @return std::vector<double>
     */
    std::vector<double> get_junction_locs() const
    {
        std::vector<double> output(junctions_locs);
        return output;
    }

    /**
     * @brief Get the location (in length coordinate) of the endpoints, ie start, junctions and end
     *
     * @return std::vector<double>
     */
    std::vector<double> get_endpoints_locs() const
    {
        std::vector<double> output{0.};

        for (double l : junctions_locs)
        {
            output.push_back(l);
        }

        output.push_back(get_length());
        return output;
    }

    /**
     * @brief Get the poses at the junctions between shapes
     *
     * @return std::vector<Pose3D>
     */
    std::vector<Pose3D> get_junction_points() const
    {
        std::vector<Pose3D> output(junctions);
        return output;
    }

    /**
     * @brief Get the poses of the endpoints, ie start, junctions and end
     *
     * @return std::vector<Pose3D>
     */
    std::vector<Pose3D> get_endpoints() const
    {
        std::vector<Pose3D> output;
        output.push_back(get_start());

        for (Pose3D l : junctions)
        {
            output.push_back(l);
        }

        output.push_back(get_end());
        return output;
    }

    /**
     * @brief Shorthand to get only the section type at the given location
     *
     * @param len
     * @return DubinsMove
     */
    DubinsMove get_section_type(double len) const
    {
        return get_pathshape(len).m;
    }

    /**
     * @brief Get a new Dubins path that follows this one for a given length
     *
     * @param len Length to follow
     * @return Dubins
     */
    Dubins follow_for(double len) const;

    /**
     * @brief Get a new Dubins path that follows this one for a given time at a given speed
     *
     * @param time Time to follow
     * @param stats Aircraft stats
     * @return Dubins
     */
    Dubins follow_for(double time, const AircraftStats& stats) const
    {
        double duration = get_duration(stats);
        return follow_for(length_traveled(stats, time, duration));
    }

    /************************ Seperation functions ************************/

    /**
     * @brief Compute the first point such that 'this' and 'that' breach their minimal separation,
     * or the minimal distance between the two proving that they are separated
     *
     *
     * @param other A second Dubins path
     * @param this_stats    Aircraft stats along `this` path
     * @param other_stats   Aircraft stats along `other` path
     * @param duration Duration for which to look for conflicts (in [time])
     * @param min_dist The minimal distance required for ensuring separation (in [time], to be multiplied by the aircraft's speed)
     * @param tol      Solver tolerance used when looking for the minimal distance
     * @param hostart  Optional value for hotstarting the search: a place where the minimal separation might fail
     * @param rec      Depth of the recursive call to the fast but inexact geometric approximation
     * @return A pair containing the location and value of either:
     *                  - The first trajectory part breaching the minimal distance
     *                  - The global minimal distance between the two
     */
    std::pair<double, double> XY_distance_to(const Dubins &other, const AircraftStats& this_stats, const AircraftStats& other_stats,
                                             double duration, double min_dist, double tol, std::optional<double> hostart = std::nullopt, uint rec = DubinsDistDefaultRec) const;

    /**
     * @brief A static version of `XY_distance_to`
     *
     */
    static std::pair<double, double> compute_XY_distance(const Dubins &first, const Dubins &second, const AircraftStats& first_stats, const AircraftStats& second_stats,
                                                         double duration, double min_dist,
                                                         double tol = DubinsFleetPlanner_PRECISION,
                                                         std::optional<double> hotstart = std::nullopt,
                                                         uint rec = DubinsDistDefaultRec)
    {
        return first.XY_distance_to(second, first_stats, second_stats, duration, min_dist, tol, hotstart, rec);
    }

    /**
     * @brief Generic type for distance function
     *
     * The arguments are:
     * First path, second path, first stats, second stats, duration, minimal distance, computation precision, optional hot start, heuristic recursion
     *
     * Returns the location and value of the minimal distance between trajectories
     */
    typedef std::pair<double, double> (*DubinsDistanceFunction)(const Dubins &, const Dubins &,
                                                                const AircraftStats&, const AircraftStats&, double, double, double, std::optional<double>, uint);

    /**
     * @brief Check whether `this` and `other` are horitzontaly separated
     *
     *
     * @param other A second Dubins path
     * @param this_stats    Aircraft stats along `this` path
     * @param other_stats   Aircraft stats along `other` path
     * @param duration Duration for which to look for conflicts (in [time])
     * @param min_dist The minimal distance required for ensuring separation (in [time], to be multiplied by the aircraft's speed)
     * @param tol      Solver tolerance used when looking for the minimal distance
     * @param hostart  Optional value for hotstarting the search: a place where the minimal separation might fail
     * @param rec      Depth of the recursive call to the fast but inexact geometric approximation
     * @return true  The two trajectories are well separated
     * @return false The two trajectories are not separated
     */
    bool is_XY_separated_from(const Dubins &other, const AircraftStats& this_stats, const AircraftStats& other_stats,
                              double duration, double min_dist,
                              double tol = DubinsFleetPlanner_PRECISION,
                              std::optional<double> hotstart = std::nullopt,
                              uint rec = DubinsDistDefaultRec) const;

    /**
     * @brief A static version of `is_XY_separated_from`
     *
     * @param first         First path
     * @param second        Second path
     * @param first_stats   Aircraft stats along `first` path
     * @param second_stats  Aircraft stats along `second` path
     * @param duration      Duration for which to look for conflicts (in [time])
     * @param min_dist      The minimal distance required for ensuring separation (in [time], to be multiplied by the aircraft's speed)
     * @param tol           Solver tolerance used when looking for the minimal distance
     * @param rec           Depth of the recursive call to the fast but inexact geometric approximation
     * @return true         The two trajectories are well separated
     * @return false        The two trajectories are not separated
     */
    static bool are_XY_separated(const Dubins &first, const Dubins &second, const AircraftStats& first_stats, const AircraftStats& second_stats,
                                 double duration, double min_dist,
                                 double tol = DubinsFleetPlanner_PRECISION,
                                 uint rec = DubinsDistDefaultRec)
    {
        return first.is_XY_separated_from(second, first_stats, second_stats, duration, min_dist, tol, std::nullopt, rec);
    }

    /**
     * @brief Check wether two Dubins paths are separated by using sampling
     *
     * @param first         First path
     * @param second        Second path
     * @param first_speed   First path speed
     * @param second_speed  Second path speed
     * @param duration      Duration over which to check for separation
     * @param min_dist      Minimal distance required, as a time unit (to be multiplied by the aircraft's speed)
     * @param tol           Distance between two successive time samples
     * @param rec           UNUSED, required for compatibility with other functions
     * @return true         The two trajectories are separated
     * @return false        There is a conflict
     */
    static bool are_XY_separated_sampling(const Dubins &first, const Dubins &second, const AircraftStats& first_stats, const AircraftStats& second_stats,
                                          double duration, double min_dist,
                                          double tol, [[maybe_unused]] uint rec)
    {
        double t = 0.;
        double this_duration = first.get_duration(first_stats);
        double other_duration = second.get_duration(second_stats);
        while (t < duration)
        {
            Pose3D p1 = first.get_position(length_traveled(first_stats, t, this_duration));
            Pose3D p2 = second.get_position(length_traveled(second_stats, t, other_duration));

            double dx = p1.x - p2.x;
            double dy = p1.y - p2.y;

            if (dx * dx + dy * dy < min_dist * min_dist)
            {
                return false;
            }

            t += tol;
        }

        return true;
    }

    /**
     * @brief Generic type for separation function
     *
     * The arguments are:
     * First path, second path, first aircraft stats, second aircraft stats, duration, minimal distance (as time unit, to be multiplied by the aircraft's speed), computation precision, heuristic recursion
     */
    typedef bool (*DubinsSeparationFunction)(const Dubins &, const Dubins &, const AircraftStats&, const AircraftStats&, double, double, double, uint);

    /**
     * @brief Generic type for path generation function (minimal turn radius)
     *
     * The arguments are:
     * Aircraft stats, start pose, end pose, endpoints dist tolerance
     *
     * Returns the list of valid paths as unique_ptr
     */
    typedef std::vector<std::unique_ptr<Dubins>> (*PathGeneratorFunction)(const AircraftStats&, const Pose3D &, const Pose3D &, double);

    /**
     * @brief Generic type for fitted path generation function
     *
     * The arguments are:
     * Aircraft stats, start pose, end pose, target length, length (and endpoints dist) tolerance
     *
     * Returns the list of valid paths as unique_ptr
     */
    typedef std::vector<std::unique_ptr<Dubins>> (*FittedPathGeneratorFunction)(const AircraftStats&, const Pose3D &, const Pose3D &, double, double);
};

Dubins join_paths(const Dubins &first, const Dubins &second, double precision_tol = DubinsFleetPlanner_PRECISION);