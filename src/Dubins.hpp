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
#include "SpeedProfile.hpp"

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
    SpeedProfile speed_profile;

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
            Pose3D end_check = get_position_at_length(length);
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

    Dubins() : start(UndefPose),end(UndefPose),sections({}),length(NAN),speed_profile(1.,0.) {}

    Dubins(const Pose3D &_start, const Pose3D &_end, std::vector<DynamicPathShape> _sections, const SpeedProfile& prof = SpeedProfile(1.,0.))
        : start(_start), end(_end), sections(_sections), speed_profile(prof)
    {
        recompute();
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(pose_dist(_sections.front().initial_pose(), _start) < DubinsFleetPlanner_PRECISION);
        assert(pose_dist(_sections.back().final_pose(), _end) < DubinsFleetPlanner_PRECISION);

        assert(std::abs(pose_aligned(_sections.front().initial_pose(), _start)) < DubinsFleetPlanner_PRECISION);
        assert(std::abs(pose_aligned(_sections.back().final_pose(), _end)) < DubinsFleetPlanner_PRECISION);
#endif
    }

    /************************ Getters ************************/

    Pose3D get_start() const { return start; }
    Pose3D get_end() const { return end; }

    double get_length() const { return length; }
    bool is_valid() const { return valid; }
    double get_speed() const { return speed_profile.base_speed; }
    double set_speed(double new_speed) { speed_profile.base_speed = new_speed; return speed_profile.base_speed; }
    double get_acceleration() const { return speed_profile.acceleration; }
    double set_acceleration(double new_acceleration) { speed_profile.acceleration = new_acceleration; return speed_profile.acceleration; }

    double get_speed_at_time(double time) const
    {
        return speed_profile.get_speed_at_time(time);
    }

    double get_length_at_time(double time) const
    {
        return speed_profile.get_length_at_time(time);
    }

    double get_time_at_length(double len) const
    {
        return speed_profile.get_time_at_length(len);
    }

    double get_duration() const
    {
        return get_time_at_length(length);
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
    Pose3D get_position_at_length(double len) const;

    /**
     * @brief Get the position at the given time, assuming linearly changing speed according to the aircraft's stats.
     *
     * @param time
     * @param stats Aircraft stats
     * @return Pose3D
     */
    Pose3D get_position_at_time(double time) const
    {
        return get_position_at_length(get_length_at_time(time));
    }

    /**
     * @brief Get positions at several lengths
     *
     * @param lens Length abscissas
     * @param sorted If True, assume lengths are sorted (speed up computations)
     * @return const std::vector<Pose3D>
     */
    const std::vector<Pose3D> get_positions_at_lengths(std::vector<double> &lens, bool sorted = false) const;

    /**
     * @brief Get positions at several times, assuming linearly changing speed according to the aircraft's stats.
     *
     * @param times
     * @param stats Aircraft stats
     * @param sorted If True, assume times are sorted (speed up computations)
     * @return const std::vector<Pose3D>
     */
    const std::vector<Pose3D> get_positions_at_times(std::vector<double> &times, bool sorted = false) const
    {
        std::vector<double> lens(times);
        for (double &e : lens)
        {
            e = get_length_at_time(e);
        }
        return get_positions_at_lengths(lens, sorted);
    }

    /**
     * @brief Get the path shape at the given length (object representing the FULL section)
     *
     * @param len
     * @return DynamicPathShape
     */
    DynamicPathShape get_pathshape_at_length(double len) const
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
    DubinsMove get_section_type_at_length(double len) const
    {
        return get_pathshape_at_length(len).m;
    }

    /**
     * @brief Get a new Dubins path that follows this one for a given length
     *
     * @param len Length to follow
     * @return Dubins
     */
    Dubins follow_for_length(double len) const;

    /**
     * @brief Get a new Dubins path that follows this one for a given time at a given speed
     *
     * @param time Time to follow
     * @param stats Aircraft stats
     * @return Dubins
     */
    Dubins follow_for_duration(double time) const
    {
        return follow_for_length(get_length_at_time(time));
    }

    /************************ Seperation functions ************************/

    /**
     * @brief Compute the first point such that 'this' and 'that' breach their minimal separation,
     * or the minimal distance between the two proving that they are separated
     *
     *
     * @param other A second Dubins path
     * @param duration Duration for which to look for conflicts (in [time])
     * @param min_dist The minimal distance required for ensuring separation (in [time], to be multiplied by the aircraft's speed)
     * @param tol      Solver tolerance used when looking for the minimal distance
     * @param hostart  Optional value for hotstarting the search: a place where the minimal separation might fail
     * @param rec      Depth of the recursive call to the fast but inexact geometric approximation
     * @return A pair containing the location and value of either:
     *                  - The first trajectory part breaching the minimal distance
     *                  - The global minimal distance between the two
     */
    std::pair<double, double> XY_distance_to(const Dubins &other,
                                             double duration, double min_dist, double tol, std::optional<double> hostart = std::nullopt, uint rec = DubinsDistDefaultRec) const;

    /**
     * @brief A static version of `XY_distance_to`
     *
     */
    static std::pair<double, double> compute_XY_distance(const Dubins &first, const Dubins &second,
                                                         double duration, double min_dist,
                                                         double tol = DubinsFleetPlanner_PRECISION,
                                                         std::optional<double> hotstart = std::nullopt,
                                                         uint rec = DubinsDistDefaultRec)
    {
        return first.XY_distance_to(second, duration, min_dist, tol, hotstart, rec);
    }

    static std::pair<double, double> sampling_XY_distance(const Dubins &first, const Dubins &second,
                                                         double duration, [[maybe_unused]] double min_dist,
                                                         double tol = DubinsFleetPlanner_PRECISION,
                                                         [[maybe_unused]] std::optional<double> hotstart = std::nullopt,
                                                         [[maybe_unused]] uint rec = DubinsDistDefaultRec)
    {
        std::vector<double> times;
        double t = 0.;
        while(t < duration)
        {
            times.push_back(t);
            t += tol;
        }
        std::vector<Pose3D> first_poses = first.get_positions_at_times(times, true);
        std::vector<Pose3D> second_poses = second.get_positions_at_times(times, true);
        double mTime = 0.;
        double mDist = pose_dist_XY(first_poses.front(),second_poses.front());

        for(int i = 1; i < times.size(); i++)
        {
            double dist = pose_dist_XY(first_poses[i], second_poses[i]);
            if (dist < mDist)
            {
                mTime = times[i];
                mDist = dist;
            }
        }

        return {mTime, mDist};
    }

    /**
     * @brief Compute the time and value of the minimal time-to-collision between two Dubins paths
     *
     * @param first The first Dubins path
     * @param second The second Dubins path
     * @param duration The duration of the paths
     * @param min_tdist The minimum time-to-collision distance
     * @param tol The time step between samples
     * @param hotstart UNUSED (kept for compatibility)
     * @param rec      UNUSED (kept for compatibility)
     *
     * @return A pair containing the time and value of the minimum time-to-collision
     */
    static std::pair<double, double> sampling_XY_tdistance(const Dubins &first, const Dubins &second, 
                                                         double duration, double min_tdist,
                                                         double tol = DubinsFleetPlanner_PRECISION,
                                                         [[maybe_unused]] std::optional<double> hotstart = std::nullopt,
                                                         [[maybe_unused]] uint rec = DubinsDistDefaultRec)
    {
        std::vector<double> times;
        double t = 0.;
        while(t < duration)
        {
            times.push_back(t);
            t += tol;
        }
        std::vector<Pose3D> first_poses = first.get_positions_at_times(times, true);
        std::vector<Pose3D> second_poses = second.get_positions_at_times(times, true);
        double mTime = 0.;
        double mVal = std::pow(pose_dist_XY(first_poses.front(),second_poses.front()), 2) - std::pow(min_tdist * std::max(first.get_speed(), second.get_speed()), 2);

        for(int i = 1; i < times.size(); i++)
        {
            double dx = first_poses[i].x - second_poses[i].x;
            double dy = first_poses[i].y - second_poses[i].y;

            double v1 = first.get_speed_at_time(times[i]);
            double v2 = second.get_speed_at_time(times[i]);
            double vMax = std::max(v1, v2);

            double val = dx * dx + dy * dy - min_tdist * min_tdist * vMax * vMax;

            if (val < mVal)
            {
                mTime = times[i];
                mVal = val;
            }
        }

        return {mTime, mVal};
    }

    /**
     * @brief Generic type for distance function
     *
     * The arguments are:
     * First path, second path, duration, minimal distance, computation precision, optional hot start, heuristic recursion
     *
     * Returns the location and value of the minimal distance between trajectories
     */
    typedef std::pair<double, double> (*DubinsDistanceFunction)(const Dubins &, const Dubins &, double, double, double, std::optional<double>, uint);

    /**
     * @brief Check whether `this` and `other` are horitzontaly separated
     *
     *
     * @param other A second Dubins path
     * @param duration Duration for which to look for conflicts (in [time])
     * @param min_dist The minimal distance required for ensuring separation (in [time], to be multiplied by the aircraft's speed)
     * @param tol      Solver tolerance used when looking for the minimal distance
     * @param hostart  Optional value for hotstarting the search: a place where the minimal separation might fail
     * @param rec      Depth of the recursive call to the fast but inexact geometric approximation
     * @return true  The two trajectories are well separated
     * @return false The two trajectories are not separated
     */
    bool is_XY_separated_from(const Dubins &other, 
                              double duration, double min_dist,
                              double tol = DubinsFleetPlanner_PRECISION,
                              std::optional<double> hotstart = std::nullopt,
                              uint rec = DubinsDistDefaultRec) const;

    /**
     * @brief A static version of `is_XY_separated_from`
     *
     * @param first         First path
     * @param second        Second path
     * @param duration      Duration for which to look for conflicts (in [time])
     * @param min_dist      The minimal distance required for ensuring separation (in [time], to be multiplied by the aircraft's speed)
     * @param tol           Solver tolerance used when looking for the minimal distance
     * @param rec           Depth of the recursive call to the fast but inexact geometric approximation
     * @return true         The two trajectories are well separated
     * @return false        The two trajectories are not separated
     */
    static bool are_XY_separated(const Dubins &first, const Dubins &second,
                                 double duration, double min_dist,
                                 double tol = DubinsFleetPlanner_PRECISION,
                                 uint rec = DubinsDistDefaultRec)
    {
        return first.is_XY_separated_from(second, duration, min_dist, tol, std::nullopt, rec);
    }

    /**
     * @brief Check wether two Dubins paths are separated by using sampling
     *
     * @param first         First path
     * @param second        Second path
     * @param duration      Duration over which to check for separation
     * @param min_dist      Minimal distance required, as a time unit (to be multiplied by the aircraft's speed)
     * @param tol           Delta t for the sampling. The smaller, the more precise but the slower.
     * @param rec           UNUSED, required for compatibility with other functions
     * @return true         The two trajectories are separated
     * @return false        There is a conflict
     */
    static bool are_XY_separated_sampling(const Dubins &first, const Dubins &second,
                                          double duration, double min_dist,
                                          double tol, [[maybe_unused]] uint rec)
    {
        std::pair<double,double> result = sampling_XY_distance(first, second, duration, min_dist, tol);
        return result.second >= min_dist;
    }

    /**
     * @brief Check wether two Dubins paths are separated by a minimum time distance
     * 
     * @param first         First path
     * @param second        Second path
     * @param duration      Duration over which to check for separation
     * @param min_tdist     Minimal time distance required
     * @param tol           Tolerance for the time distance
     * @param rec           Recursive depth for the geometric approximation
     * @return true         The two trajectories are separated
     * @return false        There is a conflict
     */
    static bool are_XY_tdist_separated(const Dubins &first, const Dubins &second,
                                 double duration, double min_tdist,
                                 double tol = DubinsFleetPlanner_PRECISION,
                                 uint rec = DubinsDistDefaultRec);

    /**
     * @brief Check wether two Dubins paths are separated by using sampling
     *
     * @param first         First path
     * @param second        Second path
     * @param duration      Duration over which to check for separation
     * @param min_tdist     Minimal time distance required
     * @param tol           Delta t for the sampling. The smaller, the more precise but the slower.
     * @param rec           UNUSED, required for compatibility with other functions
     * @return true         The two trajectories are separated
     * @return false        There is a conflict
     */
    static bool are_XY_tdist_separated_sampling(const Dubins &first, const Dubins &second,
                                          double duration, double min_tdist,
                                          double tol, [[maybe_unused]] uint rec)
    {
        std::pair<double,double> result = sampling_XY_tdistance(first, second, duration, min_tdist, tol);
        return result.second >= 0.;
    }

    /**
     * @brief Generic type for separation function
     *
     * The arguments are:
     * First path, second path, duration, minimal distance, computation precision, heuristic recursion
     */
    typedef bool (*DubinsSeparationFunction)(const Dubins &, const Dubins &, double, double, double, uint);

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

/**
 * @brief Join paths 'first' and 'second' into a new Dubins path, by following 'first' for its full length, then 'second' for its full length.
 * This requires that the end of 'first' is equal to the start of 'second', up to a precision of 'precision_tol'.
 * Speed and acceleration are not modified, and are taken from 'first'.
 * 
 * @param first 
 * @param second 
 * @param precision_tol 
 * @return Dubins 
 */
Dubins join_paths(const Dubins &first, const Dubins &second, double precision_tol = DubinsFleetPlanner_PRECISION);