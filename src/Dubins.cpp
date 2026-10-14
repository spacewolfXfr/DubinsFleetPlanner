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

#include "Dubins.hpp"

#include "ConflictDetection.hpp"

// ---------- Set up functions ---------- //

void Dubins::recompute()
{
    compute_length();
    junctions_locs.clear();
    junctions.clear();

    double curr_len = 0;
    Pose3D p = start;

    uint N = sections.size();

    for(DynamicPathShape& s : sections)
    {
        s.set_planar_speed(1.);
    }

    for (uint i = 0; i < N - 1; i++)
    {
        const DynamicPathShape &s = sections[i];

        // Vertical control is not handled, make sure everything stays at zero
        // assert(abs(s.p3) < DubinsFleetPlanner_ZERO_TOLERANCE);
        // assert(abs(p.z) < DubinsFleetPlanner_ZERO_TOLERANCE);

        curr_len += s.length;
        junctions_locs.push_back(curr_len);
        p = follow_dubins(s, s.length);
        junctions.push_back(p);
    }
}

// ---------- Compute position ---------- //

Pose3D Dubins::get_position(double len) const
{
    uint i = 0;
    uint N = sections.size();

    while (i < N - 1 && len > junctions_locs[i])
    {
        i++;
    }

    Pose3D p;
    if (i == 0)
    {
        p = start;
    }
    else
    {
        p = junctions[i - 1];
        len -= junctions_locs[i - 1];
    }

    DynamicPathShape s = sections[i];
    return follow_dubins(s, len);
}

const std::vector<Pose3D> Dubins::get_positions(std::vector<double> &lens, bool sorted) const
{
    std::vector<Pose3D> output;
    if (!sorted)
    {
        for (const double &l : lens)
        {
            output.push_back(get_position(l));
        }
    }
    else
    {
        uint i = 0;
        uint N = sections.size();
        for (double len : lens)
        {
            while (i < N - 1 && len > junctions_locs[i])
            {
                i++;
            }

            Pose3D p;
            if (i == 0)
            {
                p = start;
            }
            else
            {
                p = junctions[i - 1];
                len -= junctions_locs[i - 1];
            }

            DynamicPathShape s = sections[i];
            output.push_back(follow_dubins(s, len));
        }
    }
    return output;
}

Dubins Dubins::follow_for(double len) const
{
    if (len < 0)
    {
        throw std::runtime_error("Negative length not supported");
    }

    if (len > get_length())
    {
        throw std::runtime_error("Length exceeds path length");
    }

    uint i = 0;
    uint N = sections.size();

    while (i < N - 1 && len > junctions_locs[i])
    {
        i++;
    }

    Pose3D p;
    if (i == 0)
    {
        p = start;
    }
    else
    {
        p = junctions[i - 1];
        len -= junctions_locs[i - 1];
    }

    DynamicPathShape s = sections[i];
    std::vector<DynamicPathShape> new_sections(sections.begin() + i, sections.end());
    new_sections[0] = shift_start(s, len);

    return Dubins(new_sections[0].initial_pose(), end, new_sections);
}

// ---------- Local template for computing separation between shapes ---------- //

/**
 * @brief Compute the minimal XY distance between two paths elements
 *
 * @tparam use_derivatives Use derivatives based global minimum seeking method. Default to true.
 * @param p1 First path
 * @param p2 Second path
 * @param duration Time duration of the section to study
 * @param min_sep Minial required distance
 * @param tol Precision with respect
 * @return A pair: Location and value of the minimal distance between two paths
 */
template <bool use_derivatives = true>
std::pair<double, double> compute_XY_separation(
    const DynamicPathShape &p1, const DynamicPathShape &p2, const AircraftStats& p1_stats, const AircraftStats& p2_stats,
    double duration, double min_sep, double tol,
    uint rec = DubinsDistDefaultRec)
{
    std::pair<double, double> output;

    double v1_S = speed_at_time(p1_stats,0,duration);
    double v1_M = speed_at_time(p1_stats,duration/2.,duration);
    double v1_E = speed_at_time(p1_stats,duration,duration);

    double v2_S = speed_at_time(p1_stats,0,duration);
    double v2_M = speed_at_time(p1_stats,duration/2.,duration);
    double v2_E = speed_at_time(p1_stats,duration,duration);

    DubinsMove m1 = p1.m;
    DubinsMove m2 = p2.m;

    // Degenerate case: negligible duration -> move to point-point distance
    if (duration < tol)
    {
        Pose3D p2_start = p2.initial_pose();
        Pose3D p1_start = p1.initial_pose();

        output.first = duration / 2.;
        output.second = pose_dist_XY(p1_start, p2_start);

        return output;
    }

    // In the straight-straight case, analytic exact solution exists
    if ((m1 == STRAIGHT) && (m2 == STRAIGHT))
    {
        output = temporal_XY_dist<STRAIGHT, STRAIGHT, true>(
            to_static_shape<STRAIGHT>(p1),
            to_static_shape<STRAIGHT>(p2), p1_stats, p2_stats, duration, tol);
    }
    else
    {
        double geo_dist;
        double t1, t2;
        std::tie(geo_dist, t1, t2) = geometric_XY_dist(p1, p2, duration);

        double v1_max = std::max(v1_S,v1_E);
        double v2_max = std::max(v2_S,v2_E);

        // If there may be a conflict, do a detailed search
        if (geo_dist < min_sep)
        {
            // If recursive computation is enabled, retry computing separation using geometry
            if (rec > 0)
            {
                DynamicPathShape p1_cpy = p1;
                DynamicPathShape p2_cpy = p2;

                path_set_planar_speed(p1_cpy,1.);
                path_set_planar_speed(p2_cpy,1.);

                double l1 = length_traveled(p1_stats,duration/2.,duration);
                double l2 = length_traveled(p2_stats,duration/2.,duration);

                auto shifted_p1 = shift_start(p1_cpy, l1);
                auto shifted_p2 = shift_start(p2_cpy, l2);

                AircraftStats p1_first_half  = p1_stats;
                p1_first_half.target_airspeed = v1_M;
                AircraftStats p2_first_half  = p2_stats;
                p2_first_half.target_airspeed = v2_M;

                AircraftStats p1_second_half = p1_stats;
                p1_second_half.airspeed = v1_M;
                AircraftStats p2_second_half = p2_stats;
                p2_second_half.airspeed = v2_M;

                // If the geometric test suggests there is a collision in the second half, look at the second half first
                if (t1 >= duration / 2 && t2 >= duration / 2)
                {
                    auto rec_output = compute_XY_separation(shifted_p1, shifted_p2, p1_second_half, p2_second_half, duration / 2, min_sep, tol, rec - 1);
                    rec_output.first += duration /2.; // Location needs to be corrected due to shifting


                    // If the recursive search found a conflict, output it
                    if (rec_output.second < min_sep)
                    {
                        return rec_output;
                    }
                    // Otherwise, explore the other branch and output the smallest distance value
                    else
                    {
                        auto output_bis = compute_XY_separation(p1, p2, p1_first_half, p2_first_half, duration / 2, min_sep, tol, rec - 1);
                        if (output_bis.second < rec_output.second)
                        {
                            return output_bis;
                        }
                        else
                        {
                            return rec_output;
                        }
                    }
                }
                else
                {
                    auto rec_output = compute_XY_separation(p1, p2, p1_first_half, p2_first_half, duration / 2, min_sep, tol, rec - 1);

                    // If the recursive search found a conflict, output it
                    if (rec_output.second < min_sep)
                    {
                        return rec_output;
                    }
                    // Otherwise, explore the other branch and output the smallest distance value
                    else
                    {
                        auto output_bis = compute_XY_separation(shifted_p1, shifted_p2, p1_second_half, p2_second_half, duration / 2, min_sep, tol, rec - 1);
                        output_bis.first += duration / 2; // Location needs to be corrected due to shifting
                        if (output_bis.second < rec_output.second)  
                        {
                            return output_bis;
                        }
                        else
                        {
                            return rec_output;
                        }
                    }
                }
            }
            else // Otherwise, fallback on temporal method
            {
                output = temporal_XY_dist<use_derivatives>(p1, p2, p1_stats, p2_stats, duration, tol);
            }
        }
        // Otherwise, use the result of the geometric separation
        else
        {
            output.first = duration / 2.;
            output.second = geo_dist;
        }
    }

    return output;
}

// ---------- Separation between Dubins ---------- //

/**
 * @brief For two Dubins paths, compute the sections identifying the part when we stick to a specific basic path
 *
 * @param p1 One Dubins path
 * @param p2 Other Dubins path
 * @param p1_stats Aircraft stats along first path
 * @param p2_stats Aircraft stats along second path
 * @param duration Duration for which we follow the path
 * @return std::vector<double> List of junctions timestamps
 */
static std::vector<double> compute_timepoints(const Dubins &p1, const Dubins &p2, const AircraftStats& p1_stats, const AircraftStats& p2_stats, double &duration)
{
    std::vector<double> timepoints = {0.};

    std::vector<double> p1_junctions = p1.get_junction_locs();
    std::vector<double> p2_junctions = p2.get_junction_locs();
    double p1_duration = p1.get_duration(p1_stats);
    double p2_duration = p2.get_duration(p2_stats);

    std::vector<double> p1_junctions_times;
    for(const double &l : p1_junctions)
    {
        p1_junctions_times.push_back(time_to_travel_length(p1_stats, l, p1_duration));
    }

    std::vector<double> p2_junctions_times;
    for(const double &l : p2_junctions)
    {
        p2_junctions_times.push_back(time_to_travel_length(p2_stats, l, p2_duration));
    }

    size_t p1_index = 0;
    size_t p2_index = 0;

    // Merge the junctions points lists by timestamp until total duration is met
    while ((p1_index < p1_junctions.size()) && (p2_index < p2_junctions.size()))
    {
        double this_candidate = p1_junctions_times[p1_index];
        double other_candidate = p2_junctions_times[p2_index];
        if ((this_candidate > duration) && (other_candidate > duration))
        {
            break;
        }
        if (this_candidate < other_candidate)
        {
            timepoints.push_back(this_candidate);
            p1_index++;
        }
        else
        {
            timepoints.push_back(other_candidate);
            p2_index++;
        }
    }

    while ((p1_index < p1_junctions.size()) && (p1_junctions_times[p1_index] < duration))
    {
        timepoints.push_back(p1_junctions_times[p1_index]);
        p1_index++;
    }

    while ((p2_index < p2_junctions.size()) && (p2_junctions_times[p2_index] < duration))
    {
        timepoints.push_back(p2_junctions_times[p2_index]);
        p2_index++;
    }

    timepoints.push_back(duration);
    return timepoints;
}


std::pair<double, double> Dubins::XY_distance_to(const Dubins &other, const AircraftStats& this_stats, const AircraftStats& other_stats,
                                                 double duration, double min_dist, double tol, std::optional<double> hotstart, uint rec) const
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert((this_speed > 0) && (other_speed > 0));
    assert((this->is_valid()) && (other.is_valid()));
#endif

    std::vector<double> timepoints = compute_timepoints(*this, other, this_stats, other_stats, duration);
    std::pair<double, double> output = {0., INFINITY};

    size_t index; // Path section index for hot start

    if (hotstart.has_value())
    {
        index = binary_search(timepoints, hotstart.value());
        double sec_duration = timepoints[index + 1] - timepoints[index];

        std::vector<double> times{timepoints[index], timepoints[index + 1]};

        const DynamicPathShape &curr_ref_shape = this->get_pathshape((times.front() + times.back()) / 2);
        const DynamicPathShape &other_ref_shape = other.get_pathshape((times.front() + times.back()) / 2);

        std::vector<Pose3D> curr_poses = this->get_positions(times, this_stats, true);
        std::vector<Pose3D> other_poses = other.get_positions(times, other_stats, true);

        DynamicPathShape curr_shape(curr_ref_shape.m, curr_poses.front(), curr_poses.back(),
                                    1., curr_ref_shape.p1, curr_ref_shape.p3);

        DynamicPathShape other_shape(other_ref_shape.m, other_poses.front(), other_poses.back(),
                                     1., other_ref_shape.p1, other_ref_shape.p3);


        AircraftStats curr_stats    = this_stats;
        AircraftStats other_stats   = other_stats;

        curr_stats.airspeed  = speed_at_time(this_stats, timepoints[index], duration);
        other_stats.airspeed = speed_at_time(other_stats, timepoints[index], duration);

        curr_stats.target_airspeed  = speed_at_time(this_stats, timepoints[index + 1], duration);
        other_stats.target_airspeed = speed_at_time(other_stats, timepoints[index + 1], duration);                     
        std::pair<double, double> current_sep = compute_XY_separation(curr_shape, other_shape, curr_stats, other_stats,sec_duration, min_dist, tol, rec);

        if (current_sep.second < min_dist)
        {
            return current_sep;
        }
        else
        {
            output = current_sep;
        }
    }
    else
    {
        index = timepoints.size(); // Return past-the-post value to avoid skipping
    }

    std::vector<Pose3D> curr_poses = this->get_positions(timepoints, this_stats, true);
    std::vector<Pose3D> other_poses = other.get_positions(timepoints, other_stats, true);

    std::vector<double> curr_endlocs = this->get_endpoints_locs();
    std::vector<double> other_endlocs = other.get_endpoints_locs();

    uint curr_sec_index = 0;
    uint other_sec_index = 0;

    // For each base case, compute the distance
    for (size_t i = 0; i < timepoints.size() - 1; i++)
    {
        // Update section tracker
        if (curr_endlocs[curr_sec_index + 1] < length_traveled(this_stats,timepoints[i + 1],duration) * (1 - 1e-6))
        {
            curr_sec_index++;
        }

        if (other_endlocs[other_sec_index + 1] < length_traveled(other_stats,timepoints[i + 1],duration) * (1 - 1e-6))
        {
            other_sec_index++;
        }

        // Skip the hot start
        if (i == index)
        {
            continue;
        }

        const DynamicPathShape &curr_ref_shape = sections[curr_sec_index];
        const DynamicPathShape &other_ref_shape = other.sections[other_sec_index];

        DynamicPathShape curr_shape(curr_ref_shape.m, curr_poses[i], curr_poses[i + 1],
                                    1., curr_ref_shape.p1, curr_ref_shape.p3);

        DynamicPathShape other_shape(other_ref_shape.m, other_poses[i], other_poses[i + 1],
                                     1., other_ref_shape.p1, other_ref_shape.p3);

        AircraftStats curr_stats    = this_stats;
        AircraftStats other_stats   = other_stats;

        curr_stats.airspeed  = speed_at_time(this_stats, timepoints[i], duration);
        other_stats.airspeed = speed_at_time(other_stats, timepoints[i], duration);

        curr_stats.target_airspeed  = speed_at_time(this_stats, timepoints[i + 1], duration);
        other_stats.target_airspeed = speed_at_time(other_stats, timepoints[i + 1], duration);

        std::pair<double, double> current_sep = compute_XY_separation(curr_shape, other_shape, curr_stats, other_stats, (timepoints[i + 1] - timepoints[i]), min_dist, tol, rec);

        if (current_sep.second < min_dist)
        {
            return current_sep;
        }
        else if (current_sep.second < output.second)
        {
            output = current_sep;
        }
    }

    return output;
}

bool Dubins::is_XY_separated_from(const Dubins &other, const AircraftStats& this_stats, const AircraftStats& other_stats,
                                  double duration, double min_dist, double tol, std::optional<double> hotstart, uint rec) const
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert((this_speed > 0) && (other_speed > 0));
    assert((this->is_valid()) && (other.is_valid()));
#endif

    std::vector<double> timepoints = compute_timepoints(*this, other, this_stats, other_stats, duration);
    std::pair<double, double> output = {0., INFINITY};

    size_t index; // Path section index for hot start

    if (hotstart.has_value())
    {
        index = binary_search(timepoints, hotstart.value());
        double duration = timepoints[index + 1] - timepoints[index];

        std::vector<double> times{timepoints[index], timepoints[index + 1]};

        const DynamicPathShape &curr_ref_shape = this->get_pathshape((times.front() + times.back()) / 2);
        const DynamicPathShape &other_ref_shape = other.get_pathshape((times.front() + times.back()) / 2);

        std::vector<Pose3D> curr_poses = this->get_positions(times, this_stats, true);
        std::vector<Pose3D> other_poses = other.get_positions(times, other_stats, true);

        DynamicPathShape curr_shape(curr_ref_shape.m, curr_poses.front(), curr_poses.back(),
                                    1., curr_ref_shape.p1, curr_ref_shape.p3);

        DynamicPathShape other_shape(other_ref_shape.m, other_poses.front(), other_poses.back(),
                                     1., other_ref_shape.p1, other_ref_shape.p3);

        AircraftStats curr_stats    = this_stats;
        AircraftStats other_stats   = other_stats;

        curr_stats.airspeed  = speed_at_time(this_stats, times.front(), duration);
        other_stats.airspeed = speed_at_time(other_stats, times.front(), duration);

        curr_stats.target_airspeed  = speed_at_time(this_stats, times.back(), duration);
        other_stats.target_airspeed = speed_at_time(other_stats, times.back(), duration);

        std::pair<double, double> current_sep = compute_XY_separation(curr_shape, other_shape, curr_stats, other_stats, duration, min_dist, tol, rec);

        if (current_sep.second < min_dist)
        {
            return false;
        }
    }
    else
    {
        index = timepoints.size(); // Return past-the-post value to avoid skipping
    }

    std::vector<Pose3D> curr_poses = this->get_positions(timepoints, this_stats, true);
    std::vector<Pose3D> other_poses = other.get_positions(timepoints, other_stats, true);

    std::vector<double> curr_endlocs = this->get_endpoints_locs();
    std::vector<double> other_endlocs = other.get_endpoints_locs();

    uint curr_sec_index = 0;
    uint other_sec_index = 0;

    // For each base case, compute the distance
    for (size_t i = 0; i < timepoints.size() - 1; i++)
    {
        // Update section tracker
        if (curr_endlocs[curr_sec_index + 1] < length_traveled(this_stats,timepoints[i + 1],duration) * (1 - 1e-6))
        {
            curr_sec_index++;
        }

        if (other_endlocs[other_sec_index + 1] < length_traveled(other_stats,timepoints[i + 1],duration) * (1 - 1e-6))
        {
            other_sec_index++;
        }

        // Skip the hot start
        if (i == index)
        {
            continue;
        }

        const DynamicPathShape &curr_ref_shape = sections[curr_sec_index];
        const DynamicPathShape &other_ref_shape = other.sections[other_sec_index];

        DynamicPathShape curr_shape(curr_ref_shape.m, curr_poses[i], curr_poses[i + 1],
                                    1., curr_ref_shape.p1, curr_ref_shape.p3);

        DynamicPathShape other_shape(other_ref_shape.m, other_poses[i], other_poses[i + 1],
                                     1., other_ref_shape.p1, other_ref_shape.p3);

        AircraftStats curr_stats    = this_stats;
        AircraftStats other_stats   = other_stats;

        curr_stats.airspeed  = speed_at_time(this_stats, timepoints[i], duration);
        other_stats.airspeed = speed_at_time(other_stats, timepoints[i], duration);

        curr_stats.target_airspeed  = speed_at_time(this_stats, timepoints[i + 1], duration);
        other_stats.target_airspeed = speed_at_time(other_stats, timepoints[i + 1], duration);

        std::pair<double, double> current_sep = compute_XY_separation(curr_shape, other_shape, curr_stats, other_stats, (timepoints[i + 1] - timepoints[i]), min_dist, tol, rec);

        if (current_sep.second < min_dist)
        {
            return false;
        }
    }

    return true;
}

Dubins join_paths(const Dubins &first, const Dubins &second, double precision_tol)
{
    if (!first.is_valid() || !second.is_valid())
    {
        throw std::runtime_error("Cannot join invalid Dubins paths");
    }

    if (pose_dist(first.get_end(), second.get_start()) > precision_tol || std::abs(first.get_end().theta - second.get_start().theta) > precision_tol)
    {
        throw std::runtime_error("Cannot join Dubins paths: end of first path does not match start of second path");
    }

    auto joined_sections = first.get_all_sections();
    joined_sections.insert(joined_sections.end(), second.get_all_sections().begin(), second.get_all_sections().end());

    Dubins joined(first.get_start(), second.get_end(), joined_sections);
    return joined;
}