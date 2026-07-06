// Copyright (C) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
// 
// This file is part of PH_Spline.
// 
// PH_Spline is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// PH_Spline is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with PH_Spline.  If not, see <https://www.gnu.org/licenses/>.

#include "Dubins.hpp"

#include "ConflictDetection.hpp"

// ---------- Local template for computing separation between shapes ---------- //

/**
 * @brief Check is two basic path elements are separated based on the XY euclidean distance
 * 
 * @tparam m1 Type of the first element
 * @tparam m2 Type of the second element
 * @param p1 First path
 * @param p2 Second path
 * @param duration Time duration of the section to study
 * @param min_sep Minial required distance
 * @param tol Precision with respect
 * @return true Path elements are separated
 * @return false Path are not separated
 */
template<DubinsMove m1, DubinsMove m2>
static bool check_XY_separation(const PathShape<m1>& p1, const PathShape<m2>& p2, double duration, double min_sep, double tol)
{
    if (duration < tol)
    {
        Pose3D p1_start = initial_pose(p1);
        Pose3D p2_start = initial_pose(p2);
        return pose_dist_XY(p1_start,p2_start) > min_sep;
    }

    if ((m1 == STRAIGHT) && (m2 == STRAIGHT))
    {
        auto loc_val_dist = temporal_XY_dist<m1,m2,true>(p1,p2,duration,tol);
        if (loc_val_dist.second < min_sep)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        double geo_dist;
        double t1,t2;
        std::tie(geo_dist,t1,t2) = geometric_XY_dist(p1,p2,duration);

        if (geo_dist < min_sep)
        {
            auto loc_val_dist = temporal_XY_dist<m1,m2,true>(p1,p2,duration,tol);
            if (loc_val_dist.second < min_sep)
            {
                return false;
            }
        }

        return true;
    }
}

/**
 * @brief Compute the minimal XY distance between two paths elements 
 * 
 * @tparam m1 Type of the first element
 * @tparam m2 Type of the second element
 * @tparam use_derivatives Use derivatives based global minimum seeking method. Default to true.
 * @param p1 First path
 * @param p2 Second path
 * @param duration Time duration of the section to study
 * @param min_sep Minial required distance
 * @param tol Precision with respect
 * @return A pair: Location and value of the minimal distance between two paths
 */
template<DubinsMove m1, DubinsMove m2, bool use_derivatives=true>
std::pair<double,double> compute_XY_separation(
    const PathShape<m1>& p1, const PathShape<m2>& p2, double duration, double min_sep, double tol,
    uint rec=DubinsDistDefaultRec)
{
    std::pair<double,double> output;

    // Degenerate case: negligible duration -> move to point-point distance
    if (duration < tol)
    {
        Pose3D p1_start = initial_pose(p1);
        Pose3D p2_start = initial_pose(p2);

        output.first = duration/2.;
        output.second = pose_dist_XY(p1_start,p2_start);

        return output;
    }

    
    // In the straight-straight case, analytic exact solution exists
    if ((m1 == STRAIGHT) && (m2 == STRAIGHT))
    {
        output = temporal_XY_dist<m1,m2,use_derivatives>(p1,p2,duration,tol);
    }
    else
    {
        double geo_dist;
        double t1,t2;
        std::tie(geo_dist,t1,t2) = geometric_XY_dist(p1,p2,duration);

        // If there may be a conflict, do a detailed search
        if (geo_dist < min_sep)
        {
            // If recursive computation is enabled, retry computing separation using geometry
            if (rec > 0)
            {

                auto shifted_p1 = shift_start(p1,duration/2);
                auto shifted_p2 = shift_start(p2,duration/2);


                // If the geometric test suggests there is a collision in the second half, look at the second half first
                if (t1 >= duration/2 && t2 >= duration/2)
                {
                    output = compute_XY_separation(shifted_p1,shifted_p2,duration/2,min_sep,tol,rec-1);
                    output.first += duration/2; // Location needs to be corrected due to shifting

                    // If the recursive search found a conflict, output it
                    if (output.second < min_sep)
                    {
                        return output;
                    }
                    // Otherwise, explore the other branch and output the smallest distance value
                    else
                    {
                        auto output_bis = compute_XY_separation(p1,p2,duration/2,min_sep,tol,rec-1);
                        if (output_bis.second < output.second)
                        {
                            return output_bis;
                        }
                        else
                        {
                            return output;
                        }
                    }
                }
                else
                {
                    output = compute_XY_separation(p1,p2,duration/2,min_sep,tol,rec-1);

                    // If the recursive search found a conflict, output it
                    if (output.second < min_sep)
                    {
                        return output;
                    }
                    // Otherwise, explore the other branch and output the smallest distance value
                    else
                    {
                        auto output_bis = compute_XY_separation(shifted_p1,shifted_p2,duration/2,min_sep,tol,rec-1);
                        output_bis.first += duration/2; // Location needs to be corrected due to shifting
                        if (output_bis.second < output.second)
                        {
                            return output_bis;
                        }
                        else
                        {
                            return output;
                        }
                    }
                }
            }
            else // Otherwise, fallback on temporal method
            {
                output = temporal_XY_dist<m1,m2,use_derivatives>(p1,p2,duration,tol);
            }
        }
        // Otherwise, use the result of the geometric separation
        else
        {
            output.first = duration/2.;
            output.second = geo_dist;
        }
    }

    return output;
}

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
template<bool use_derivatives=true>
std::pair<double,double> compute_XY_separation(
    const DynamicPathShape& p1, const DynamicPathShape& p2, double duration, double min_sep, double tol,
    uint rec=DubinsDistDefaultRec)
{
    std::pair<double,double> output;

    DubinsMove m1 = p1.m;
    DubinsMove m2 = p2.m;

    // Degenerate case: negligible duration -> move to point-point distance
    if (duration < tol)
    {
        Pose3D p2_start = p2.initial_pose();
        Pose3D p1_start = p1.initial_pose();

        output.first = duration/2.;
        output.second = pose_dist_XY(p1_start,p2_start);

        return output;
    }

    
    // In the straight-straight case, analytic exact solution exists    
    if ((m1 == STRAIGHT) && (m2 == STRAIGHT))
    {
        output = temporal_XY_dist<STRAIGHT,STRAIGHT,true>(
            to_static_shape<STRAIGHT>(p1),
            to_static_shape<STRAIGHT>(p2),duration,tol);
    }
    else
    {
        double geo_dist;
        double t1,t2;
        std::tie(geo_dist,t1,t2) = geometric_XY_dist(p1,p2,duration);

        // If there may be a conflict, do a detailed search
        if (geo_dist < min_sep)
        {
            // If recursive computation is enabled, retry computing separation using geometry
            if (rec > 0)
            {
                auto shifted_p1 = shift_start(p1,duration/2);
                auto shifted_p2 = shift_start(p2,duration/2);


                // If the geometric test suggests there is a collision in the second half, look at the second half first
                if (t1 >= duration/2 && t2 >= duration/2)
                {
                    output = compute_XY_separation(shifted_p1,shifted_p2,duration/2,min_sep,tol,rec-1);
                    output.first += duration/2; // Location needs to be corrected due to shifting

                    // If the recursive search found a conflict, output it
                    if (output.second < min_sep)
                    {
                        return output;
                    }
                    // Otherwise, explore the other branch and output the smallest distance value
                    else
                    {
                        auto output_bis = compute_XY_separation(p1,p2,duration/2,min_sep,tol,rec-1);
                        if (output_bis.second < output.second)
                        {
                            return output_bis;
                        }
                        else
                        {
                            return output;
                        }
                    }
                }
                else
                {
                    output = compute_XY_separation(p1,p2,duration/2,min_sep,tol,rec-1);

                    // If the recursive search found a conflict, output it
                    if (output.second < min_sep)
                    {
                        return output;
                    }
                    // Otherwise, explore the other branch and output the smallest distance value
                    else
                    {
                        auto output_bis = compute_XY_separation(shifted_p1,shifted_p2,duration/2,min_sep,tol,rec-1);
                        output_bis.first += duration/2; // Location needs to be corrected due to shifting
                        if (output_bis.second < output.second)
                        {
                            return output_bis;
                        }
                        else
                        {
                            return output;
                        }
                    }
                }
            }
            else // Otherwise, fallback on temporal method
            {
                output = temporal_XY_dist<use_derivatives>(p1,p2,duration,tol);
            }
        }
        // Otherwise, use the result of the geometric separation
        else
        {
            output.first = duration/2.;
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
 * @param p1_speed Speed along first path
 * @param p2_speed Speed along second path
 * @param duration Duration for which we follow the path
 * @return std::vector<double> List of junctions timestamps
 */
static std::vector<double> compute_timepoints(const Dubins &p1, const Dubins &p2, double p1_speed, double p2_speed, double &duration)
{
    std::vector<double> timepoints = {0.};
    
    std::vector<double> this_junctions = p1.get_junction_locs();
    std::vector<double> other_junctions = p2.get_junction_locs();
    size_t this_index = 0;
    size_t other_index = 0;

    // Merge the junctions points lists by timestamp until total duration is met
    while ((this_index < this_junctions.size()) && (other_index < other_junctions.size()))
    {
        double this_candidate = this_junctions[this_index] / p1_speed;
        double other_candidate = other_junctions[other_index] / p2_speed;
        if ((this_candidate > duration) && (other_candidate > duration))
        {
            break;
        }
        if (this_candidate < other_candidate)
        {
            timepoints.push_back(this_candidate);
            this_index++;
        }
        else
        {
            timepoints.push_back(other_candidate);
            other_index++;
        }
    }

    while ((this_index < this_junctions.size()) && (this_junctions[this_index] / p1_speed < duration))
    {
        timepoints.push_back(this_junctions[this_index] / p1_speed);
        this_index++;
    }

    while ((other_index < other_junctions.size()) && (other_junctions[other_index] / p2_speed < duration))
    {
        timepoints.push_back(other_junctions[other_index] / p2_speed);
        other_index++;
    }

    timepoints.push_back(duration);
    return timepoints;
}

template<DubinsMove m1, DubinsMove m2>
static std::pair<double,double> compute_XY_separation_between_trajectories(
    const Pose3D& start_1, const Pose3D& end_1, double v1, double r1, double h1,
    const Pose3D& start_2, const Pose3D& end_2, double v2, double r2, double h2,
    double duration, double min_dist, double tol, uint rec
)
{
    return compute_XY_separation<m1,m2>(
                    compute_params<m1>(start_1,end_1,v1,r1,h1),
                    compute_params<m2>(start_2,end_2,v2,r2,h2),
                    duration,min_dist,tol,rec);
}


std::pair<double,double> Dubins::XY_distance_to(const Dubins& other, double this_speed, double other_speed, 
        double duration, double min_dist, double tol, std::optional<double> hotstart, uint rec) const
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert((this_speed > 0) && (other_speed > 0));
    assert((this->is_valid()) && (other.is_valid()));
#endif

    std::vector<double> timepoints = compute_timepoints(*this, other, this_speed, other_speed, duration);
    std::pair<double,double> output = {0.,INFINITY};


    size_t index; // Path section index for hot start

    if (hotstart.has_value())
    {
        index = binary_search(timepoints,hotstart.value());
        double duration = timepoints[index+1]-timepoints[index];

        std::vector<double> times{timepoints[index],timepoints[index+1]};

        const DynamicPathShape& curr_ref_shape  = this->get_pathshape((times.front()+times.back())/2);
        const DynamicPathShape& other_ref_shape = other.get_pathshape((times.front()+times.back())/2);

        std::vector<Pose3D> curr_poses  = this->get_positions(times,this_speed,true);
        std::vector<Pose3D> other_poses = other.get_positions(times,other_speed,true);

        DynamicPathShape curr_shape(curr_ref_shape.m,curr_poses.front(),curr_poses.back(),
            this_speed,curr_ref_shape.p1,curr_ref_shape.p3);

        DynamicPathShape other_shape(other_ref_shape.m,other_poses.front(),other_poses.back(),
            other_speed,other_ref_shape.p1,other_ref_shape.p3);

        std::pair<double,double> current_sep = compute_XY_separation(curr_shape,other_shape,duration,min_dist,tol,rec);

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

    std::vector<Pose3D> curr_poses = this->get_positions(timepoints,this_speed,true);
    std::vector<Pose3D> other_poses = other.get_positions(timepoints,other_speed,true);

    std::vector<double> curr_endlocs = this->get_endpoints_locs();
    std::vector<double> other_endlocs = other.get_endpoints_locs();

    uint curr_sec_index = 0;
    uint other_sec_index = 0;

    // For each base case, compute the distance
    for(size_t i = 0; i < timepoints.size()-1; i++)
    {
        // Update section tracker
        if (curr_endlocs[curr_sec_index+1] < timepoints[i+1]*this_speed*(1-1e-6))
        {
            curr_sec_index++;
        }

        if (other_endlocs[other_sec_index+1] < timepoints[i+1]*other_speed*(1-1e-6))
        {
            other_sec_index++;
        }

        // Skip the hot start
        if (i == index)
        {
            continue;
        }

        const DynamicPathShape& curr_ref_shape = sections[curr_sec_index];
        const DynamicPathShape& other_ref_shape = other.sections[other_sec_index];

        DynamicPathShape curr_shape(curr_ref_shape.m,curr_poses[i], curr_poses[i+1],
            this_speed, curr_ref_shape.p1, curr_ref_shape.p3);

        DynamicPathShape other_shape(other_ref_shape.m,other_poses[i], other_poses[i+1],
            other_speed, other_ref_shape.p1, other_ref_shape.p3);

        std::pair<double,double> current_sep = compute_XY_separation(curr_shape,other_shape,(timepoints[i+1] - timepoints[i]),min_dist,tol,rec);

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

bool Dubins::is_XY_separated_from(const Dubins& other, double this_speed, double other_speed, 
        double duration, double min_dist, double tol, std::optional<double> hotstart, uint rec) const
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert((this_speed > 0) && (other_speed > 0));
    assert((this->is_valid()) && (other.is_valid()));
#endif

    std::vector<double> timepoints = compute_timepoints(*this, other, this_speed, other_speed, duration);
    std::pair<double,double> output = {0.,INFINITY};


    size_t index; // Path section index for hot start

    if (hotstart.has_value())
    {
        index = binary_search(timepoints,hotstart.value());
        double duration = timepoints[index+1]-timepoints[index];

        std::vector<double> times{timepoints[index],timepoints[index+1]};

        const DynamicPathShape& curr_ref_shape  = this->get_pathshape((times.front()+times.back())/2);
        const DynamicPathShape& other_ref_shape = other.get_pathshape((times.front()+times.back())/2);

        std::vector<Pose3D> curr_poses  = this->get_positions(times,this_speed,true);
        std::vector<Pose3D> other_poses = other.get_positions(times,other_speed,true);

        DynamicPathShape curr_shape(curr_ref_shape.m,curr_poses.front(),curr_poses.back(),
            this_speed,curr_ref_shape.p1,curr_ref_shape.p3);
        
        DynamicPathShape other_shape(other_ref_shape.m,other_poses.front(),other_poses.back(),
            other_speed,other_ref_shape.p1,other_ref_shape.p3);

        std::pair<double,double> current_sep = compute_XY_separation(curr_shape,other_shape,duration,min_dist,tol,rec);

        if (current_sep.second < min_dist)
        {
            return false;
        }

    }
    else
    {
        index = timepoints.size(); // Return past-the-post value to avoid skipping
    }

    std::vector<Pose3D> curr_poses = this->get_positions(timepoints,this_speed,true);
    std::vector<Pose3D> other_poses = other.get_positions(timepoints,other_speed,true);

    std::vector<double> curr_endlocs = this->get_endpoints_locs();
    std::vector<double> other_endlocs = other.get_endpoints_locs();

    uint curr_sec_index = 0;
    uint other_sec_index = 0;

    // For each base case, compute the distance
    for(size_t i = 0; i < timepoints.size()-1; i++)
    {
        // Update section tracker
        if (curr_endlocs[curr_sec_index+1] < timepoints[i+1]*this_speed*(1-1e-6))
        {
            curr_sec_index++;
        }

        if (other_endlocs[other_sec_index+1] < timepoints[i+1]*other_speed*(1-1e-6))
        {
            other_sec_index++;
        }

        // Skip the hot start
        if (i == index)
        {
            continue;
        }

        const DynamicPathShape& curr_ref_shape = sections[curr_sec_index];
        const DynamicPathShape& other_ref_shape = other.sections[other_sec_index];

        DynamicPathShape curr_shape(curr_ref_shape.m,curr_poses[i], curr_poses[i+1],
            this_speed, curr_ref_shape.p1, curr_ref_shape.p3);

        DynamicPathShape other_shape(other_ref_shape.m,other_poses[i], other_poses[i+1],
            other_speed, other_ref_shape.p1, other_ref_shape.p3);

        std::pair<double,double> current_sep = compute_XY_separation(curr_shape,other_shape,(timepoints[i+1] - timepoints[i]),min_dist,tol,rec);

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