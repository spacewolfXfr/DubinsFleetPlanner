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

/********************  Given radius basic planning  ********************/

std::vector<DynamicPathShape> set_radius_LSL(const Pose3D &start, const Pose3D &end, double turn_radius, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);

    double radius = turn_radius;

    double fst_length = LSL_first_distance(a, b, d / radius) * radius;
    double snd_length = LSL_middle_distance(a, b, d / radius) * radius;
    double trd_length = LSL_last_distance(a, b, d / radius) * radius;

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;
    if (!std::isfinite(length) || length < 0)
    {
        return {};
    }

    Pose3D junc_1 = follow_dubins<LEFT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<STRAIGHT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(LEFT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(STRAIGHT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(LEFT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_radius_RSR(const Pose3D &start, const Pose3D &end, double turn_radius, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = turn_radius;

    double fst_length = RSR_first_distance(a, b, d / radius) * radius;
    double snd_length = RSR_middle_distance(a, b, d / radius) * radius;
    double trd_length = RSR_last_distance(a, b, d / radius) * radius;

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;
    if (!std::isfinite(length) || length < 0)
    {
        return {};
    }

    Pose3D junc_1 = follow_dubins<RIGHT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<STRAIGHT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(RIGHT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(STRAIGHT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(RIGHT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_radius_RSL(const Pose3D &start, const Pose3D &end, double turn_radius, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = turn_radius;
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = RSL_first_distance(a, b, d / radius) * radius;
    double snd_length = RSL_middle_distance(a, b, d / radius) * radius;
    double trd_length = RSL_last_distance(a, b, d / radius) * radius;

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;
    if (!std::isfinite(length) || length < 0)
    {
        return {};
    }

    Pose3D junc_1 = follow_dubins<RIGHT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<STRAIGHT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(RIGHT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(STRAIGHT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(LEFT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_radius_LSR(const Pose3D &start, const Pose3D &end, double turn_radius, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = turn_radius;
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = LSR_first_distance(a, b, d / radius) * radius;
    double snd_length = LSR_middle_distance(a, b, d / radius) * radius;
    double trd_length = LSR_last_distance(a, b, d / radius) * radius;

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;
    if (!std::isfinite(length) || length < 0)
    {
        return {};
    }

    Pose3D junc_1 = follow_dubins<LEFT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<STRAIGHT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(LEFT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(STRAIGHT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(RIGHT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_radius_RLR(const Pose3D &start, const Pose3D &end, double turn_radius, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = turn_radius;
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = RLR_first_distance(a, b, d / radius) * radius;
    double snd_length = RLR_middle_distance(a, b, d / radius) * radius;
    double trd_length = RLR_last_distance(a, b, d / radius) * radius;

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;
    if (!std::isfinite(length) || length < 0)
    {
        return {};
    }

    Pose3D junc_1 = follow_dubins<RIGHT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<LEFT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(RIGHT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(LEFT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(RIGHT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_radius_LRL(const Pose3D &start, const Pose3D &end, double turn_radius, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = turn_radius;
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = LRL_first_distance(a, b, d / radius) * radius;
    double snd_length = LRL_middle_distance(a, b, d / radius) * radius;
    double trd_length = LRL_last_distance(a, b, d / radius) * radius;

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;
    if (!std::isfinite(length) || length < 0)
    {
        return {};
    }

    Pose3D junc_1 = follow_dubins<LEFT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<RIGHT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(LEFT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(RIGHT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(LEFT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_radius_SRS(const Pose3D &start, const Pose3D &end, double turn_radius, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = turn_radius;
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = SRS_first_distance(a, b, d / radius) * radius;
    double snd_length = SRS_middle_distance(a, b, d / radius) * radius;
    double trd_length = SRS_last_distance(a, b, d / radius) * radius;

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;
    if (!std::isfinite(length) || length < 0)
    {
        return {};
    }

    Pose3D junc_1 = follow_dubins<STRAIGHT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<RIGHT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(STRAIGHT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(RIGHT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(STRAIGHT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_radius_SLS(const Pose3D &start, const Pose3D &end, double turn_radius, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = turn_radius;
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = SLS_first_distance(a, b, d / radius) * radius;
    double snd_length = SLS_middle_distance(a, b, d / radius) * radius;
    double trd_length = SLS_last_distance(a, b, d / radius) * radius;

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;
    if (!std::isfinite(length) || length < 0)
    {
        return {};
    }

    Pose3D junc_1 = follow_dubins<STRAIGHT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<LEFT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(STRAIGHT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(LEFT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(STRAIGHT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

/********************  Given length basic planning  ********************/

std::vector<DynamicPathShape> set_length_LSL(const Pose3D &start, const Pose3D &end, double turn_radius, double target_len, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = fit_LSL(a, b, d, turn_radius, target_len, tol);
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = LSL_first_distance(a, b, d / radius) * radius;
    double snd_length = LSL_middle_distance(a, b, d / radius) * radius;
    double trd_length = LSL_last_distance(a, b, d / radius) * radius;

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(std::abs(fst_length + snd_length + trd_length - target_len) < tol);
#endif

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;

    Pose3D junc_1 = follow_dubins<LEFT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<STRAIGHT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(LEFT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(STRAIGHT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(LEFT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_length_RSR(const Pose3D &start, const Pose3D &end, double turn_radius, double target_len, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = fit_RSR(a, b, d, turn_radius, target_len, tol);
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = RSR_first_distance(a, b, d / radius) * radius;
    double snd_length = RSR_middle_distance(a, b, d / radius) * radius;
    double trd_length = RSR_last_distance(a, b, d / radius) * radius;

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(std::abs(fst_length + snd_length + trd_length - target_len) < tol);
#endif

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;

    Pose3D junc_1 = follow_dubins<RIGHT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<STRAIGHT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(RIGHT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(STRAIGHT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(RIGHT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_length_RSL(const Pose3D &start, const Pose3D &end, double turn_radius, double target_len, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = fit_RSL(a, b, d, turn_radius, target_len, tol);
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = RSL_first_distance(a, b, d / radius) * radius;
    double snd_length = RSL_middle_distance(a, b, d / radius) * radius;
    double trd_length = RSL_last_distance(a, b, d / radius) * radius;

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(std::abs(fst_length + snd_length + trd_length - target_len) < tol);
#endif

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;

    Pose3D junc_1 = follow_dubins<RIGHT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<STRAIGHT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(RIGHT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(STRAIGHT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(LEFT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_length_LSR(const Pose3D &start, const Pose3D &end, double turn_radius, double target_len, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = fit_LSR(a, b, d, turn_radius, target_len, tol);
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = LSR_first_distance(a, b, d / radius) * radius;
    double snd_length = LSR_middle_distance(a, b, d / radius) * radius;
    double trd_length = LSR_last_distance(a, b, d / radius) * radius;

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(std::abs(fst_length + snd_length + trd_length - target_len) < tol);
#endif

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;

    Pose3D junc_1 = follow_dubins<LEFT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<STRAIGHT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(LEFT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(STRAIGHT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(RIGHT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_length_RLR(const Pose3D &start, const Pose3D &end, double turn_radius, double target_len, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = fit_RLR(a, b, d, turn_radius, target_len, tol);
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = RLR_first_distance(a, b, d / radius) * radius;
    double snd_length = RLR_middle_distance(a, b, d / radius) * radius;
    double trd_length = RLR_last_distance(a, b, d / radius) * radius;

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(std::abs(fst_length + snd_length + trd_length - target_len) < tol);
#endif

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;

    Pose3D junc_1 = follow_dubins<RIGHT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<LEFT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(RIGHT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(LEFT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(RIGHT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_length_LRL(const Pose3D &start, const Pose3D &end, double turn_radius, double target_len, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = fit_LRL(a, b, d, turn_radius, target_len, tol);
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = LRL_first_distance(a, b, d / radius) * radius;
    double snd_length = LRL_middle_distance(a, b, d / radius) * radius;
    double trd_length = LRL_last_distance(a, b, d / radius) * radius;

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(std::abs(fst_length + snd_length + trd_length - target_len) < tol);
#endif

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;

    Pose3D junc_1 = follow_dubins<LEFT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<RIGHT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(LEFT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(RIGHT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(LEFT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_length_SRS(const Pose3D &start, const Pose3D &end, double turn_radius, double target_len, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = fit_SRS(a, b, d, turn_radius, target_len, tol);
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = SRS_first_distance(a, b, d / radius) * radius;
    double snd_length = SRS_middle_distance(a, b, d / radius) * radius;
    double trd_length = SRS_last_distance(a, b, d / radius) * radius;

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(std::abs(fst_length + snd_length + trd_length - target_len) < tol);
#endif

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;

    Pose3D junc_1 = follow_dubins<STRAIGHT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<RIGHT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(STRAIGHT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(RIGHT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(STRAIGHT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

std::vector<DynamicPathShape> set_length_SLS(const Pose3D &start, const Pose3D &end, double turn_radius, double target_len, double tol)
{
    double a, b, d, theta;
    std::tie(a, b, d, theta) = normalize_poses(start, end);
    double radius = fit_SLS(a, b, d, turn_radius, target_len, tol);
    if (std::isnan(radius))
    {
        return {};
    }

    double fst_length = SLS_first_distance(a, b, d / radius) * radius;
    double snd_length = SLS_middle_distance(a, b, d / radius) * radius;
    double trd_length = SLS_last_distance(a, b, d / radius) * radius;

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(std::abs(fst_length + snd_length + trd_length - target_len) < tol);
#endif

    double length = fst_length + snd_length + trd_length;
    double dz = end.z - start.z;
    double fst_climb = dz * fst_length / length;
    double snd_climb = dz * snd_length / length;
    double trd_climb = dz * trd_length / length;

    Pose3D junc_1 = follow_dubins<STRAIGHT>(start, fst_length, 1., fst_climb, radius);
    Pose3D junc_2 = follow_dubins<LEFT>(junc_1, snd_length, 1., snd_climb, radius);

    DynamicPathShape s1(STRAIGHT, start, junc_1, 1., radius, fst_climb);
    DynamicPathShape s2(LEFT, junc_1, junc_2, 1., radius, snd_climb);
    DynamicPathShape s3(STRAIGHT, junc_2, end, 1., radius, trd_climb);

    std::vector<DynamicPathShape> output;
    if (s1.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s1);
    }
    if (s2.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s2);
    }
    if (s3.length > DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        output.push_back(s3);
    }

    Pose3D computed_end = output.back().final_pose();
    if ((pose_dist(end, computed_end) >= tol) || (abs(pose_aligned(end, computed_end)) >= tol))
    {
        return {};
    }
    else
    {
        assert(abs(length - (s1.length + s2.length + s3.length)) < tol);
        return output;
    }
}

/******************** Other ********************/

std::vector<std::unique_ptr<Dubins>> list_possible_baseDubins(double _climb, double _turn_radius, const Pose3D &_start, const Pose3D &_end, double wind_x, double wind_y, double tol)
{
    std::vector<std::unique_ptr<Dubins>> output;

    auto LSL_pathshapes = set_radius_LSL(_start, _end, _turn_radius, tol);
    auto LSR_pathshapes = set_radius_LSR(_start, _end, _turn_radius, tol);
    auto RSR_pathshapes = set_radius_RSR(_start, _end, _turn_radius, tol);
    auto RSL_pathshapes = set_radius_RSL(_start, _end, _turn_radius, tol);
    auto RLR_pathshapes = set_radius_RLR(_start, _end, _turn_radius, tol);
    auto LRL_pathshapes = set_radius_LRL(_start, _end, _turn_radius, tol);
    auto SRS_pathshapes = set_radius_SRS(_start, _end, _turn_radius, tol);
    auto SLS_pathshapes = set_radius_SLS(_start, _end, _turn_radius, tol);

    auto transfer = [&](auto &pathshapes)
    {
        if (pathshapes.size() > 0)
        {
            output.push_back(std::make_unique<Dubins>(_start, _end, pathshapes));
        }
    };

    transfer(LSL_pathshapes);
    transfer(LSR_pathshapes);
    transfer(RSR_pathshapes);
    transfer(RSL_pathshapes);
    transfer(RLR_pathshapes);
    transfer(LRL_pathshapes);
    transfer(SRS_pathshapes);
    transfer(SLS_pathshapes);
    return output;
}

std::unique_ptr<Dubins> shortest_possible_baseDubins(double _climb, double _turn_radius, const Pose3D &_start, const Pose3D &_end, double wind_x, double wind_y, double tol)
{
    std::vector<std::unique_ptr<Dubins>> all_dubins = list_possible_baseDubins(_climb, _turn_radius, _start, _end, wind_x, wind_y, tol);

    uint best_i;
    double min_length = INFINITY;

    for (uint i = 0; i < all_dubins.size(); i++)
    {
        double len = all_dubins[i]->get_length();
        if (len < min_length)
        {
            min_length = len;
            best_i = i;
        }
    }

    return std::move(all_dubins[best_i]);
}

std::vector<std::unique_ptr<Dubins>> fit_possible_baseDubins(double _climb, double _turn_radius, const Pose3D &_start, const Pose3D &_end,
                                                             double target_len, double tol)
{
    std::vector<std::unique_ptr<Dubins>> output;

    auto LSL_pathshapes = set_length_LSL(_start, _end, _turn_radius, target_len, tol);
    auto LSR_pathshapes = set_length_LSR(_start, _end, _turn_radius, target_len, tol);
    auto RSR_pathshapes = set_length_RSR(_start, _end, _turn_radius, target_len, tol);
    auto RSL_pathshapes = set_length_RSL(_start, _end, _turn_radius, target_len, tol);
    auto RLR_pathshapes = set_length_RLR(_start, _end, _turn_radius, target_len, tol);
    auto LRL_pathshapes = set_length_LRL(_start, _end, _turn_radius, target_len, tol);
    auto SRS_pathshapes = set_length_SRS(_start, _end, _turn_radius, target_len, tol);
    auto SLS_pathshapes = set_length_SLS(_start, _end, _turn_radius, target_len, tol);

    auto transfer = [&](auto &pathshapes)
    {
        if (pathshapes.size() > 0)
        {
            output.push_back(std::make_unique<Dubins>(_start, _end, pathshapes));
        }
    };

    transfer(LSL_pathshapes);
    transfer(LSR_pathshapes);
    transfer(RSR_pathshapes);
    transfer(RSL_pathshapes);
    transfer(RLR_pathshapes);
    transfer(LRL_pathshapes);
    transfer(SRS_pathshapes);
    transfer(SLS_pathshapes);
    return output;
}