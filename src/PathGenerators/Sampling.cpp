#include "PathGenerators/Sampling.hpp"

#include "Ellipse.hpp"

#include <iostream>
// -------------------- Helper functions --------------------//

static std::optional<Ellipse> generate_ellipse_from_poses(const Pose3D &f, const Pose3D &fbis, double radius)
{
    return Ellipse::from_foci(
        f.x, f.y, fbis.x, fbis.y, radius);
}


static void join_and_add_paths(std::vector<std::unique_ptr<Dubins>> &store,
                               std::vector<std::unique_ptr<Dubins>> &s1, std::vector<std::unique_ptr<Dubins>> &s2, double join_precision,
                               const AircraftStats& stats)
{
    for (const auto &p1 : s1)
    {
        for (const auto &p2 : s2)
        {
            auto joined = join_paths(*p1, *p2, join_precision);
            if (joined.is_valid())
            {
                joined.speed_profile = SpeedProfile::from_length(stats.airspeed, stats.target_airspeed, joined.get_length());
                store.push_back(std::make_unique<Dubins>(joined));
            }
        }
    }
}

// -------------------- Non-fitted paths --------------------//

std::vector<std::unique_ptr<Dubins>> EllipseSampling::generate_paths(
    const Pose3D &start, const Pose3D &end, const AircraftStats& stats) const
{
    std::vector<std::unique_ptr<Dubins>> output = underlying->generate_paths(start, end, stats);

    double dist = pose_dist_XY(start, end);
    for (auto ratio : ellipse_ratios)
    {
        double radius = dist * ratio;
        std::optional<Ellipse> ell_o = generate_ellipse_from_poses(start, end, radius);

        if (!ell_o.has_value())
        {
            continue;
        }

        Ellipse ell = ell_o.value();

        for (double angle : samples_angles)
        {
            double px, py;
            std::tie(px, py) = ell.point_at_angle(angle);
            Pose3D midpoint(px, py, 0, angle + M_PI_2);

            double d1 = pose_dist_XY(midpoint, start);
            double d2 = pose_dist_XY(midpoint, end);

            midpoint.z = (d1 * start.z + d2 * end.z) * (d1 + d2);

            Pose3D midpoint_bis(midpoint);
            midpoint_bis.theta = angle - M_PI_2;

            std::vector<std::unique_ptr<Dubins>> p1, p2, p1_bis, p2_bis;

            p1 = underlying->generate_paths(start, midpoint, stats);
            p2 = underlying->generate_paths(midpoint, end, stats);

            join_and_add_paths(output, p1, p2, DubinsFleetPlanner_PRECISION, stats);

            p1_bis = underlying->generate_paths(start, midpoint_bis, stats);
            p2_bis = underlying->generate_paths(midpoint_bis, end, stats);

            join_and_add_paths(output, p1_bis, p2_bis, DubinsFleetPlanner_PRECISION, stats);
        }
    }

    return output;
}

std::vector<std::unique_ptr<Dubins>> RandomEllipseSampling::generate_paths(
    const Pose3D &start, const Pose3D &end, const AircraftStats& stats) const
{
    std::vector<std::unique_ptr<Dubins>> output = underlying->generate_paths(start, end, stats);

    double dist = pose_dist_XY(start, end);
    for (auto ratio : ellipse_ratios)
    {

        double radius = dist * ratio;
        std::optional<Ellipse> ell_o = generate_ellipse_from_poses(start, end, radius);

        if (!ell_o.has_value())
        {
            continue;
        }

        Ellipse ell = ell_o.value();

        std::uniform_real_distribution<double> angle_distrib(-M_PI, M_PI);

        for (int i = 0; i < samples_num; i++)
        {
            double px, py;
            std::tie(px, py) = ell.random_point_inside(rng());

            double angle = angle_distrib(rng);

            Pose3D midpoint(px, py, 0, angle);

            double d1 = pose_dist_XY(midpoint, start);
            double d2 = pose_dist_XY(midpoint, end);

            midpoint.z = (d1 * start.z + d2 * end.z) * (d1 + d2);

            std::vector<std::unique_ptr<Dubins>> p1, p2, p1_bis, p2_bis;

            p1 = underlying->generate_paths(start, midpoint, stats);
            p2 = underlying->generate_paths(midpoint, end, stats);

            join_and_add_paths(output, p1, p2, DubinsFleetPlanner_PRECISION, stats);
        }
    }

    return output;
}

// -------------------- Fitted paths -------------------- //

std::vector<std::unique_ptr<Dubins>> FittedEllipseSampling::generate_paths(
    const Pose3D &start, const Pose3D &end, const AircraftStats& stats, double target_len) const
{
    std::vector<std::unique_ptr<Dubins>> output = underlying->generate_paths(start,end,stats,target_len);

    double dist = pose_dist_XY(start, end);
    double min_radius  = dist * 0.5; // Minimum possible ellipse radius (distance between foci)
    double max_radius  = target_len * 0.5; // Maximum radius (half of the target length, to ensure that the path can be generated)
    // double dr = max_radius - min_radius;
    // if (dr < 0)
    // {
    //     std::cout << "Warning: unable to generate ellipse with distance " << dist << " and target length " << target_len << std::endl;
    //     return output;
    // }

    // int n = target_length_ratios.size();
    for (auto ratio : target_length_ratios)
    {


        double radius = target_len * ratio;
        if (radius < min_radius || radius > max_radius)
        {
            continue;
        }

        std::optional<Ellipse> ell_o = generate_ellipse_from_poses(start, end, radius);

        if (!ell_o.has_value())
        {
            continue;
        }

        Ellipse ell = ell_o.value();

        for (double angle : samples_angles)
        {
            double px, py;
            std::tie(px, py) = ell.point_at_angle(angle);
            Pose3D midpoint(px, py, 0, angle + M_PI_2);

            double d1 = pose_dist_XY(midpoint, start);
            double d2 = pose_dist_XY(midpoint, end);

            double r1 = d1 / (d1 + d2);
            double r2 = 1 - r1;

            midpoint.z = r1 * start.z + r2 * end.z;

            Pose3D midpoint_bis(midpoint);
            midpoint_bis.theta = angle - M_PI_2;

            std::vector<std::unique_ptr<Dubins>> p1, p2, p1_bis, p2_bis;

            p1 = underlying->generate_paths(start, midpoint, stats, r1 * target_len);
            p2 = underlying->generate_paths(midpoint, end, stats, r2 * target_len);

            join_and_add_paths(output, p1, p2, DubinsFleetPlanner_PRECISION, stats);

            p1_bis = underlying->generate_paths(start, midpoint_bis, stats, r1 * target_len);
            p2_bis = underlying->generate_paths(midpoint_bis, end, stats, r2 * target_len);

            join_and_add_paths(output, p1_bis, p2_bis, DubinsFleetPlanner_PRECISION, stats);
        }
    }
    return output;
}

std::vector<std::unique_ptr<Dubins>> FittedRandomEllipseSampling::generate_paths(
    const Pose3D &start, const Pose3D &end, const AircraftStats& stats, double target_len) const
{
    std::vector<std::unique_ptr<Dubins>> output = underlying->generate_paths(start, end, stats, target_len);

    for (auto ratio : target_length_ratios)
    {
        double radius = target_len * ratio;
        std::optional<Ellipse> ell_o = generate_ellipse_from_poses(start, end, radius);

        if (!ell_o.has_value())
        {
            continue;
        }

        Ellipse ell = ell_o.value();

        std::uniform_real_distribution<double> angle_distrib(-M_PI, M_PI);

        for (int i = 0; i < samples_num; i++)
        {
            double px, py;
            std::tie(px, py) = ell.random_point_inside(rng());

            double angle = angle_distrib(rng);

            Pose3D midpoint(px, py, 0, angle);

            double d1 = pose_dist_XY(midpoint, start);
            double d2 = pose_dist_XY(midpoint, end);

            double r1 = d1 / (d1 + d2);
            double r2 = 1 - r1;

            midpoint.z = r1 * start.z + r2 * end.z;

            std::vector<std::unique_ptr<Dubins>> p1, p2, p1_bis, p2_bis;

            p1 = underlying->generate_paths(start, midpoint, stats, r1 * target_len);
            p2 = underlying->generate_paths(midpoint, end, stats, r2 * target_len);

            join_and_add_paths(output, p1, p2, DubinsFleetPlanner_PRECISION, stats);
        }
    }
    return output;
}
