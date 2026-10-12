#pragma once

#include "PathGenerator.hpp"
#include "BaseDubins.hpp"

class BasePathsGenerator : public PathsGenerator
{
public:
    double tol;
    BasePathsGenerator(double _tol) : tol(_tol) {}

    std::string my_name() const override
    {
        return "BaseFittedPathsGenerator";
    }

    std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats) const override
    {
        return list_possible_baseDubins(stats.climb, stats.turn_radius, start, end, tol);
    }

    uint max_path_num() const override
    {
        return NumberOfBaseDubins;
    }
};

class BaseFittedPathsGenerator : public FittedPathsGenerator
{
public:
    double precision_tol = DubinsFleetPlanner_PRECISION;

    BaseFittedPathsGenerator() {}
    BaseFittedPathsGenerator(double _tol) : precision_tol(_tol) {}

    std::string my_name() const override
    {
        return "BaseFittedPathsGenerator";
    }

    std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_length) const override
    {
        return fit_possible_baseDubins(stats, start, end, target_length, precision_tol);
    }

    uint max_path_num() const override
    {
        return NumberOfBaseDubins;
    }
};