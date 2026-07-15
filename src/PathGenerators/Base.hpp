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
        const Pose3D& start, const Pose3D& end, double climb, double turn_radius) const override
    {
        return list_possible_baseDubins(climb, turn_radius, start, end, tol);
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
        const Pose3D& start, const Pose3D& end, double climb, double turn_radius, double target_len) const override
    {
        return fit_possible_baseDubins(climb, turn_radius, start, end, target_len, precision_tol);
    }

    uint max_path_num() const override
    {
        return NumberOfBaseDubins;
    }
};