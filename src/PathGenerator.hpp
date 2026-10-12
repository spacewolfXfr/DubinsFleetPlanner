#pragma once

#include <string>

#include "Dubins.hpp"
#include "Pose3D.hpp"

/**
 * @brief Toplevel class describing a path generator (from a start to an end, with given climb rate and turn radius),
 * generating several paths of arbitrary lengths.
 * 
 * Should replace PathGeneratorFunction type of Dubins.hpp
 */
class PathsGenerator
{
public:

    virtual ~PathsGenerator() = default;

    virtual std::string my_name() const
    {
        return "PathsGenerator";
    }

    virtual std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats) const = 0;

    virtual uint max_path_num() const = 0;
};

class AggregatedPathsGenerator : public PathsGenerator
{
public:
    std::vector<std::unique_ptr<PathsGenerator>> subgenerators;

    AggregatedPathsGenerator(std::vector<std::unique_ptr<PathsGenerator>> _gens)
        : subgenerators(std::move(_gens)) {}

    AggregatedPathsGenerator(AggregatedPathsGenerator&& other)
        : subgenerators(std::move(other.subgenerators)) {}

    std::string my_name() const override
    {
        std::string output = "[ ";
        for(const auto &g : subgenerators)
        {
            output += g->my_name() + ", ";
        }
        output += " ]";
        return output;
    }

    std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats) const override
    {
        std::vector<std::unique_ptr<Dubins>> output;
        for(const auto& g: subgenerators)
        {
            std::vector<std::unique_ptr<Dubins>> tmp = g->generate_paths(start,end,stats);
            output.insert(output.end(),std::move_iterator(tmp.begin()),std::move_iterator(tmp.end()));
        }
        return output;
    }

    uint max_path_num() const override
    {
        uint output = 0;
        for(const auto& g: subgenerators)
        {
            output += g->max_path_num();
        }
        return output;
    }
};

/**
 * @brief Toplevel class describing a path generator (from a start to an end, with given climb rate and turn radius),
 * generating several paths of a given length.
 * 
 * Should replace FittedPathGeneratorFunction type of Dubins.hpp
 */
class FittedPathsGenerator
{
public:
    virtual ~FittedPathsGenerator() = default;

    virtual std::string my_name() const
    {
        return "FittedPathsGenerator";
    }

    virtual std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const = 0;

    virtual uint max_path_num() const = 0;
};


class AggregatedFittedPathsGenerator : public FittedPathsGenerator
{
public:
    std::vector<std::unique_ptr<FittedPathsGenerator>> subgenerators;

    AggregatedFittedPathsGenerator(std::vector<std::unique_ptr<FittedPathsGenerator>> _gens)
        : subgenerators(std::move(_gens)) {}

    AggregatedFittedPathsGenerator(AggregatedFittedPathsGenerator&& other)
        : subgenerators(std::move(other.subgenerators)) {}

    std::string my_name() const override
    {
        std::string output = "[ ";
        for(const auto &g : subgenerators)
        {
            output += g->my_name() + ", ";
        }
        output += " ]";
        return output;
    }

    std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const override
    {
        std::vector<std::unique_ptr<Dubins>> output;
        for(const auto& g: subgenerators)
        {
            std::vector<std::unique_ptr<Dubins>> tmp = g->generate_paths(start,end,stats,target_len);
            for(const auto& el : tmp)
            {
                if (el->is_valid())
                {
                    assert(abs(el->get_length() - target_len) < 1e-1);
                }
            }
            output.insert(output.end(),std::move_iterator(tmp.begin()),std::move_iterator(tmp.end()));
        }
        return output;
    }

    uint max_path_num() const override
    {
        uint output = 0;
        for(const auto& g: subgenerators)
        {
            output += g->max_path_num();
        }
        return output;
    }
};