#pragma once

#include <random>
#include "PathGenerator.hpp"
#include "Dubins.hpp"

// -------------------- Non-fitted paths --------------------//

/**
 * @brief Draw samples on an ellipse as a new midpoint for path generation
 * 
 */
class EllipseSampling : public PathsGenerator
{
public:
    std::unique_ptr<PathsGenerator> underlying;
    std::vector<double> ellipse_ratios; // Ellipse 'radius' is equal to the distance separating start and end multiplied by this ratio (should be greater than 1)
    std::vector<double> samples_angles;

    EllipseSampling(std::unique_ptr<PathsGenerator> _under, double _ratio, uint _samples)
    : underlying(std::move(_under)), ellipse_ratios({_ratio})
    {
        for(int i = 0; i < _samples; i++)
        {
            samples_angles.push_back(i*2*M_PI/_samples);
        }
    }

    EllipseSampling(std::unique_ptr<PathsGenerator> _under, double _ratio, const std::vector<double> _samples)
    : underlying(std::move(_under)), ellipse_ratios({_ratio}), samples_angles(_samples) {}


    EllipseSampling(std::unique_ptr<PathsGenerator> _under, std::vector<double> _ratios, uint _samples)
    : underlying(std::move(_under)), ellipse_ratios(_ratios)
    {
        for(int i = 0; i < _samples; i++)
        {
            samples_angles.push_back(i*2*M_PI/_samples);
        }
    }

    EllipseSampling(std::unique_ptr<PathsGenerator> _under, std::vector<double> _ratios, const std::vector<double> _samples)
    : underlying(std::move(_under)), ellipse_ratios(_ratios), samples_angles(_samples) {}


    std::string my_name() const override
    {
        return "EllipseSampling<"+underlying->my_name()+">";
    }

    std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, double climb, double turn_radius) const override;

    uint max_path_num() const override
    {
        uint underlying_num = underlying->max_path_num();
        return samples_angles.size() * underlying_num * underlying_num;
    }
};

/**
 * @brief Draw random samples inside an ellipse as a new midpoint for path generation
 * 
 */
class RandomEllipseSampling : public PathsGenerator
{
public:
    std::unique_ptr<PathsGenerator> underlying;
    std::vector<double> ellipse_ratios; // Ellipse 'radius' is equal to the distance separating start and end multiplied by this ratio (should be greater than 1)
    uint samples_num;
    mutable std::mt19937 rng;

    RandomEllipseSampling(std::unique_ptr<PathsGenerator> _under, double _ratio, uint _samples, int seed = 0)
    : underlying(std::move(_under)), ellipse_ratios({_ratio}), samples_num(_samples), rng(std::mt19937(seed)) {}

    RandomEllipseSampling(std::unique_ptr<PathsGenerator> _under, std::vector<double> _ratios, uint _samples, int seed = 0)
    : underlying(std::move(_under)), ellipse_ratios(_ratios), samples_num(_samples), rng(std::mt19937(seed)) {}


    std::string my_name() const override
    {
        return "RandomEllipseSampling<"+underlying->my_name()+">";
    }

    std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, double climb, double turn_radius) const override;

    uint max_path_num() const override
    {
        uint underlying_num = underlying->max_path_num();
        return samples_num * underlying_num * underlying_num;
    }
};

// -------------------- Fitted paths -------------------- //

class FittedEllipseSampling : public FittedPathsGenerator
{
public:
    std::unique_ptr<FittedPathsGenerator> underlying;
    std::vector<double> target_length_ratios; // Ellipse 'radius' is equal to the target length multiplied by this ratio. Should be smaller than 1 to give some leeway to path generation.
    std::vector<double> samples_angles;

    FittedEllipseSampling(std::unique_ptr<FittedPathsGenerator> _under, double _ratio, uint _samples)
    : underlying(std::move(_under)), target_length_ratios({_ratio})
    {
        for(int i = 0; i < _samples; i++)
        {
            samples_angles.push_back(i*2*M_PI/_samples);
        }
    }

    FittedEllipseSampling(std::unique_ptr<FittedPathsGenerator> _under, double _ratio, const std::vector<double> _samples)
    : underlying(std::move(_under)), target_length_ratios({_ratio}), samples_angles(_samples) {}


    FittedEllipseSampling(std::unique_ptr<FittedPathsGenerator> _under, std::vector<double> _ratios, uint _samples)
    : underlying(std::move(_under)), target_length_ratios(_ratios)
    {
        for(int i = 0; i < _samples; i++)
        {
            samples_angles.push_back(i*2*M_PI/_samples);
        }
    }

    FittedEllipseSampling(std::unique_ptr<FittedPathsGenerator> _under, std::vector<double> _ratios, const std::vector<double> _samples)
    : underlying(std::move(_under)), target_length_ratios(_ratios), samples_angles(_samples) {}

    std::string my_name() const override
    {
        return "FittedEllipseSampling<"+underlying->my_name()+">";
    }

    std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, double climb, double turn_radius, double target_len) const override;

    uint max_path_num() const override
    {
        uint underlying_num = underlying->max_path_num();
        return samples_angles.size() * underlying_num * underlying_num;
    }
};

class FittedRandomEllipseSampling : public FittedPathsGenerator
{
public:
    std::unique_ptr<FittedPathsGenerator> underlying;
    std::vector<double> target_length_ratios; // Ellipse 'radius' is equal to the target length multiplied by this ratio. Should be smaller than 1 to give some leeway to path generation.
    uint samples_num;
    mutable std::mt19937 rng;

    FittedRandomEllipseSampling(std::unique_ptr<FittedPathsGenerator> _under, double _ratio, uint _samples, int seed=0)
    : underlying(std::move(_under)), target_length_ratios({_ratio}), samples_num(_samples), rng(std::mt19937(seed)) {}
    
    FittedRandomEllipseSampling(std::unique_ptr<FittedPathsGenerator> _under, std::vector<double> _ratios, uint _samples, int seed=0)
    : underlying(std::move(_under)), target_length_ratios(_ratios), samples_num(_samples), rng(std::mt19937(seed)) {}

    std::string my_name() const override
    {
        return "FittedRandomEllipseSampling<"+underlying->my_name()+">";
    }

    std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, double climb, double turn_radius, double target_len) const override;

    uint max_path_num() const override
    {
        uint underlying_num = underlying->max_path_num();
        return samples_num * underlying_num * underlying_num;
    }
};