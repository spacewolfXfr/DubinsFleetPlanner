#pragma once

#include "PathGenerator.hpp"
#include "Primitives.hpp"

// ---------- Generic templates ---------- //

/**
 * @brief Given an arbitrary path generating function, extend the generated paths
 * 
 * @tparam beginMove    Type of the begin extension
 * @tparam endMove      Type of the end extension
 * @param generator     Path generator object
 * @param start         Path start
 * @param end           Path end
 * @param start_lens    Initial length extensions
 * @param end_lens      Final length extensions
 * @param _climb        Maximal climb rate
 * @param _turn_radius  Minimal turn radius
 * @return std::vector<std::unique_ptr<Dubins>> 
 */
template <DubinsMove beginMove, DubinsMove endMove>
std::vector<std::unique_ptr<Dubins>> generate_extended_paths(
    const PathsGenerator& generator,
    const Pose3D& start, const Pose3D& end, std::vector<double> start_lens, std::vector<double> end_lens,
    double _climb, double _turn_radius)
{
    std::vector<std::unique_ptr<Dubins>> output;

    for(double sl : start_lens)
    {
        Pose3D shift_start = follow_dubins<beginMove>(start,sl,1.,0., _turn_radius);
        DynamicPathShape shape_start(beginMove, start, shift_start, 1., _turn_radius, 0.);

        for(double el : end_lens)
        {
            Pose3D shift_end = follow_dubins<endMove>(end,-el,1.,0., _turn_radius);
            DynamicPathShape shape_end(beginMove, shift_end, end, 1., _turn_radius, 0.);

            auto generated = generator.generate_paths(shift_start, shift_end, _climb, _turn_radius);

            for(auto& b : generated)
            {
                std::vector<DynamicPathShape> extended_shapes = {shape_start};
                extended_shapes.insert(
                    extended_shapes.end(),
                    b->get_all_sections().begin(),
                    b->get_all_sections().end());

                extended_shapes.push_back(shape_end);

                output.push_back(std::make_unique<Dubins>(start,end,extended_shapes));
            }        
        }
    }

    return output;
}

/**
 * @brief Given an arbitrary fitted path generating function, extend the generated paths
 * 
 * @tparam beginMove    Type of the begin extension
 * @tparam endMove      Type of the end extension
 * @param generator     Path generator object
 * @param start         Path start
 * @param end           Path end
 * @param start_lens    Initial length extensions
 * @param end_lens      Final length extensions
 * @param _climb        Maximal climb rate
 * @param _turn_radius  Minimal turn radius
 * @param target_len    Target length
 * @return std::vector<std::unique_ptr<Dubins>> 
 */
template <DubinsMove beginMove, DubinsMove endMove>
std::vector<std::unique_ptr<Dubins>> generate_extended_fitted_paths(
    const FittedPathsGenerator& generator,
    const Pose3D& start, const Pose3D& end, std::vector<double> start_lens, std::vector<double> end_lens,
    double _climb, double _turn_radius, double target_len)
{
    std::vector<std::unique_ptr<Dubins>> output;

    for(double sl : start_lens)
    {
        Pose3D shift_start = follow_dubins<beginMove>(start,sl,1.,0., _turn_radius);
        DynamicPathShape shape_start(beginMove, start, shift_start, 1., _turn_radius, 0.);

        for(double el : end_lens)
        {
            Pose3D shift_end = follow_dubins<endMove>(end,-el,1.,0., _turn_radius);
            DynamicPathShape shape_end(endMove, shift_end, end, 1., _turn_radius, 0.);

            if (target_len - sl - el < 0.)
            {
                continue;
            }

            auto generated = generator.generate_paths(shift_start, shift_end, _climb, _turn_radius, target_len-sl-el);


            for(auto& b : generated)
            {
                std::vector<DynamicPathShape> extended_shapes = {shape_start};
                extended_shapes.insert(
                    extended_shapes.end(),
                    b->get_all_sections().begin(),
                    b->get_all_sections().end());

                extended_shapes.push_back(shape_end);

                output.push_back(std::make_unique<Dubins>(start,end,extended_shapes));
            }        
        }
    }

    return output;
}

// -------------------- Classes -------------------- //

// ---------- Non-fitted paths ---------- //

/**
 * @brief Extend path generation based on constant Dubins moves
 * 
 * @tparam fstMove Type of the first move to use for extension
 * @tparam lstMove Type of the last move to use for extension
 */
template<DubinsMove fstMove, DubinsMove lstMove>
class SingleExtendedPathsGenerator : public PathsGenerator
{
public:
    std::unique_ptr<PathsGenerator> underlying;
    std::vector<double> start_lengths,end_lengths;

    SingleExtendedPathsGenerator(std::unique_ptr<PathsGenerator> _under, std::vector<double> _starts, std::vector<double> _ends)
    : underlying(std::move(_under)), start_lengths(_starts), end_lengths(_ends) {}

    std::string my_name() const override
    {
        std::string output = "ExtendedPathsGenerator<";
        output += get_DubinsMove_name(fstMove);
        output += ",";
        output += underlying->my_name();
        output += ",";
        output += get_DubinsMove_name(lstMove);
        output += ">";
        return output;
    }

    std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, double climb, double turn_radius) const override
    {
        return generate_extended_paths<fstMove,lstMove>(
            *underlying, start, end, start_lengths, end_lengths,
            climb, turn_radius);
    }

    uint max_path_num() const override
    {
        return start_lengths.size() * end_lengths.size() * underlying->max_path_num();
    }

};

// Explicit specialization for straight first and last parts
typedef SingleExtendedPathsGenerator<STRAIGHT,STRAIGHT> StraightExtendedPathsGenerator;


/**
 * @brief Use all combination of Dubins moves for generating extended paths, that is Straight,Left and Right
 * 
 */
class AllExtendedPathsGenerator : public PathsGenerator
{
public:
    std::unique_ptr<PathsGenerator> underlying;
    std::vector<double> start_lengths,end_lengths;

    AllExtendedPathsGenerator(std::unique_ptr<PathsGenerator> _under, std::vector<double> _starts, std::vector<double> _ends)
    : underlying(std::move(_under)), start_lengths(_starts), end_lengths(_ends) {}

    std::string my_name() const override
    {
        return "AllExtendedPathsGenerator<"+underlying->my_name()+">";
    }

    std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, double climb, double turn_radius) const override;

    uint max_path_num() const override;
};


// ---------- Fitted paths ---------- //

/**
 * @brief Extend path generation based on constant Dubins moves
 * 
 * @tparam fstMove Type of the first move to use for extension
 * @tparam lstMove Type of the last move to use for extension
 */
template<DubinsMove fstMove, DubinsMove lstMove>
class SingleExtendedFittedPathsGenerator : public FittedPathsGenerator
{
public:
    std::unique_ptr<FittedPathsGenerator> underlying;
    std::vector<double> start_lengths,end_lengths;

    SingleExtendedFittedPathsGenerator(std::unique_ptr<FittedPathsGenerator> _under, std::vector<double> _starts, std::vector<double> _ends)
    : underlying(std::move(_under)), start_lengths(_starts), end_lengths(_ends) {}

    std::string my_name() const override
    {
        std::string output = "ExtendedFittedPathsGenerator<";
        output += get_DubinsMove_name(fstMove);
        output += ",";
        output += underlying->my_name();
        output += ",";
        output += get_DubinsMove_name(lstMove);
        output += ">";
        return output;
    }

    std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, double climb, double turn_radius, double target_len) const override
    {
        return generate_extended_fitted_paths<fstMove,lstMove>(
            *underlying, start, end, start_lengths, end_lengths,
            climb, turn_radius, target_len);
    }

    uint max_path_num() const override
    {
        return start_lengths.size() * end_lengths.size() * underlying->max_path_num();
    }

};

// Explicit specialization for straight first and last parts
typedef SingleExtendedFittedPathsGenerator<STRAIGHT,STRAIGHT> StraightExtendedFittedPathsGenerator;


/**
 * @brief Use all combination of Dubins moves for generating extended paths, that is Straight,Left and Right
 * 
 */
class AllExtendedFittedPathsGenerator : public FittedPathsGenerator
{
public:
    std::unique_ptr<FittedPathsGenerator> underlying;
    std::vector<double> start_lengths,end_lengths;

    AllExtendedFittedPathsGenerator(std::unique_ptr<FittedPathsGenerator> _under, std::vector<double> _starts, std::vector<double> _ends)
    : underlying(std::move(_under)), start_lengths(_starts), end_lengths(_ends) {}

    std::string my_name() const override
    {
        return "AllExtendedFittedPathsGenerator<" + underlying->my_name() + ">";
    }

    std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, double climb, double turn_radius, double target_len) const override;

    uint max_path_num() const override;
};

/**
 * @brief Compute fitting length straight extensions on top of constant radius base Dubins
 * 
 */
class FittedStraightBaseDubins : public FittedPathsGenerator
{
public:
    double precision_tol = DubinsFleetPlanner_PRECISION;
    std::vector<double> ratios;

    FittedStraightBaseDubins(std::vector<double> _ratios) : ratios(_ratios) {}

    FittedStraightBaseDubins(std::vector<double> _ratios, double _tol)
    : ratios(_ratios), precision_tol(_tol) {}

    std::string my_name() const override
    {
        return "FittedStraightBaseDubins";
    }

    std::vector<std::unique_ptr<Dubins>> generate_paths(
        const Pose3D& start, const Pose3D& end, double climb, double turn_radius, double target_len) const override;

    uint max_path_num() const
    {
        return 8 * ratios.size();
    }
};
