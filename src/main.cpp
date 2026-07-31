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

#include <boost/program_options.hpp>
#include <boost/chrono.hpp>

#include <string>
#include <iostream>
#include <unistd.h>

#include <chrono>
#include <ctime>

#include <filesystem>

#include "ProjectHeader.h"
#include "ioUtils.hpp"
#include "FleetPlanner.hpp"
#include "PathGenerators/all.hpp"
#include "ConflictList.hpp"

/******************** Util functions ********************/

static void lower_string(std::string& s)
{
    std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c){ return std::tolower(c); });
}

template<Dubins::DubinsSeparationFunction sep_fun>
bool check_solution(const std::vector<Dubins>& s, const std::vector<AircraftStats>& stats, double min_sep)
{
    SharedListOfPossibilities l;
    
    for(auto &d : s)
    {
        l.push_back(std::vector<std::shared_ptr<Dubins>>({std::make_shared<Dubins>(d)}));
    }

    std::vector<Conflict_T> conflicts = generic_parallel_compute_separations<sep_fun>(2,l,stats,min_sep);
    // std::vector<Conflict_T> re_conflicts = generic_compute_separations<Dubins::are_XY_separated>(l,stats,min_sep);

    return conflicts.size() == 0;
}

/******************** Argument parsing ********************/

namespace fs = std::filesystem;
namespace po = boost::program_options;
namespace chrono = boost::chrono;


struct program_arguments
{
    string src_pathname;
    string out_pathname;
    double separation;
    string wind_x_str,wind_y_str;
    double wind_x,wind_y;
    bool reallocate;
    bool allow_shortest;
    vector<double> start_length_extensions,end_length_extensions;
    int thread_num;
    int max_iters,weave_iters;
    double min_weave_dist,min_weave_dist_percent;
    int samples;
    vector<double> ratios;
    bool ellipse_border_only;
    int export_samples;
    int verbosity;
    double precision;
    double max_r_length;
    bool line_fit;
    vector<double> ellipse_ratios;
    bool fixed_radius;
    bool straight_only;
    int max_solver_time;
    bool recompute;
    string obstacle_pathname,geometric_obstacle_pathname;
    string pathgen_config_pathname;
};

struct EffectivePathGenOptions
{
    vector<double> start_length_extensions;
    vector<double> end_length_extensions;
    bool straight_only = false;
    bool line_fit = false;
    vector<double> ratios;
    bool fixed_radius = false;
    int samples = 0;
    vector<double> ellipse_ratios;
    bool ellipse_border_only = false;
};

struct AircraftGenerators
{
    std::unique_ptr<PathsGenerator> shortest_path_gen;
    std::unique_ptr<FittedPathsGenerator> path_gen;
};

static void normalize_pathgen_options(EffectivePathGenOptions& options)
{
    if (options.start_length_extensions.size() > 0 && options.end_length_extensions.size() == 0)
    {
        options.end_length_extensions = {0.};
    }

    if (options.end_length_extensions.size() > 0 && options.start_length_extensions.size() == 0)
    {
        options.start_length_extensions = {0.};
    }

    if (options.line_fit && options.ratios.size() == 0)
    {
        options.ratios = {0.,0.5,1.};
    }
}

static string autoremove_quotes(const string& s)
{
    if (s.size() >= 2 && s.front() == '\'' && s.back() == '\'')
    {
        return s.substr(1,s.size()-2);
    }
    else
    {
        return s;
    }
}

static EffectivePathGenOptions base_pathgen_options_from_args(const program_arguments& args)
{
    EffectivePathGenOptions options;
    options.start_length_extensions = args.start_length_extensions;
    options.end_length_extensions = args.end_length_extensions;
    options.straight_only = args.straight_only;
    options.line_fit = args.line_fit;
    options.ratios = args.ratios;
    options.fixed_radius = args.fixed_radius;
    options.samples = args.samples;
    options.ellipse_ratios = args.ellipse_ratios;
    options.ellipse_border_only = args.ellipse_border_only;
    normalize_pathgen_options(options);
    return options;
}

static void apply_pathgen_overrides(EffectivePathGenOptions& options, const DubinsPP::InputParser::PathGenerationOptionOverrides& overrides)
{
    if (overrides.extend_start.has_value())
    {
        options.start_length_extensions = overrides.extend_start.value();
    }
    if (overrides.extend_end.has_value())
    {
        options.end_length_extensions = overrides.extend_end.value();
    }
    if (overrides.straights_only.has_value())
    {
        options.straight_only = overrides.straights_only.value();
    }
    if (overrides.line_fit.has_value())
    {
        options.line_fit = overrides.line_fit.value();
    }
    if (overrides.line_ratios.has_value())
    {
        options.ratios = overrides.line_ratios.value();
    }
    if (overrides.fixed_radius.has_value())
    {
        options.fixed_radius = overrides.fixed_radius.value();
    }
    if (overrides.samples.has_value())
    {
        options.samples = overrides.samples.value();
    }
    if (overrides.ellipse_ratios.has_value())
    {
        options.ellipse_ratios = overrides.ellipse_ratios.value();
    }
    if (overrides.ellipse_border_only.has_value())
    {
        options.ellipse_border_only = overrides.ellipse_border_only.value();
    }
    normalize_pathgen_options(options);
}

static AircraftGenerators build_generators_from_options(const EffectivePathGenOptions& options, double precision)
{
    AircraftGenerators output;

    output.shortest_path_gen = std::make_unique<BasePathsGenerator>(precision);

    if (options.line_fit)
    {
        auto fitted_line_extensions = std::make_unique<FittedStraightBaseDubins>(options.ratios,precision);

        if (options.fixed_radius)
        {
            output.path_gen = std::move(fitted_line_extensions);
        }
        else
        {
            auto fitted_radius = std::make_unique<BaseFittedPathsGenerator>(precision);

            std::vector<std::unique_ptr<FittedPathsGenerator>> tmp;
            tmp.emplace_back(std::move(fitted_line_extensions));
            tmp.emplace_back(std::move(fitted_radius));
            output.path_gen = std::make_unique<AggregatedFittedPathsGenerator>(std::move(tmp));
        }
    }
    else
    {
        auto fitted_line_extensions = std::make_unique<FittedStraightBaseDubins>(options.ratios,precision);
        auto fitted_radius          = std::make_unique<BaseFittedPathsGenerator>(precision);

        std::vector<std::unique_ptr<FittedPathsGenerator>> tmp;
        tmp.emplace_back(std::move(fitted_line_extensions));
        tmp.emplace_back(std::move(fitted_radius));

        output.path_gen = std::make_unique<AggregatedFittedPathsGenerator>(std::move(tmp));
    }

    if (options.samples > 0)
    {
        if (options.ellipse_ratios.size() > 0)
        {
            if (options.ellipse_border_only)
            {
                output.path_gen = std::make_unique<FittedEllipseSampling>(
                    std::move(output.path_gen),options.ellipse_ratios,options.samples);
            }
            else
            {
                output.path_gen = std::make_unique<FittedRandomEllipseSampling>(
                    std::move(output.path_gen),options.ellipse_ratios,options.samples,0);
            }
        }
        else
        {
            output.path_gen = std::make_unique<FittedRandomEllipseSampling>(
                std::move(output.path_gen),1.,options.samples,0);
        }
    }

    if (options.start_length_extensions.size() || options.end_length_extensions.size())
    {
        if (options.straight_only)
        {
            output.path_gen = std::make_unique<StraightExtendedFittedPathsGenerator>(
                std::move(output.path_gen),options.start_length_extensions,options.end_length_extensions);
            output.shortest_path_gen = std::make_unique<StraightExtendedPathsGenerator>(
                std::move(output.shortest_path_gen),options.start_length_extensions,options.end_length_extensions);
        }
        else
        {
            output.path_gen = std::make_unique<AllExtendedFittedPathsGenerator>(
                std::move(output.path_gen),options.start_length_extensions,options.end_length_extensions);
            output.shortest_path_gen = std::make_unique<AllExtendedPathsGenerator>(
                std::move(output.shortest_path_gen),options.start_length_extensions,options.end_length_extensions);
        }
    }

    return output;
}


bool process_command_line(int argc, char* argv[], program_arguments& parsed_args)
{
    // Compute current time, for default file format
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    string datestring(std::ctime(&time_t));
    datestring += ".json";
    
    // Define options
    po::options_description desc("Allowed options");
    desc.add_options()
    ("input"        , po::value<string>(&parsed_args.src_pathname)->required()
                    , "REQUIRED, positional. Either a CSV file describing a problem, or a folder containing such files. See 'USAGE.md' for the format description")
    ("output"       , po::value<string>(&parsed_args.out_pathname)->required()
                    , "REQUIRED, positional. Output file/folder (if input is a folder). Format is deduced by extension type, either JSON or CSV."
                        " Format is described in 'USAGE.md'." )

    ("separation"   , po::value<double>(&parsed_args.separation)->required()
                    ,"REQUIRED, positional. Minimal required XY separation between aircraft")
    
    ("wind-x"       , po::value<string>(&parsed_args.wind_x_str)->default_value("0.")
                    ,"Positional. Wind speed along the X axis. Needs to be simple quoted, as in '-1.' (safeguard for negative values). Default to '0.'")
    
    ("wind-y"       , po::value<string>(&parsed_args.wind_y_str)->default_value("0.")
                    ,"Positional. Wind speed along the Y axis. Needs to be simple quoted, as in '-1.' (safeguard for negative values). Default to '0.'")

    ("reallocate,A" , po::bool_switch(&parsed_args.reallocate)->default_value(false)
                    , "Flag. If set, allows aircraft to switch endpoints.")

    ("obstacles,O"  , po::value<string>(&parsed_args.obstacle_pathname)->default_value("")
                    , "Path to a JSON file describing multiple paths to be considered as obstacles")
    ("geo-obstacles,G"  , po::value<string>(&parsed_args.geometric_obstacle_pathname)->default_value("")
                    , "Path to a JSON file describing multiple paths to be considered as geometric obstacles, i.e. time-independant barriers.")
    ("pathgen-config" , po::value<string>(&parsed_args.pathgen_config_pathname)->default_value("")
                    , "Path to a JSON file overriding individual path generation options per aircraft.")
    
    ("extend-start ", po::value<vector<double>>(&parsed_args.start_length_extensions)->multitoken()
                    ,"Use extended Dubins curves with the given lengths, for the starts")
    ("extend-end"   , po::value<vector<double>>(&parsed_args.end_length_extensions)->multitoken()
                    ,"Use extended Dubins curves with the given lengths, for the ends. If undefined while starts are, copy starts.")

    ("straights-only", po::bool_switch(&parsed_args.straight_only)->default_value(false)
                    , "Allow only straight extensions. Applies only if 'extend-' is used")

    ("allow-shortest", po::bool_switch(&parsed_args.allow_shortest)->default_value(false)
                    , "Only relevant for solving with timeslots. If true, allows adding the shortest paths (ignoring timeslots). Default to false.")
    
    ("line,l"       , po::bool_switch(&parsed_args.line_fit)->default_value(false)
                    , "Add paths with minimal turn radius and fitted extra straights at start and end")
    ("line-ratios"  , po::value<vector<double>>(&parsed_args.ratios)->multitoken()
                    , "If 'line' is activated, specifies the ratios (between 0 and 1) to use for extra straights generation. If undefined, default to 0, 0.5 and 1")
    ("fixed-radius" , po::bool_switch(&parsed_args.fixed_radius)->default_value(false)
                    , "If both this and 'line' are set, only use the minimal turn radius for Dubins paths. Otherwise, allow any radius (i.e. fitted). Default to false.")

    ("samples,S"    , po::value<int>(&parsed_args.samples)->default_value(0)
                    ,"Number of samples used by the sampling planner. Default to 0 (i.e. don't use sampling).")
    ("ellipse,e"    , po::value<vector<double>>(&parsed_args.ellipse_ratios)->multitoken()
                    ,"Use ellipses and specifies their radii as ratios of target length. Default to empty (i.e. don't use an ellipse).")
    ("ellipse-border-only", po::bool_switch(&parsed_args.ellipse_border_only)->default_value(false)
                    ,"If true, only samples on the border of the ellipse are used. Default to false.")

    ("threads,t"    , po::value<int>(&parsed_args.thread_num)->default_value(-1)
                    ,"Number of threads to use (if 0, the program chooses automatically). Disabled multi-threading by default.")

    
    ("precision,p"  , po::value<double>(&parsed_args.precision)->default_value(1e-3)
                    ,"Numeric precision for computations. Default to 1e-3.")
    ("max-r-length,r", po::value<double>(&parsed_args.max_r_length)->default_value(3.)
                    ,"Maximal relative length. Default to 3.")
    ("max-iters,I"  , po::value<int>(&parsed_args.max_iters)->default_value(300)
                    ,"Maximal number of iterations. Default to 300.")
    ("weave,w"      , po::value<int>(&parsed_args.weave_iters)->default_value(2)
                    ,"Number of samples to add when resampling between two points. Default to 2.")
    ("weave-dist"   , po::value<double>(&parsed_args.min_weave_dist)->default_value(0.1)
                    ,"Minimal value required between two time-points for ressampling between them (no ressampling if below). Default to 0.1")
    ("weave-dist-percent", po::value<double>(&parsed_args.min_weave_dist_percent)->default_value(0.0001)
                    ,"Minimal value required between two time-points for ressampling between them as a fraction of the maximal duration (no ressampling if below). Default to 0.0001")

    ("max-time,T"   , po::value<int>(&parsed_args.max_solver_time)->default_value(60)
                    , "Maximal allowed runtime for the solver, in seconds. Default to 60s.")

    ("recompute,R"  , po::bool_switch(&parsed_args.recompute)->default_value(false)
                    , "Force overwriting output file. Otherwise, skip cases where an output file is found. "
                      "Only relevant when parsing cases from directories. Default to False")
    
    ("verbose,v"    , po::value<int>(&parsed_args.verbosity)->default_value(1)
                    , "Set verbosity, from 0 (silent), to 3 (very very verbose). Default to 1.")
    ("export-samples,E"    , po::value<int>(&parsed_args.export_samples)->default_value(1000)
                    ,"Number of samples for display and/or exporting to CSV. Default to 1000.")
    ("help", "Produce help message")
    ;
    
    // Specify positional arguments
    po::positional_options_description p;
    p.add("input", 1);
    p.add("output", 1);
    p.add("separation", 1);
    p.add("wind-x", 1);
    p.add("wind-y", 1);
    
    // Parse
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
    options(desc).positional(p).run(), vm);
    
    // Check if help was asked (and if it is the case, display then indicate failure)
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return false;
    }
    
    try
    {
        // Check arguments are correct
        po::notify(vm); 
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        
        std::cout << desc << std::endl;
        return false;
    }
    catch(...)
    {
        std::cerr << "Unknown error!" << std::endl;
        return false;
    }
    
    // == Implement default behaviors ==
    // If only one of the two extensions is set, set the other to contain only 0 (no extension, but required for underlying code)
    if (parsed_args.start_length_extensions.size() > 0 && parsed_args.end_length_extensions.size() == 0)
    {
        parsed_args.end_length_extensions = {0.};
    }

    if (parsed_args.end_length_extensions.size() > 0 && parsed_args.start_length_extensions.size() == 0)
    {
        parsed_args.start_length_extensions = {0.};
    }

    // Default values for line-fitting ratios
    if (parsed_args.line_fit && parsed_args.ratios.size() == 0)
    {
        parsed_args.ratios = {0,0.5,1.};
    }

    // Manually parse wind values, to allow negative values (boost program_options does not allow negative values for positional arguments)
    try
    {
        parsed_args.wind_x = std::stod(autoremove_quotes(parsed_args.wind_x_str));
    }
    catch(const std::exception& e)
    {
        std::cerr << "Error while parsing wind-x value: " << e.what() << " with arguments: " << autoremove_quotes(parsed_args.wind_x_str) << std::endl;
        return false;
    }
    try
    {
        parsed_args.wind_y = std::stod(autoremove_quotes(parsed_args.wind_y_str));
    }
    catch(const std::exception& e)
    {
        std::cerr << "Error while parsing wind-y value: " << e.what() << " with arguments: " << autoremove_quotes(parsed_args.wind_y_str) << std::endl;
        return false;
    }


    return true;
}

/******************** Problem solving ********************/

void write_result(const fs::path& out_filepath, vector<Dubins>& sol, vector<AircraftStats>& stats, program_arguments& args)
{

    std::ofstream out_data(out_filepath);
    
    string ext;
    if (out_filepath.has_extension())
    {
        ext = out_filepath.extension();
    }
    else
    {
        ext = ".json";
    }
    
    // Put to lowercase
    lower_string(ext);

    if (args.verbosity >= DubinsFleetPlanner_VERBOSE)
    {
        std::cout << "Writing result at: " << out_filepath.string() << std::endl << std::endl;
    }
     
    
    if (ext == ".csv")
    {
        DubinsPP::OutputPrinter::print_paths_as_CSV(out_data,sol,stats,args.wind_x,args.wind_y,args.export_samples);
    }
    else
    {
        DubinsPP::OutputPrinter::print_paths_as_ModernJSON(out_data,sol,stats,args.separation,args.wind_x,args.wind_y);

        // fs::path outbis_filepath = out_filepath.string()+".bis.json";
        // std::cout << "Also printing at:" << std::endl << outbis_filepath.string() << std::endl;
        // std::ofstream outbis_data(outbis_filepath);

        // DubinsPP::OutputPrinter::print_paths_as_ModernJSON(outbis_data,sol,stats,args.separation,args.wind_x,args.wind_y);
    }
}

std::tuple<int,std::vector<Dubins>,ExtraPPResults> solve_case(const fs::path& input_path, const fs::path& output_path, program_arguments& args)
{

    // ----- Parsing obstacle trajectories if existing ----- //

    

    std::vector<Dubins> obstacle_paths = {};
    std::vector<AircraftStats> obstacle_stats = {};
    double __wind_x,__wind_y,__min_sep,__z_alpha;

    if (!args.obstacle_pathname.empty())
    {
        std::ifstream obstacle_src(args.obstacle_pathname);
        DubinsPP::InputParser::parse_paths_as_ModernJSON(obstacle_src,obstacle_paths,obstacle_stats,
            __min_sep,__wind_x,__wind_y,__z_alpha);       
    }
    

    std::vector<DynamicPathShape> geometric_obstacles = {};
    if (!args.geometric_obstacle_pathname.empty())
    {
        std::ifstream limits_src(args.geometric_obstacle_pathname);
        DubinsPP::InputParser::parse_pathshapes_ModernJSON(limits_src,geometric_obstacles);
    }

    // ----- Parsing case data ----- //

    ExtraPPResults extra;
    extra.case_name = input_path.string();

    if (!(fs::exists(input_path) && fs::is_regular_file(input_path)))
    {
        std::cerr   << "Error: input path is not a regular file (tried path: " << input_path << " )" << std::endl
        << "Exiting now..." << std::endl;
        return std::make_tuple(DubinsFleetPlanner_PARSING_FAILURE,std::vector<Dubins>{},extra);
    }

    if (args.verbosity >= DubinsFleetPlanner_VERBOSE)
    {
        std::cout << "Reading file: " << input_path << std::endl;
    }


    std::ifstream data_src(input_path);

    if (data_src.bad())
    {
        std::cerr   << "Error while reading trying to open the input file (tried path: " << input_path << " )" << std::endl
        << "Exiting now..." << std::endl;
        return std::make_tuple(DubinsFleetPlanner_PARSING_FAILURE,std::vector<Dubins>{},extra);
    }
    
    
    DubinsPP::InputParser::CaseData data = DubinsPP::InputParser::parse_data_csv(data_src);

    std::vector<Pose3D> starts,ends;
    std::vector<AircraftStats> stats;
    std::vector<double> dt;
    std::vector<std::vector<double>> timeslots;
    
    starts      = std::get<0>(data);
    ends        = std::get<1>(data);
    stats       = std::get<2>(data);
    dt          = std::get<3>(data);
    timeslots   = std::get<4>(data);

    DubinsPP::InputParser::PathGenerationConfig pathgen_config;
    if (!args.pathgen_config_pathname.empty())
    {
        std::ifstream cfg_src(args.pathgen_config_pathname);
        if (cfg_src.bad())
        {
            std::cerr   << "Error while trying to open path generation config file (tried path: "
                        << args.pathgen_config_pathname << " )" << std::endl
                        << "Exiting now..." << std::endl;
            return std::make_tuple(DubinsFleetPlanner_PARSING_FAILURE,std::vector<Dubins>{},extra);
        }

        try
        {
            pathgen_config = DubinsPP::InputParser::parse_path_generation_config_json(cfg_src);
        }
        catch(const std::exception& e)
        {
            std::cerr   << "Error while parsing path generation config file: " << e.what() << std::endl
                        << "Exiting now..." << std::endl;
            return std::make_tuple(DubinsFleetPlanner_PARSING_FAILURE,std::vector<Dubins>{},extra);
        }
    }

    // Check if timeslots is empty:

    bool empty_timeslots = true;
    for(const auto& v : timeslots)
    {
        if (v.size() > 0)
        {
            empty_timeslots = false;
            break;
        }
    }
    
    // ----- Start optimization ----- //
    
    bool good_solution = true;

    // string output_log_pathname = output_path.string() + ".log.json";
    // std::ofstream output_log(output_log_pathname);

    EffectivePathGenOptions base_options = base_pathgen_options_from_args(args);
    std::vector<AircraftGenerators> per_ac_generators;
    per_ac_generators.reserve(stats.size());

    for(const AircraftStats& ac_stat : stats)
    {
        EffectivePathGenOptions options = base_options;

        apply_pathgen_overrides(options,pathgen_config.defaults);

        auto ac_it = pathgen_config.per_aircraft.find(ac_stat.id);
        if (ac_it != pathgen_config.per_aircraft.end())
        {
            apply_pathgen_overrides(options,ac_it->second);
        }

        if (options.samples < 0)
        {
            std::cerr << "Error: negative 'samples' value after pathgen config merge for aircraft id "
                      << ac_stat.id << " (got " << options.samples << " )." << std::endl
                      << "Exiting now..." << std::endl;
            return std::make_tuple(DubinsFleetPlanner_PARSING_FAILURE,std::vector<Dubins>{},extra);
        }

        per_ac_generators.push_back(build_generators_from_options(options,args.precision));
    }

    if (per_ac_generators.size() == 0)
    {
        std::cerr << "Error: no aircraft found in input case." << std::endl
                  << "Exiting now..." << std::endl;
        return std::make_tuple(DubinsFleetPlanner_PARSING_FAILURE,std::vector<Dubins>{},extra);
    }

    std::unique_ptr<FleetPlanner> planner = std::make_unique<FleetPlanner>(
        *per_ac_generators[0].shortest_path_gen,*per_ac_generators[0].path_gen,
        args.precision,args.max_r_length,args.verbosity);

    std::vector<std::reference_wrapper<const PathsGenerator>> shortest_refs;
    std::vector<std::reference_wrapper<const FittedPathsGenerator>> fitted_refs;
    shortest_refs.reserve(per_ac_generators.size());
    fitted_refs.reserve(per_ac_generators.size());

    for(const AircraftGenerators& ac_gen : per_ac_generators)
    {
        shortest_refs.push_back(*ac_gen.shortest_path_gen);
        fitted_refs.push_back(*ac_gen.path_gen);
    }

    planner->set_aircraft_generators(shortest_refs,fitted_refs);
    
    if (args.verbosity >= DubinsFleetPlanner_VERBOSE)
    {
        std::cout << "Starting optimization..." << std::endl;
    }

    auto start = chrono::thread_clock::now(boost::throws());
    
    std::vector<Dubins> sols;

    if (!empty_timeslots)
    {
        planner->solve_with_timeslots<Dubins::are_XY_separated,Dubins::compute_XY_distance>(sols,extra,starts,ends,stats,args.separation,timeslots,
            args.wind_x,args.wind_y,args.thread_num,args.allow_shortest,
            obstacle_paths,obstacle_stats,geometric_obstacles
            );
    }
    else
    {
        planner->solve<Dubins::are_XY_separated,Dubins::compute_XY_distance>(sols,extra,starts,ends,stats,
            args.separation,dt,
            args.wind_x,args.wind_y,
            args.reallocate,
            args.max_iters,
            args.weave_iters,args.min_weave_dist,args.min_weave_dist_percent,
            args.thread_num,
            args.max_solver_time,
            obstacle_paths,obstacle_stats,
            geometric_obstacles);
    }

    extra.duration = chrono::thread_clock::now(boost::throws()) - start;
    
    // if (sols.size() != stats.size()) // If no solution, retry without separation
    // {
    //     ExtraPPResults _backup;
    //     good_solution = false;
    //     std::cerr << "WARNING: Could not find a solution; retrying with HALF SEPARATION ( " << args.separation/2 << " )" << std::endl;
        
    //     if (!empty_timeslots)
    //     {
    //         planner->solve_with_timeslots<Dubins::are_XY_separated,Dubins::compute_XY_distance>(sols,_backup,starts,ends,stats,0,timeslots,
    //             args.wind_x,args.wind_y,args.thread_num,args.allow_shortest,
    //             obstacle_paths,obstacle_stats,geometric_obstacles
    //             );
    //     }
    //     else
    //     {
    //         planner->solve<Dubins::are_XY_separated,Dubins::compute_XY_distance>(sols,_backup,starts,ends,stats,
    //             args.separation/2,dt,
    //             args.wind_x,args.wind_y,
    //             args.reallocate,
    //             args.max_iters,
    //             args.weave_iters,args.min_weave_dist,args.min_weave_dist_percent,
    //             args.thread_num,
    //             args.max_solver_time,
    //             obstacle_paths,obstacle_stats,
    //             geometric_obstacles);
    //     }
    // }
        
    if (sols.size() != stats.size())
    {
        std::cerr << "ERROR: Could not find a solution!" << std::endl;
        return std::make_tuple(DubinsFleetPlanner_PARSING_FAILURE,std::vector<Dubins>{},extra);
    }
    else
    {
        if (good_solution)
        {
            extra.false_positive = !check_solution<Dubins::are_XY_separated_sampling>(sols,stats,args.separation);
            if (extra.false_positive)
            {
                good_solution = false;
                std::cerr << "ERROR: False positive detected!!!" << std::endl;
            }
        }

        obstacle_paths.insert(obstacle_paths.end(),
            std::move_iterator(sols.begin()),std::move_iterator(sols.end()));

        obstacle_stats.insert(obstacle_stats.end(),
            stats.begin(),stats.end());

        write_result(output_path,obstacle_paths,obstacle_stats,args);

        if (good_solution)
        {
            return std::make_tuple(DubinsFleetPlanner_SOLVING_SUCCESS,sols,extra);
        }
        else
        {
            return std::make_tuple(DubinsFleetPlanner_SOLVING_FAILURE,sols,extra);
        }
    }
}



/******************** Main entrypoint ********************/

int main(int argc, char *argv[])
{
    
    // ----- Parse arguments ----- //
    
    program_arguments args;
    
    bool successful_parse = process_command_line(argc,argv,args);
    
    if (!successful_parse) {exit(0);}
    
    if (args.max_iters <= 0)
    {
        std::cerr   << "Maximal number of iterations is non-positive (Got: " << args.max_iters << " )!" << std::endl 
        << "Exiting now..." << std::endl; 
        exit(DubinsFleetPlanner_PARSING_FAILURE);
    }
    
    if (args.export_samples < 3)
    {
        std::cerr   << "Number of samples is too small, ie less than 3 (Got: " << args.export_samples << " )!" << std::endl 
        << "Exiting now..." << std::endl; 
        exit(DubinsFleetPlanner_PARSING_FAILURE);
    }

    if (args.samples < 0)
    {
        std::cerr   << "Number of samples is negative (Got: " << args.samples << " )!" << std::endl 
        << "Exiting now..." << std::endl; 
        exit(DubinsFleetPlanner_PARSING_FAILURE);
    }
    
    // ----- Parse input file ----- //
    
    fs::path src_path(args.src_pathname);
    fs::path out_path(args.out_pathname);
    

    if (fs::is_directory(src_path))
    {
        // Handle a directory: several test cases

        uint testcase_count = 0;
        uint success_count  = 0;

        vector<fs::path> failures;

        chrono::nanoseconds total_cpu_time(0);
        long unsigned int total_iterations = 0;
        
        if (!fs::is_directory(out_path))
        {
            if (!fs::create_directory(out_path))
            {
                std::cerr << "ERROR: Failure when creating ouput directory: " << out_path << std::endl
                << "Exiting now..." << std::endl;

                exit(DubinsFleetPlanner_PARSING_FAILURE);
            }
        }

        fs::path summary_file_path = out_path / "summary.csv";
        std::ofstream summary_file;
        bool add_header = true;

        if (fs::exists(summary_file_path) && !args.recompute)
        {
            summary_file = std::ofstream(summary_file_path,std::ios_base::app | std::ios_base::out);
            add_header = false;
        }
        else
        {
            summary_file = std::ofstream(summary_file_path,std::ios_base::out);
        }

        if (summary_file.bad())
        {
            std::cerr << "ERROR: Could not create summary output file at: " << summary_file_path << std::endl
            << "Exiting now..." << std::endl;

            exit(DubinsFleetPlanner_PARSING_FAILURE);
        }

        if (add_header)
        {
            summary_file << ExtraPPResults::CSV_header() << std::endl;
        }
        
        for(auto srcfile : fs::directory_iterator(src_path))
        {
            if (fs::is_regular_file(srcfile))
            {
                fs::path out_filepath(out_path);
                string ext = "sol";
                ext.append((out_path.has_extension()) ? (out_path.extension()) : (".json"));

                out_filepath /= srcfile.path().filename();
                out_filepath.replace_extension(ext);

                if (!args.recompute && fs::exists(out_filepath))
                {
                    if (args.verbosity >= DubinsFleetPlanner_VERBOSE)
                    {
                        std::cout << "File: " << srcfile << " is already solved at " << out_filepath << std::endl;
                    }
                    continue;
                }

                auto __ret = solve_case(srcfile,out_filepath,args);

                int code = std::get<0>(__ret);
                ExtraPPResults& extra = std::get<2>(__ret);

                total_cpu_time      += extra.duration;
                total_iterations    += extra.iterations;

                if (code == DubinsFleetPlanner_SOLVING_SUCCESS)
                {
                    success_count++;
                }
                else
                {
                    failures.push_back(srcfile.path());
                }

                testcase_count++;

                summary_file << extra.as_CSV() << std::endl;
            }
        }

        summary_file.close();

        
        std::cout   << "List of failures:" << std::endl;
        
        for(auto& inpath : failures)
        {
            std::cout << inpath << std::endl;
        }

        std::cout   << std::endl
                    << "Succes ratio: " << success_count << " / " << testcase_count << std::endl
                    << "Succes rate : " << 100*(static_cast<double>(success_count)/static_cast<double>(testcase_count)) << " %" << std::endl 
                    << "Average time per case (s)       : " << chrono::duration<double>(total_cpu_time)/(testcase_count) << std::endl
                    << "Average time per iteration (s)  : " << chrono::duration<double>(total_cpu_time)/(total_iterations) << std::endl
                    << std::endl;
    }
    else
    {
        // Handle a single file
        auto __ret = solve_case(src_path,out_path,args);
        
        int code = std::get<0>(__ret);
        const ExtraPPResults& extra = std::get<2>(__ret);

        std::cout << extra.format() << std::endl;

        exit(code);
    }
}

        