#include <boost/program_options.hpp>

#include <iostream>
#include <unistd.h>
#include <filesystem>

#include "plotDubins.hpp"

#include "ioUtils.hpp"
#include "Aircraft.h"

namespace fs = std::filesystem;
namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    string json_pathname;

    // ----- Parsing using Program Options ----- //

    po::options_description desc("Allowed options");

    desc.add_options()
    ("input"    , po::value<string>(&json_pathname)->required()
                , "REQUIRED, positional. A JSON file describing a fleet movement")
    ("help", "Produce help message");
    
    // Specify positional arguments
    po::positional_options_description p;
    p.add("input", 1);

    // Parse
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
    options(desc).positional(p).run(), vm);
    
    // Check if help was asked (and if it is the case, display then indicate failure)
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
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
        return 1;
    }
    catch(...)
    {
        std::cerr << "Unknown error!" << std::endl;
        return 1;
    }

    // ----- Opening file ----- //
    fs::path json_path(json_pathname);

    if (!fs::is_regular_file(json_path))
    {
        std::cerr << json_pathname << " does not point to a regular file!" << std::endl;
        return 1;
    }

    std::ifstream json_stream(json_path);

    // ----- Setting up variables and parsing ----- //


    std::vector<Dubins> paths;
    std::vector<AircraftStats> stats;
    std::vector<double> delays;
    double min_sep;
    double wind_x, wind_y;
    double z_alpha;

    DubinsPP::InputParser::parse_paths_as_ModernJSON(json_stream,
        paths,stats,delays,min_sep,wind_x,wind_y,z_alpha);

    // ----- Plotting ----- //

    Visualisation::Plot2D plot = Visualisation::init_plot();
    Visualisation::plot_multiple_paths<200>(plot, paths, stats, wind_x, wind_y);

    sciplot::Figure fig     = {{plot}};
    sciplot::Canvas canvas  = {{fig}};
    canvas.size(1920,1080);
    canvas.show();
}