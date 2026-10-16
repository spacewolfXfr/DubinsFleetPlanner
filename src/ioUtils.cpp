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

#include "ioUtils.hpp"
#include <limits>

using json = nlohmann::json;

// ---------------------------------------- Misc function ---------------------------------------- //

static double compute_plan_duration(const std::vector<Dubins>& paths)
{
    uint N = paths.size();

    double max_time = 0;

    for(uint i = 0; i < N; i++)
    {
        double time = paths[i].get_duration();
        max_time = std::max(time,max_time);
    }

    return max_time;
}

// ---------- nlohmann (De)Serialization (JSON) ---------- //

namespace {

    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Pose3D,x,y,z,theta);

    template<DubinsMove m>
    void to_json(json& j, const PathShape<m>& p)
    {
        j = json{
            {"x"    ,p.x},
            {"y"    ,p.y},
            {"z"    ,p.z},
            {"p1"   ,p.p1},
            {"p2"   ,p.p2},
            {"p3"   ,p.p3},
            {"p4"   ,p.p4},
            {"type" ,get_DubinsMove_name(m)},
            {"m"    ,m}
        };
    }

    void to_json(json& j, const DynamicPathShape& p)
    {
        j = json{
            {"x"        ,p.x},
            {"y"        ,p.y},
            {"z"        ,p.z},
            {"p1"       ,p.p1},
            {"p2"       ,p.p2},
            {"p3"       ,p.p3},
            {"p4"       ,p.p4},
            {"type"     ,get_DubinsMove_name(p.m)},
            {"m"        ,p.m},
            {"length"   ,p.length}
        };
    }

    void from_json(const json& j, DynamicPathShape& p)
    {
        p.x     = j.at("x");
        p.y     = j.at("y");
        p.z     = j.at("z");
        p.p1    = j.at("p1");
        p.p2    = j.at("p2");
        p.p3    = j.at("p3");
        p.p4    = j.at("p4");
        p.m     = j.at("m");
        p.length= j.at("length");
    }

    void to_json(json& j, const AircraftStats& s)
    {
        j = json{
            {"id"               ,s.id},
            {"airspeed"         ,s.airspeed},
            {"target_airspeed"  ,s.target_airspeed},
            {"climb"            ,s.climb},
            {"turn_radius"      ,s.turn_radius},
            {"min_acceleration" ,s.min_acceleration},
            {"max_acceleration" ,s.max_acceleration}
        };
    }

    void from_json(const json& j, AircraftStats& s)
    {
        s.id               = j.at("id");
        s.airspeed         = j.at("airspeed");
        s.climb            = j.at("climb");
        s.turn_radius      = j.at("turn_radius");
        try
        {
            s.target_airspeed  = j.at("target_airspeed");
        }
        catch (nlohmann::json::exception& e)
        {
            s.target_airspeed = s.airspeed;
        }
        try
        {
            s.min_acceleration  = j.at("min_acceleration");
        }
        catch (nlohmann::json::exception& e)
        {
            s.min_acceleration = 0.;
        }
        try
        {
            s.max_acceleration  = j.at("max_acceleration");
        }
        catch (nlohmann::json::exception& e)
        {
            s.max_acceleration = 0.;
        }
    }

    void to_json_RichConflict(json& j, const RichConflict_T& t)
    {
        j = json{
            {"AC_id1"  ,std::get<0>(t)},
            {"Path_id1",std::get<1>(t)},
            {"AC_id2"  ,std::get<2>(t)},
            {"Path_id2",std::get<3>(t)},
            {"min_loc" ,std::get<4>(t)},
            {"min_val" ,std::get<5>(t)}
        };
    }

    void from_json_RichConflict(const json& j, RichConflict_T& t)
    {
        std::get<0>(t) = j.at("AC_id1"  );
        std::get<1>(t) = j.at("Path_id1");
        std::get<2>(t) = j.at("AC_id2"  );
        std::get<3>(t) = j.at("Path_id2");
        std::get<4>(t) = j.at("min_loc" );
        std::get<5>(t) = j.at("min_val" );
    }
}

// ---------------------------------------- Input handling ---------------------------------------- //

typedef DubinsPP::InputParser::RowInfo RowInfo;
typedef DubinsPP::InputParser::CaseData CaseData;

template<typename T>
static inline std::from_chars_result from_stringview(const std::string_view& v, T& t)
{
    return std::from_chars(v.begin(), v.end(), t);
}

static inline void check_from_chars_result(const std::from_chars_result& res, const std::string_view& src)
{
    if (res.ec != std::errc())
    {
        
        std::cout << "Failed parsing:" << std::endl << src << std::endl;

        if (res.ec == std::errc::invalid_argument)
        {
            std::cout << " -> Not a number" << std::endl;
        }
        else if (res.ec == std::errc::result_out_of_range)
        {
            std::cout << " -> Number too big" << std::endl;
        }
    }

    assert(res.ec == std::errc());
}

template<typename T>
static void failable_from_stringview(const std::string_view& v, T& t)
{
    check_from_chars_result(from_stringview(v,t),v);
}

struct CsvColumnLayout
{
    std::size_t id;
    std::size_t start_x;
    std::size_t start_y;
    std::size_t start_z;
    std::size_t start_theta;
    std::size_t end_x;
    std::size_t end_y;
    std::size_t end_z;
    std::size_t end_theta;
    std::size_t airspeed;
    std::size_t climb;
    std::size_t turn_radius;
    std::size_t dt;
    std::size_t target_airspeed = std::string_view::npos;
    std::size_t min_acceleration = std::string_view::npos;
    std::size_t max_acceleration = std::string_view::npos;
};

static std::string_view without_line_ending(std::string_view value)
{
    while (!value.empty() && (value.back() == '\r' || value.back() == '\n'))
    {
        value.remove_suffix(1);
    }
    return value;
}

static bool parse_csv_layout(const CSVRow& header, CsvColumnLayout& layout)
{
    auto find_column = [&header](std::string_view name) {
        for (std::size_t i = 0; i < header.size(); ++i)
        {
            if (without_line_ending(header[i]) == name)
            {
                return i;
            }
        }
        return std::string_view::npos;
    };

    layout.id = find_column("ac_id");
    layout.start_x = find_column("start_x");
    layout.start_y = find_column("start_y");
    layout.start_z = find_column("start_z");
    layout.start_theta = find_column("start_theta");
    layout.end_x = find_column("end_x");
    layout.end_y = find_column("end_y");
    layout.end_z = find_column("end_z");
    layout.end_theta = find_column("end_theta");
    layout.airspeed = find_column("airspeed");
    layout.climb = find_column("climb");
    layout.turn_radius = find_column("turn_radius");
    layout.dt = find_column("dt");
    layout.target_airspeed = find_column("target_airspeed");
    layout.min_acceleration = find_column("min_acceleration");
    layout.max_acceleration = find_column("max_acceleration");

    return layout.id != std::string_view::npos &&
        layout.start_x != std::string_view::npos &&
        layout.start_y != std::string_view::npos &&
        layout.start_z != std::string_view::npos &&
        layout.start_theta != std::string_view::npos &&
        layout.end_x != std::string_view::npos &&
        layout.end_y != std::string_view::npos &&
        layout.end_z != std::string_view::npos &&
        layout.end_theta != std::string_view::npos &&
        layout.airspeed != std::string_view::npos &&
        layout.climb != std::string_view::npos &&
        layout.turn_radius != std::string_view::npos &&
        layout.dt != std::string_view::npos;
}

/**
 * @brief Check if the CSV row matches the expected header row
 * 
 * Reference CSV header: ac_id,start_x,start_y,start_z,start_theta,end_x,end_y,end_z,end_theta,airspeed,climb,turn_radius,dt
 * 
 * @param r The first CSV row of the file
 * @return true 
 * @return false 
 */
bool check_header(const CSVRow& r)
{
    CsvColumnLayout layout;
    return parse_csv_layout(r, layout);
}

static RowInfo parse_row(const CSVRow& r, const CsvColumnLayout& layout)
{
    RowInfo o;

    std::from_chars_result res;

    failable_from_stringview(r[layout.id] ,o.stats.id);

    failable_from_stringview(r[layout.start_x] ,o.start.x);
    failable_from_stringview(r[layout.start_y] ,o.start.y);
    failable_from_stringview(r[layout.start_z] ,o.start.z);
    failable_from_stringview(r[layout.start_theta] ,o.start.theta);

    failable_from_stringview(r[layout.end_x] ,o.end.x);
    failable_from_stringview(r[layout.end_y] ,o.end.y);
    failable_from_stringview(r[layout.end_z] ,o.end.z);
    failable_from_stringview(r[layout.end_theta] ,o.end.theta);

    failable_from_stringview(r[layout.airspeed] ,o.stats.airspeed);
    failable_from_stringview(r[layout.climb],o.stats.climb);
    failable_from_stringview(r[layout.turn_radius],o.stats.turn_radius);

    o.stats.target_airspeed = o.stats.airspeed;
    o.stats.min_acceleration = 0.;
    o.stats.max_acceleration = 0.;

    if (layout.target_airspeed != std::string_view::npos)
    {
        failable_from_stringview(r[layout.target_airspeed],o.stats.target_airspeed);
    }
    if (layout.min_acceleration != std::string_view::npos)
    {
        failable_from_stringview(r[layout.min_acceleration],o.stats.min_acceleration);
    }
    if (layout.max_acceleration != std::string_view::npos)
    {
        failable_from_stringview(r[layout.max_acceleration],o.stats.max_acceleration);
    }

    failable_from_stringview(r[layout.dt],o.dt);

    for(std::size_t i = layout.dt + 1; i < r.size(); i++)
    {
        if (i == layout.target_airspeed || i == layout.min_acceleration || i == layout.max_acceleration)
        {
            continue;
        }
        double tslot;
        auto ret = from_stringview(r[i],tslot);
        if (ret.ec == std::errc())
        {
            o.timeslots.push_back(tslot);
        }
    }

    return o;
}

static CaseData tranpose_data(const std::vector<RowInfo>& rows)
{
    uint N = rows.size();

    std::vector<Pose3D> starts(N);
    std::vector<Pose3D> ends(N);
    std::vector<AircraftStats> stats(N);
    std::vector<double> dt(N-1);
    std::vector<std::vector<double>> timeslots(N);

    for(uint i = 0; i < N; i++)
    {
        if (i > 0)
        {
            dt[i-1] = rows[i].dt;
        }

        starts[i]       = rows[i].start;
        ends[i]         = rows[i].end;
        stats[i]        = rows[i].stats;
        timeslots[i]    = rows[i].timeslots;
    }

    return std::make_tuple(starts,ends,stats,dt,timeslots);
}

CaseData DubinsPP::InputParser::parse_data_csv(std::istream& stream)
{
    bool first = true;
    CsvColumnLayout layout;
    std::vector<RowInfo> parsed_rows;

    for(auto &row: CSVRange(stream,';'))
    {
        if (first)
        {
            if (!(parse_csv_layout(row, layout)))
            {
                std::cout << std::endl << "========== WRONG HEADER! ==========" << std::endl << "Exiting now..." << std::endl; 
                exit(1);
            }
            first = false;
            continue;
        }

        parsed_rows.push_back(parse_row(row, layout));
    }

    return tranpose_data(parsed_rows);

}

static DubinsPP::InputParser::PathGenerationOptionOverrides parse_path_generation_option_overrides(const json& j)
{
    using DubinsPP::InputParser::PathGenerationOptionOverrides;

    PathGenerationOptionOverrides out;

    if (j.contains("extend-start"))
    {
        out.extend_start = j.at("extend-start").get<std::vector<double>>();
    }
    if (j.contains("extend-end"))
    {
        out.extend_end = j.at("extend-end").get<std::vector<double>>();
    }
    if (j.contains("straights-only"))
    {
        out.straights_only = j.at("straights-only").get<bool>();
    }
    if (j.contains("line"))
    {
        out.line_fit = j.at("line").get<bool>();
    }
    if (j.contains("line-ratios"))
    {
        out.line_ratios = j.at("line-ratios").get<std::vector<double>>();
    }
    if (j.contains("fixed-radius"))
    {
        out.fixed_radius = j.at("fixed-radius").get<bool>();
    }
    if (j.contains("samples"))
    {
        out.samples = j.at("samples").get<int>();
    }
    if (j.contains("ellipse"))
    {
        out.ellipse_ratios = j.at("ellipse").get<std::vector<double>>();
    }
    if (j.contains("ellipse-border-only"))
    {
        out.ellipse_border_only = j.at("ellipse-border-only").get<bool>();
    }

    return out;
}

DubinsPP::InputParser::PathGenerationConfig DubinsPP::InputParser::parse_path_generation_config_json(std::istream& stream)
{
    PathGenerationConfig out;
    json j;
    stream >> j;

    if (j.contains("default"))
    {
        out.defaults = parse_path_generation_option_overrides(j.at("default"));
    }

    if (j.contains("aircraft"))
    {
        const json& ac = j.at("aircraft");
        if (!ac.is_object())
        {
            throw std::invalid_argument("Invalid pathgen config: 'aircraft' must be a JSON object keyed by aircraft id.");
        }

        for (auto it = ac.begin(); it != ac.end(); ++it)
        {
            int ac_id = 0;
            auto conv = std::from_chars(it.key().data(), it.key().data() + it.key().size(), ac_id);
            if (conv.ec != std::errc())
            {
                throw std::invalid_argument("Invalid aircraft id key in pathgen config: '" + it.key() + "'.");
            }

            out.per_aircraft[ac_id] = parse_path_generation_option_overrides(it.value());
        }
    }

    return out;
}

// ---------- nlohmann Deserialization (JSON) ---------- //

void parse_json_ModernTrajectory(const json& j, 
    Dubins& path,
    AircraftStats& stats)
{
    from_json(j.at("stats"),stats);

    json j_path = j.at("path");

    size_t section_count = j_path.at("sections_count");
    Pose3D start,end;
    from_json(j_path.at("start"),start);
    from_json(j_path.at("end"),end);

    double total_length = j_path.at("total_length");
    
    std::vector<DynamicPathShape> basic_paths;
    for(const json &j_bpath : j_path.at("sections"))
    {
        DynamicPathShape bshape;
        from_json(j_bpath,bshape);
        basic_paths.push_back(bshape);
    }

    path = Dubins(start,end,basic_paths);
    assert(pose_dist(path.get_start(),start) < DubinsFleetPlanner_PRECISION);
    assert(pose_dist(path.get_end(),end) < DubinsFleetPlanner_PRECISION);
}

void DubinsPP::InputParser::parse_paths_as_ModernJSON(std::istream& s, 
    std::vector<Dubins>& paths,
    std::vector<AircraftStats>& stats,
    std::vector<double>& delays,
    double& min_sep,
    double& wind_x, double& wind_y,
    double& z_alpha)
{
    json j;

    s >> j;

    min_sep         = j.at("separation");
    z_alpha         = j.at("z_alpha");
    wind_x          = j.at("wind_x");
    wind_y          = j.at("wind_y");
    double duration = j.at("duration");
    uint N          = j.at("AC_num");

    {
        std::vector<json> trajectories = j.at("trajectories");

        assert(N == trajectories.size());

        for(uint i = 0; i < N; i++)
        {
            json j_trajectory = trajectories[i];
            Dubins path;
            AircraftStats stat;
            parse_json_ModernTrajectory(j_trajectory,path,stat);
            paths.push_back(path);
            stats.push_back(stat);
            double delay = 0.;
            if (j_trajectory.contains("delay"))
            {
                delay = j_trajectory.at("delay");
            }
            delays.push_back(delay);

        }

    }
}

void DubinsPP::InputParser::parse_pathshapes_ModernJSON(std::istream& s, std::vector<DynamicPathShape>& shapes)
{
    json j;

    s >> j;

    for(const json &j_bpath : j.at("sections"))
    {
        DynamicPathShape bshape;
        from_json(j_bpath,bshape);
        shapes.push_back(bshape);
    }
}
            

// ---------------------------------------- Output handling ---------------------------------------- //

// ----- Conflicts ----- //

void DubinsPP::OutputPrinter::append_rich_conflicts(std::ostream& s,
        double time,
        const std::vector<RichConflict_T>& vec,
        const SharedListOfPossibilities& possibilities,
        const std::vector<AircraftStats>& stats,
        bool prepend_comma)
{

    json j;

    j["time"] = time;

    std::vector<json> conflicts;

    for(auto v: vec)
    {
        auto ac_id1     = std::get<0>(v);
        auto path_id1   = std::get<1>(v);
        auto ac_id2     = std::get<2>(v);
        auto path_id2   = std::get<3>(v);

        json j_conflict = json{
            {"AC_id1"       , stats[ac_id1].id},
            {"Path_type1"   , possibilities[ac_id1][path_id1]->get_type_abbr()},
            {"AC_id2"       , stats[ac_id2].id},
            {"Path_type2"   , possibilities[ac_id2][path_id2]->get_type_abbr()},
            {"min_loc"      , std::get<4>(v)},
            {"min_val"      , std::get<5>(v)}
        };

        conflicts.push_back(j_conflict);
    }

    j["conflicts"] = conflicts;

    if (prepend_comma)
    {
        s << ",";
    }

    s << j;
}

// ---------- nlohmann Serialization (JSON) ---------- //

void to_json_ModernTrajectory(json& j, 
    const Dubins& path,
    const AircraftStats& stats,
    double wind_x, double wind_y)
{

    json j_stats;
    to_json(j_stats,stats);

    j["stats"] = j_stats;

    json j_path;
    
    j_path["total_length"] = path.get_length();

    json j_start,j_end;
    to_json(j_start,path.get_start());
    to_json(j_end,path.get_end());

    j_path["start"] = j_start;
    j_path["end"]   = j_end;

    std::vector<json> sections;

    {
        for(DynamicPathShape s : path.get_all_sections())
        {
            if (s.length < 1e-9) continue;
            
            s.set_planar_speed(stats.airspeed);
            
            json j_section;

            to_json(j_section,s);

            sections.push_back(j_section);
        }

        j_path["sections"] = sections;
    }

    j_path["sections_count"] = sections.size();
    j["path"] = j_path;
}

void DubinsPP::OutputPrinter::print_paths_as_ModernJSON(std::ostream& s, 
            const std::vector<Dubins>& paths,
            const std::vector<AircraftStats>& stats,
            double min_sep,
            double wind_x, double wind_y,
            double z_alpha)
{
    using json = nlohmann::json;

    assert(paths.size() == stats.size());

    double duration = compute_plan_duration(paths);
    uint N = paths.size();

    json j;

    j["separation"] = min_sep;
    j["z_alpha"]    = z_alpha;
    j["wind_x"]     = wind_x;
    j["wind_y"]     = wind_y;
    j["duration"]   = duration;
    j["AC_num"]     = N;

    {
        std::vector<json> trajectories;
        
        for(uint i = 0; i < N; i++)
        {
            json j_trajectory;
            to_json_ModernTrajectory(j_trajectory,paths[i],stats[i],wind_x,wind_y);
            trajectories.push_back(j_trajectory);
        }

        j["trajectories"] = trajectories;
    }

    s << j;
}

// ---------- Obstacles Serialization (JSON) ---------- //

void DubinsPP::OutputPrinter::print_obstacles_collection(std::ostream& s, const std::vector<DynamicPathShape>& obstacles)
{
    using json = nlohmann::json;
    json j;

    std::vector<json> j_obstacles;
    for(const DynamicPathShape& obs : obstacles)
    {
        json j_obs;
        to_json(j_obs,obs);
        j_obstacles.push_back(j_obs);
    }

    j["sections"] = j_obstacles;
    s << j;
}

// ---------- Handmade Serialization (CSV) ---------- //

void DubinsPP::OutputPrinter::print_paths_as_CSV(std::ostream& s, 
    const std::vector<Dubins>& paths,
    const std::vector<AircraftStats>& stats,
    double wind_x, double wind_y, uint samples)
{
    assert(paths.size() == stats.size());
    assert(samples > 1);

    double duration = compute_plan_duration(paths);

    uint N = paths.size();

    // Write header
    s << "time";
    
    for(uint i = 0; i < N; i++)
    {
        s   << ";X_"     << stats[i].id
            << ";Y_"     << stats[i].id
            << ";Z_"     << stats[i].id
            << ";theta_" << stats[i].id;
    }
    s << std::endl;

    // Compute data
    std::vector<double> times;
    std::vector<std::vector<Pose3D>> pts_storages(N);

    for(uint i = 0; i < samples; i++)
    {
        times.push_back(i*duration/(samples-1));
    }
    
    for(uint i = 0; i < N; i++)
    {
        pts_storages[i] = paths[i].get_positions_at_times(times,true);
    }

    // Print data
    for(uint t_i = 0; t_i < samples; t_i++)
    {
        // Compute wind deflection
        double dx = wind_x*times[t_i];
        double dy = wind_y*times[t_i];

        s   << times[t_i];

        for(uint i = 0; i < N; i++)
        {
            Pose3D pt = pts_storages[i][t_i];
            
            s   << ";" << dx + pt.x 
                << ";" << dy + pt.y
                << ";" << pt.z
                << ";" << pt.theta;
        }

        s << std::endl;
    }
}


