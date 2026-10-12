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

#include "ConflictList.hpp"

// ---------------------------------------- Util functions ---------------------------------------- //

std::vector<Conflict_T> drop_conflict_details(const std::vector<RichConflict_T>& vec, double min_dist)
{
    std::vector<Conflict_T> output;
    for(size_t i = 0; i < vec.size(); i++)
    {
        auto t = vec[i];

        // Only keep significant conflicts
        if (std::get<5>(t) > min_dist)
        {
            continue;
        }
        else
        {
            output.push_back(std::make_tuple(
                std::get<0>(t),
                std::get<1>(t),
                std::get<2>(t),
                std::get<3>(t)
            ));
        }
    }
    return output;
}

uint number_of_valid_paths(ListOfPossibilities& list)
{
    uint output = 0;
    for (auto& v : list)
    {
        for(const std::unique_ptr<Dubins>& d : v)
        {
            if (d->is_valid())
            {
                output++;
            }
        }
    }

    return output;
}


// ---------------------------------------- LP Solver ---------------------------------------- //

std::vector<Dubins> find_pathplanning_LP_solution(
    SharedListOfPossibilities& list_of_possibilites,
    const std::vector<AircraftStats>& stats,
    const std::vector<Conflict_T>& conflicts,
    uint max_path_num, int THREADS, bool duration_based_costs, int verbosity)
{

    uint AC_num = stats.size();
    uint var_num = AC_num * max_path_num;

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(list_of_possibilites.size() == stats.size());
#endif
    
    Highs model;
    // -- Setup model

    // - General Options

    model.setOptionValue("threads",THREADS);
    model.setOptionValue("output_flag",true);
    model.setOptionValue("presolve","on");
    model.setOptionValue("parallel","on");
    // model.setOptionValue("log_file","highs.log");
    model.setOptionValue("mip_abs_gap", 1-1e-5); // Since all variables in the objective are binary, it should stop when the absolute gap is below 1
    // model.setOptionValue('random_seed', SEED)

    if (verbosity >= DubinsFleetPlanner_VERYVERY_VERBOSE)
    {
        model.setOptionValue("log_to_console",true);
    }
    else
    {
        model.setOptionValue("log_to_console",false);
    }

    // -- Define variables with bounds (01-LP problem)

    size_t max_section_count = 0;
    for(const auto& list : list_of_possibilites)
    {
        for(const auto& p : list)
        {
            max_section_count = std::max(max_section_count,p->section_count());
        }
    }

    // - Setup costs based on path durations (default to 1 everywhere: all paths are equally good)
    std::vector<double> costs(var_num,1.);
    for(uint ac = 0; ac < AC_num; ac++)
    {
        const auto& list = list_of_possibilites[ac];
        const AircraftStats& stat = stats[ac];

        uint p_index = 0;
        for(const std::shared_ptr<Dubins>& p : list)
        {
            if (duration_based_costs)
            {
                costs[ac*max_path_num+p_index]  = p->get_duration(stat) * max_section_count * 2 * M_PI + p->turning_angle();
            }
            else
            {
                costs[ac*max_path_num+p_index]  = p->turning_angle();
            }
            p_index++;
        }
        while(p_index < max_path_num)
        {
            costs[ac*max_path_num+p_index]  = 1e9;
            p_index++;
        }
    }

    std::vector<double> lowers(var_num);
    std::fill(lowers.begin(),lowers.end(),0.);
    std::vector<double> uppers(var_num);
    std::fill(uppers.begin(),uppers.end(),1.);

    model.addCols(var_num,costs.data(),lowers.data(),uppers.data(),0,nullptr,nullptr,nullptr);

    std::vector<HighsVarType> integrality(var_num);
    std::fill(integrality.begin(),integrality.end(),HighsVarType::kInteger);
    model.changeColsIntegrality(0,var_num-1,integrality.data());
    model.changeObjectiveSense(ObjSense::kMinimize);

    // -- Constraints 

    std::vector<int> indices(max_path_num);


    // Unique path for each AC
    for(uint ac_id = 0; ac_id < AC_num; ac_id++)
    {
        for(uint i = 0; i < max_path_num; i++)
        {
            indices[i] = ac_id*max_path_num+i;
        }

        model.addRow(1.,1.,max_path_num,indices.data(),uppers.data());
    }

    // -- Remove impossible paths by adding constraints

    for(uint ac_id = 0; ac_id < list_of_possibilites.size(); ac_id++)
    {
        auto& path = list_of_possibilites[ac_id];
        if (path.size() < max_path_num)
        {
            std::vector<int> indices;
            std::vector<double> values;
            uint base_id = ac_id*max_path_num;
            for(uint p_i = path.size(); p_i < max_path_num; p_i++)
            {
                indices.push_back(base_id+p_i);
                values.push_back(1.);
            }
            model.addRow(-1,0.,max_path_num-path.size(),indices.data(),values.data());
        }
    }

    // -- Add conflicts

    for(const Conflict_T& c : conflicts)
    {
        uint ac_id1     = std::get<0>(c);
        uint path_id1   = std::get<1>(c);
        uint ac_id2     = std::get<2>(c);
        uint path_id2   = std::get<3>(c);


        uint indices[] = {
            ac_id1*max_path_num + path_id1,
            ac_id2*max_path_num + path_id2
        };

        double values[] = {
            1.,1.
        };

        // std::cout << "Adding conflict between ( " << ac_id1 << " , " << path_id1 << " ) and ( " << ac_id2 << " , " << path_id2 << " )" << std::endl;
        // std::cout << "Code : ( " <<  indices[0] << " , " << indices[1] << " )" << std::endl;

        model.addRow(0,1.,2,(const int*)indices,values);
    }

    // -- Solve

    auto highs_ret = model.run();

    if (highs_ret != HighsStatus::kOk)
    {
        std::cerr   << "ERROR: HiGHS Failure" << std::endl
                    << " Model status     : " << model.modelStatusToString(model.getModelStatus()) << std::endl << std::endl;

        
        return {};
    }


    if (model.getModelStatus() == HighsModelStatus::kOptimal)
    {
        std::vector<Dubins> output;
        uint i = 0;

        auto solutions = model.getSolution();
        for(double val : solutions.col_value)
        {
            if (val > 0.5)
            {
                uint ac_id = i / max_path_num;
                uint path_id = i % max_path_num;
                output.push_back(*list_of_possibilites[ac_id][path_id]);
            }

            i++;
        }
        return output;
    }
    else
    {
        return {};
    }
}
