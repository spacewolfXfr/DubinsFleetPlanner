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

#pragma once

#include <assert.h>
#include <exception>
#include <charconv>
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <tuple>
#include <map>
#include <optional>

#include <nlohmann/json.hpp>

#include "csv_reader.hpp"

#include "ProjectHeader.h"

#include "Aircraft.h"
#include "utils.hpp"
#include "Dubins.hpp"
#include "ConflictList.hpp"
#include "Pose3D.hpp"


namespace DubinsPP
{
    namespace InputParser
    {
        struct PathGenerationOptionOverrides
        {
            std::optional<std::vector<double>> extend_start;
            std::optional<std::vector<double>> extend_end;
            std::optional<bool> straights_only;
            std::optional<bool> line_fit;
            std::optional<std::vector<double>> line_ratios;
            std::optional<bool> fixed_radius;
            std::optional<int> samples;
            std::optional<std::vector<double>> ellipse_ratios;
            std::optional<bool> ellipse_border_only;
        };

        struct PathGenerationConfig
        {
            PathGenerationOptionOverrides defaults;
            std::map<int,PathGenerationOptionOverrides> per_aircraft;
        };

        // Storage for case data has shown in a CSV line
        typedef struct 
        {
            Pose3D start;
            Pose3D end;
            AircraftStats stats;
            double dt;
            std::vector<double> timeslots = {};
        } RowInfo;

        // Transposed type of std::vector<RowInfo>
        typedef std::tuple<std::vector<Pose3D>,std::vector<Pose3D>,std::vector<AircraftStats>,std::vector<double>, std::vector<std::vector<double>>> CaseData;

        /**
         * @brief Parse an input stream reading a CSV formatted for a path planning data into the matching data
         * 
         * The first line (header) should exactly as follow:
         * ac_id,start_x,start_y,start_z,start_theta,end_x,end_y,end_z,end_theta,airspeed,climb,turn_radius,dt
         * 
         * The expected types are:
         * int (positive),float,float,float,float   ,float,float,float,float    ,float (positive),float,float (positive),float
         * 
         * The following data rows should match this format for correct parsing
         * 
         * @return CaseData 
         */
        CaseData parse_data_csv(std::istream&);

        /**
         * @brief Parse per-aircraft path generation configuration from JSON
         *
         * Expected shape:
         * {
         *   "default": { ... option overrides ... },
         *   "aircraft": {
         *     "1": { ... option overrides ... },
         *     "2": { ... option overrides ... }
         *   }
         * }
         *
         * Override keys are:
         * - "extend-start", "extend-end" (array of numbers)
         * - "straights-only", "line", "fixed-radius", "ellipse-border-only" (bool)
         * - "line-ratios" (array of numbers)
         * - "samples" (int)
         * - "ellipse" (number, use NaN to disable)
         */
        PathGenerationConfig parse_path_generation_config_json(std::istream&);

        /**
         * @brief Parse from a stream a Dubins Path planning in a JSON format according to the specs given in `USAGE.md` 
         * 
         * Use nlohmann's JSON C++ library for deserializing
         * 
         * @param s         Input stream
         * @param paths     Solution Dubins paths
         * @param stats     Statistics of the aircraft
         * @param min_sep   Minimal separation requested
         * @param wind_x    Wind, X component
         * @param wind_y    Wind, Y component
         * @param z_alpha   The distortion value for computing vertical separation (currently unused)
         */
        void parse_paths_as_ModernJSON(std::istream& s, 
            std::vector<Dubins>& paths,
            std::vector<AircraftStats>& stats,
            double& min_sep,
            double& wind_x, double& wind_y,
            double& z_alpha);

        /**
         * @brief Parse from a stream an array of path shapes
         * 
         * @param s         Input stream
         * @param shapes    Parsed shapes
         */
        void parse_pathshapes_ModernJSON(std::istream& s, std::vector<DynamicPathShape>& shapes);

    } // namespace InputParser

    namespace OutputPrinter
    {
        /**
         * @brief Print to a stream a Dubins Path planning in a JSON format according to the specs given in `USAGE.md` 
         * 
         * Use nlohmann's JSON C++ library for serializing
         * 
         * @param s         Output stream
         * @param paths     Solution Dubins paths
         * @param stats     Statistics of the aircraft
         * @param min_sep   Minimal separation requested
         * @param wind_x    Wind, X component
         * @param wind_y    Wind, Y component
         * @param z_alpha   The distortion value for computing vertical separation (currently unused)
         */
        void print_paths_as_ModernJSON(std::ostream& s, 
            const std::vector<Dubins>& paths,
            const std::vector<AircraftStats>& stats,
            double min_sep,
            double wind_x, double wind_y,
            double z_alpha=1.);

        /**
         * @brief Print to a stream samples of the Dubins paths and store them in a CSV format according to the specs given in `USAGE.md`
         * 
         * @param s         Output stream
         * @param paths     Solution Dubins paths
         * @param stats     Statistics of the aircraft
         * @param wind_x    Wind, X component
         * @param wind_y    Wind, Y component
         * @param samples   Number of equally spaced samples to take
         */
        void print_paths_as_CSV(std::ostream& s, 
            const std::vector<Dubins>& paths,
            const std::vector<AircraftStats>& stats,
            double wind_x, double wind_y, uint samples);

        /**
         * @brief Print to a stream a collection of geometric obstacles in a JSON format according to the specs given in `USAGE.md`
         * 
         * @param s         Output stream
         * @param obstacles Obstacle collection to print
         */
        void print_obstacles_collection(std::ostream& s, const std::vector<DynamicPathShape>& obstacles);

        /**
         * @brief Format conflicts in JSON and add them to the stream
         * 
         * @param os                Output stream
         * @param time              Reference time for the conflicts (in defined)
         * @param conflicts         List of conflicts to register
         * @param possibilities     Possible paths. Used to convert path id into path abbrev (for portability)
         * @param stats             AC data, for better logging
         * @param prepend_comma     Add a comma before printing to JSON
         */
        void append_rich_conflicts(std::ostream& os,
            double time,
            const std::vector<RichConflict_T>& conflicts,
            const SharedListOfPossibilities& possibilities,
            const std::vector<AircraftStats>& stats,
            bool prepend_comma);
    } // namespace OutputPrinter
    
} // namespace DubinsPP