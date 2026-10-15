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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DubinsFleetPlanner. If not, see <https://www.gnu.org/licenses/>.

#include <gtest/gtest.h>

#include "testTemporalDistanceVSpeed.hpp"

namespace
{
constexpr double min_turn_radius = 0.5;
constexpr double max_turn_radius = 5.;
constexpr double min_speed = 0.1;
constexpr double max_speed = 10.;
constexpr double max_acceleration = 1.;
constexpr size_t sample_count = static_cast<size_t>(TEST_PATH_DURATION / (DubinsFleetPlanner_PRECISION / 10));

AircraftStats random_stats(int seed)
{
    AircraftStats stats = generate_random_aircraft_stats(seed, min_turn_radius, max_turn_radius,
                                           min_speed, max_speed, max_acceleration);
    stats.target_airspeed = stats.airspeed;
    return stats;
}

template<DubinsMove m>
Pose3D position_at_time(const PathShape<m>& shape, const AircraftStats& stats,
                       double time, double duration)
{
    PathShape<m> unit_speed_shape = shape;
    path_set_planar_speed(unit_speed_shape, 1.);
    return follow_dubins(unit_speed_shape, length_traveled(stats, time, duration));
}

template<bool use_derivatives, DubinsMove m1, DubinsMove m2>
void check_solver_pair(const PathShape<m1>& first, const PathShape<m2>& second,
                      const AircraftStats& first_stats, const AircraftStats& second_stats,
                      const char* label)
{
    const auto result = temporal_XY_dist<m1, m2, use_derivatives>(
        first, second, first_stats, second_stats, TEST_PATH_DURATION, TEST_PRECISION);
    const Pose3D first_position = position_at_time(first, first_stats, result.first, TEST_PATH_DURATION);
    const Pose3D second_position = position_at_time(second, second_stats, result.first, TEST_PATH_DURATION);

    EXPECT_NEAR(pose_dist_XY(first_position, second_position), result.second,
                DubinsFleetPlanner_PRECISION)
        << label << " value error (solver vs direct evaluation)";
}

template<bool use_derivatives, DubinsMove m1, DubinsMove m2>
void check_sampled_pair(const PathShape<m1>& first, const PathShape<m2>& second,
                        const AircraftStats& first_stats, const AircraftStats& second_stats,
                        const char* label)
{
    const auto sampled = sample_temporal_XY_dist<m1, m2, sample_count>(
        first, second, first_stats, second_stats, TEST_PATH_DURATION);
    const auto solved = temporal_XY_dist<m1, m2, use_derivatives>(
        first, second, first_stats, second_stats, TEST_PATH_DURATION, TEST_PRECISION);

    EXPECT_NEAR(sampled.first, solved.first, DubinsFleetPlanner_PRECISION)
        << label << " location error (sampled vs solver)";
    EXPECT_NEAR(sampled.second, solved.second, DubinsFleetPlanner_PRECISION)
        << label << " value error (sampled vs solver)";
}

template<bool use_derivatives>
void check_sampled_pair(const DynamicPathShape& first, const DynamicPathShape& second,
                        const AircraftStats& first_stats, const AircraftStats& second_stats,
                        const char* label)
{
    const auto sampled = sample_temporal_XY_dist<sample_count>(
        first, second, first_stats, second_stats, TEST_PATH_DURATION);
    const auto solved = temporal_XY_dist<use_derivatives>(
        first, second, first_stats, second_stats, TEST_PATH_DURATION, TEST_PRECISION);

    EXPECT_NEAR(sampled.first, solved.first, DubinsFleetPlanner_PRECISION)
        << label << " location error (sampled vs solver)";
    EXPECT_NEAR(sampled.second, solved.second, DubinsFleetPlanner_PRECISION)
        << label << " value error (sampled vs solver)";
}

template<bool use_derivatives>
void run_static_solver_tests()
{
    for (int i = 0; i < TEST_NUM; ++i)
    {
        const PathShape<STRAIGHT> first_straight = generate_random_shape<STRAIGHT>(4 * i, TEST_POS_RANGE, TEST_SPEED_RANGE);
        const PathShape<STRAIGHT> second_straight = generate_random_shape<STRAIGHT>(4 * i + 1, TEST_POS_RANGE, TEST_SPEED_RANGE);
        const PathShape<LEFT> left = generate_random_shape<LEFT>(4 * i + 2, TEST_POS_RANGE, TEST_SPEED_RANGE);
        const PathShape<RIGHT> right = generate_random_shape<RIGHT>(4 * i + 3, TEST_POS_RANGE, TEST_SPEED_RANGE);

        const AircraftStats first_stats = random_stats(4 * i);
        const AircraftStats second_stats = random_stats(4 * i + 1);
        const AircraftStats left_stats = random_stats(4 * i + 2);
        const AircraftStats right_stats = random_stats(4 * i + 3);

        check_solver_pair<use_derivatives>(first_straight, second_straight, first_stats, second_stats, "STRAIGHT-STRAIGHT");
        check_solver_pair<use_derivatives>(first_straight, left, first_stats, left_stats, "STRAIGHT-LEFT");
        check_solver_pair<use_derivatives>(first_straight, right, first_stats, right_stats, "STRAIGHT-RIGHT");
        check_solver_pair<use_derivatives>(left, right, left_stats, right_stats, "LEFT-RIGHT");
    }
}

template<bool use_derivatives>
void run_static_sample_tests()
{
    for (int i = 0; i < TEST_NUM; ++i)
    {
        const PathShape<STRAIGHT> first_straight = generate_random_shape<STRAIGHT>(4 * i, TEST_POS_RANGE, TEST_SPEED_RANGE);
        const PathShape<STRAIGHT> second_straight = generate_random_shape<STRAIGHT>(4 * i + 1, TEST_POS_RANGE, TEST_SPEED_RANGE);
        const PathShape<LEFT> left = generate_random_shape<LEFT>(4 * i + 2, TEST_POS_RANGE, TEST_SPEED_RANGE);
        const PathShape<RIGHT> right = generate_random_shape<RIGHT>(4 * i + 3, TEST_POS_RANGE, TEST_SPEED_RANGE);

        const AircraftStats first_stats = random_stats(4 * i);
        const AircraftStats second_stats = random_stats(4 * i + 1);
        const AircraftStats left_stats = random_stats(4 * i + 2);
        const AircraftStats right_stats = random_stats(4 * i + 3);

        check_sampled_pair<use_derivatives>(first_straight, second_straight, first_stats, second_stats, "STRAIGHT-STRAIGHT");
        check_sampled_pair<use_derivatives>(first_straight, left, first_stats, left_stats, "STRAIGHT-LEFT");
        check_sampled_pair<use_derivatives>(first_straight, right, first_stats, right_stats, "STRAIGHT-RIGHT");
        check_sampled_pair<use_derivatives>(left, right, left_stats, right_stats, "LEFT-RIGHT");
    }
}

template<bool use_derivatives>
void run_dynamic_sample_tests()
{
    for (int i = 0; i < TEST_NUM; ++i)
    {
        const DynamicPathShape first_straight = generate_random_dynamic_shape(STRAIGHT, 4 * i, TEST_POS_RANGE, TEST_SPEED_RANGE);
        const DynamicPathShape second_straight = generate_random_dynamic_shape(STRAIGHT, 4 * i + 1, TEST_POS_RANGE, TEST_SPEED_RANGE);
        const DynamicPathShape left = generate_random_dynamic_shape(LEFT, 4 * i + 2, TEST_POS_RANGE, TEST_SPEED_RANGE);
        const DynamicPathShape right = generate_random_dynamic_shape(RIGHT, 4 * i + 3, TEST_POS_RANGE, TEST_SPEED_RANGE);

        const AircraftStats first_stats = random_stats(4 * i);
        const AircraftStats second_stats = random_stats(4 * i + 1);
        const AircraftStats left_stats = random_stats(4 * i + 2);
        const AircraftStats right_stats = random_stats(4 * i + 3);

        check_sampled_pair<use_derivatives>(first_straight, second_straight, first_stats, second_stats, "STRAIGHT-STRAIGHT");
        check_sampled_pair<use_derivatives>(first_straight, left, first_stats, left_stats, "STRAIGHT-LEFT");
        check_sampled_pair<use_derivatives>(first_straight, right, first_stats, right_stats, "STRAIGHT-RIGHT");
        check_sampled_pair<use_derivatives>(left, right, left_stats, right_stats, "LEFT-RIGHT");
    }
}
} // namespace

TEST(TemporalDistanceVSpeed, SolverDistanceDoubleCheckNoDerivatives)
{
    run_static_solver_tests<false>();
}

TEST(TemporalDistanceVSpeed, SolverDistanceDoubleCheckWithDerivatives)
{
    run_static_solver_tests<true>();
}

TEST(TemporalDistanceVSpeed, Random2DCasesNoDerivatives)
{
    run_static_sample_tests<false>();
}

TEST(TemporalDistanceVSpeed, Random2DCasesWithDerivatives)
{
    run_static_sample_tests<true>();
}

TEST(TemporalDistanceVSpeed, Random2DDynamicCasesNoDerivatives)
{
    run_dynamic_sample_tests<false>();
}

TEST(TemporalDistanceVSpeed, Random2DDynamicCasesWithDerivatives)
{
    run_dynamic_sample_tests<true>();
}
