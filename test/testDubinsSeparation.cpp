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

#include <gtest/gtest.h>
#include "testDubinsSeparation.hpp"

template<size_t samples>
void sample_test_dubins_separation(const Dubins& d1, const Dubins& d2, 
    const AircraftStats& v1, const AircraftStats& v2, 
    double duration, double separation, double tol,
    uint rec,
    std::string expect_msg = "",
    bool no_test=false)
{
    bool test_separation    = d1.is_XY_separated_from(d2,v1,v2,duration,separation,tol,std::nullopt,rec);
    std::pair<double,double> computed = d1.XY_distance_to(d2,v1,v2,duration,separation,tol,std::nullopt,rec);

    EXPECT_EQ(test_separation, computed.second > separation) << expect_msg << " Sep and dist methods disagree";

    if (!no_test)
    {
        std::pair<double,double> sampled = sample_temporal_XY_dist<samples>(d1,d2,v1,v2,duration);
        bool check_separation   = sampled.second > separation;

        ASSERT_TRUE((test_separation == check_separation) || (!test_separation && check_separation)) << expect_msg;
        EXPECT_EQ(test_separation,check_separation) << expect_msg << std::endl 
            << "Sampled : " << sampled.second << " at " << sampled.first << std::endl
            << "Computed: " << computed.second << " at " << computed.first << std::endl;

        d1.is_XY_separated_from(d2,v1,v2,duration,separation,tol,std::nullopt,rec);
    }
}

template<size_t samples>
void check_all_dubins_pairs(
    const std::vector<std::unique_ptr<Dubins>>& ds1, const AircraftStats& p1_stats,
    const std::vector<std::unique_ptr<Dubins>>& ds2, const AircraftStats& p2_stats,
    double min_sep, double tol,
    uint rec,
    std::string expect_msg = "",
    bool no_test=false)
{
    double p1_speed = p1_stats.airspeed;
    double p2_speed = p2_stats.airspeed;
    
    for(size_t i1 = 0; i1 < ds1.size(); i1++)
    {
        auto& d1 = ds1[i1];
        for(size_t i2 = 0; i2 < ds2.size(); i2++)
        {
            auto& d2 = ds2[i2];
            std::string msg = expect_msg + " using 1: " + d1->get_type_abbr() + " and 2: " + d2->get_type_abbr();
            double duration = std::min(d1->get_duration(p1_stats)/p1_speed,d2->get_duration(p2_stats)/p2_speed);
            sample_test_dubins_separation<TEST_SAMPLES_COUNT>(*d1,*d2,p1_stats,p2_stats,duration,min_sep,tol,rec,msg,no_test);
        }
    }
}

template<size_t samples>
bool fit_and_test_dubins(
    const Pose3D& p1_start, const Pose3D& p1_end, const AircraftStats& p1_stats, double p1_len,
    const Pose3D& p2_start, const Pose3D& p2_end, const AircraftStats& p2_stats, double p2_len,
    double min_sep, double tol, uint rec,
    std::string expect_msg = "",
    bool no_test=false)
{
    std::vector<std::unique_ptr<Dubins>> p1_possibilities = fit_possible_baseDubins(p1_stats,
            p1_start, p1_end, p1_len, tol);

    std::vector<std::unique_ptr<Dubins>> p2_possibilities = fit_possible_baseDubins(p2_stats,
            p2_start, p2_end, p2_len, tol);

    if (p1_possibilities.size() == 0)
    {
        std::cout << "No fitted path found for path 1. Skipping..." << std::endl;
        return false;
    }

    if (p2_possibilities.size() == 0)
    {
        std::cout << "No fitted path found for path 2. Skipping..." << std::endl;
        return false;
    }

    check_all_dubins_pairs<samples>(p1_possibilities,p1_stats,p2_possibilities,p2_stats,min_sep,tol,rec,expect_msg,no_test);

    return true;

}

[[gnu::pure]]
static double longest_path(const std::vector<std::unique_ptr<Dubins>>& dubins_list)
{
    double max_len = 0;
    for(auto& d : dubins_list)
    {
        double len = d->get_length();
        max_len = std::max(max_len,len);
    }
    return max_len;
}

static void generate_and_test_dubins_sep(AircraftStats a1, AircraftStats a2, double min_sep, int seed, uint rec, bool fitting, bool no_test=false)
{
    // Setup RNG
    std::default_random_engine gen(seed);
    std::uniform_real_distribution<double> dis_speedn(TEST_SPEED_RANGE/10, TEST_SPEED_RANGE);
    std::uniform_int_distribution dis_ints(INT32_MIN,INT32_MAX);

    // Generate endpoints
    Pose3D p1_start = generate_pose_2D(dis_ints(gen), TEST_POS_RANGE);
    Pose3D p1_end   = generate_pose_2D(dis_ints(gen), TEST_POS_RANGE);

    Pose3D p2_start = generate_pose_2D(dis_ints(gen), TEST_POS_RANGE);
    Pose3D p2_end   = generate_pose_2D(dis_ints(gen), TEST_POS_RANGE);

    // Generate random speeds
    
    double p1_speed = dis_speedn(gen);
    double p2_speed = dis_speedn(gen);

    a1.airspeed = p1_speed;
    a2.airspeed = p2_speed;

    // Draw paths before fitting
    std::vector<std::unique_ptr<Dubins>> p1_possibilities = list_possible_baseDubins(1.,TEST_MIN_TURN_RADIUS,p1_start,p1_end);
    std::vector<std::unique_ptr<Dubins>> p2_possibilities = list_possible_baseDubins(1.,TEST_MIN_TURN_RADIUS,p2_start,p2_end);

    if (p1_possibilities.size() == 0)
    {
        std::cout << "Case n° " << seed << "  : No path found for path 1. Skipping..." << std::endl;
        return;
    }

    if (p2_possibilities.size() == 0)
    {
        std::cout << "Case n° " << seed << "  : No path found for path 2. Skipping..." << std::endl;
        return;
    }

    if (fitting)
    {
        // Use longest path for fitting
        double max_duration = std::max(
            longest_path(p1_possibilities)/p1_speed,
            longest_path(p2_possibilities)/p2_speed
        );

        fit_and_test_dubins<TEST_SAMPLES_COUNT>(
            p1_start,p1_end,a1,max_duration/p1_speed,
            p2_start,p2_end,a2,max_duration/p2_speed,
            min_sep,DubinsFleetPlanner_PRECISION,
            rec,
            "Failure at case n° " + std::to_string(seed),
            no_test);
    }
    else
    {
        check_all_dubins_pairs<TEST_SAMPLES_COUNT>(
            p1_possibilities,a1,
            p2_possibilities,a2,
            min_sep,DubinsFleetPlanner_PRECISION,
            rec,
            "Failure at case n° " + std::to_string(seed),
            no_test);
    }
    
}

// ******************** Actual testing ******************** //

class DubinsSeparation : public testing::TestWithParam<uint>
{
protected:
    AircraftStats a1{1,1.,1.,TEST_MIN_TURN_RADIUS,TEST_CLIMB,-5.,5.};
    AircraftStats a2{2,1.,1.,TEST_MIN_TURN_RADIUS,TEST_CLIMB,-5.,5.};

    double min_separation = TEST_MIN_SEPARATION;

    int test_num = TEST_NUM;

public:
    DubinsSeparation() {}

    void test_random_nofit()
    {
        for(int i = 0; i < test_num; i++)
        {
            generate_and_test_dubins_sep(a1,a2,min_separation,i,this->GetParam(),false);
        }
    }

    void test_random_fit()
    {
        for(int i = 0; i < test_num; i++)
        {
            generate_and_test_dubins_sep(a1,a2,min_separation,i,this->GetParam(),true);
        }
    }

    void run_for_speed()
    {
        for(int i = 0; i < test_num; i++)
        {
            generate_and_test_dubins_sep(a1,a2,min_separation,i,this->GetParam(),false,true);
        }
    }
};


// TEST_P(DubinsSeparation,NoFit)
// {
//     this->test_random_nofit();
// }

// TEST_P(DubinsSeparation,WithFit)
// {
//     this->test_random_fit();
// }

TEST_P(DubinsSeparation,SpeedTest)
{
    this->run_for_speed();
}

INSTANTIATE_TEST_SUITE_P(
    DubinsSeparationUsingDifferentRec,
    DubinsSeparation,
    testing::Values(0,2,4,6,8)
);


TEST(DubinsSeparation,SelectedCase1)
{
    Pose3D p1_start = Pose3D(
        -1.9364916731037103,
        -3.3541019662496838,
        0.,
        5.7595865315812871
    );

    Pose3D p2_start = Pose3D(
        2.5915316948655271,
        -2.8781875328941666,
        0.,
        7.0162235930172043
    );

    Pose3D p1_end = Pose3D(
        7.5,
        34,
        0.,
        M_PI_2
    );

    Pose3D p2_end = Pose3D(
        12,
        31,
        0.,
        M_PI_2
    );

    double p1_speed = 1.;
    double p2_speed = 1.;

    double p1_turn_radius = 1.;
    double p2_turn_radius = 1.;

    double p1_climb = 1.;
    double p2_climb = 1.;

    double min_separation = 1.;
    double target_length_1 = 59.302868877849335;
    double target_length_2 = 59.302868877849335;

    AircraftStats a1{1,p1_speed,p1_speed,p1_turn_radius,p1_climb,0.,0.};
    AircraftStats a2{2,p2_speed,p2_speed,p2_turn_radius,p2_climb,0.,0.};


    fit_and_test_dubins<TEST_SAMPLES_COUNT>(
            p1_start,p1_end,a1,target_length_1,
            p2_start,p2_end,a2,target_length_2,
            min_separation,DubinsFleetPlanner_PRECISION,
            0,
            "Failure");

}