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
#include "testTemporalDistance.hpp"

TEST(TemporalDistance,SolverDistanceDoubleCheckNoDerivatives)
{
    for(int i = 0; i < TEST_NUM; i++)
    {
        PathShape<STRAIGHT> s   = generate_random_shape<STRAIGHT> (4*i  , TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<STRAIGHT> sbis= generate_random_shape<STRAIGHT> (4*i+1, TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<LEFT>     l   = generate_random_shape<LEFT>     (4*i+2, TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<RIGHT>    r   = generate_random_shape<RIGHT>    (4*i+3, TEST_POS_RANGE, TEST_SPEED_RANGE);
        std::cout << "Random shapes generation n°" << i << " done" << std::endl;

        SpeedProfile s_prof   {path_planar_speed(s)   , 0.};
        SpeedProfile sbis_prof{path_planar_speed(sbis), 0.};
        SpeedProfile l_prof   {path_planar_speed(l)   , 0.};
        SpeedProfile r_prof   {path_planar_speed(r)   , 0.};

        std::pair<double,double> test_ss_no_der = temporal_XY_dist<STRAIGHT,STRAIGHT,false>  (s,sbis,s_prof,sbis_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_sl_no_der = temporal_XY_dist<STRAIGHT,LEFT,false>  (s,l,s_prof,l_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_sr_no_der = temporal_XY_dist<STRAIGHT,RIGHT,false> (s,r,s_prof,r_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_lr_no_der = temporal_XY_dist<LEFT,RIGHT,false>     (l,r,l_prof,r_prof,TEST_PATH_DURATION,TEST_PRECISION);

        Pose3D ss_p1 = follow_dubins(s      ,test_ss_no_der.first);
        Pose3D ss_p2 = follow_dubins(sbis   ,test_ss_no_der.first);
        
        Pose3D sl_p1 = follow_dubins(s,test_sl_no_der.first);
        Pose3D sl_p2 = follow_dubins(l,test_sl_no_der.first);

        Pose3D sr_p1 = follow_dubins(s,test_sr_no_der.first);
        Pose3D sr_p2 = follow_dubins(r,test_sr_no_der.first);

        Pose3D lr_p1 = follow_dubins(l,test_lr_no_der.first);
        Pose3D lr_p2 = follow_dubins(r,test_lr_no_der.first);

        EXPECT_NEAR(pose_dist_XY(ss_p1,ss_p2),test_ss_no_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-STRAIGHT val error (sampled vs no derivative)";
        EXPECT_NEAR(pose_dist_XY(sl_p1,sl_p2),test_sl_no_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT val error (sampled vs no derivative)";
        EXPECT_NEAR(pose_dist_XY(sr_p1,sr_p2),test_sr_no_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT val error (sampled vs no derivative)";
        EXPECT_NEAR(pose_dist_XY(lr_p1,lr_p2),test_lr_no_der.second,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT val error (sampled vs no derivative)";
    }
}

TEST(TemporalDistance,SolverDistanceDoubleCheckWithDerivatives)
{
    for(int i = 0; i < TEST_NUM; i++)
    {
        PathShape<STRAIGHT> s   = generate_random_shape<STRAIGHT> (4*i  , TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<STRAIGHT> sbis= generate_random_shape<STRAIGHT> (4*i+1, TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<LEFT>     l   = generate_random_shape<LEFT>     (4*i+2, TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<RIGHT>    r   = generate_random_shape<RIGHT>    (4*i+3, TEST_POS_RANGE, TEST_SPEED_RANGE);
        std::cout << "Random shapes generation n°" << i << " done" << std::endl;

        SpeedProfile s_prof   {path_planar_speed(s)   , 0.};
        SpeedProfile sbis_prof{path_planar_speed(sbis), 0.};
        SpeedProfile l_prof   {path_planar_speed(l)   , 0.};
        SpeedProfile r_prof   {path_planar_speed(r)   , 0.};

        std::pair<double,double> test_ss_with_der = temporal_XY_dist<STRAIGHT,STRAIGHT,true>  (s,sbis,s_prof,sbis_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_sl_with_der = temporal_XY_dist<STRAIGHT,LEFT,true>  (s,l,s_prof,l_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_sr_with_der = temporal_XY_dist<STRAIGHT,RIGHT,true> (s,r,s_prof,r_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_lr_with_der = temporal_XY_dist<LEFT,RIGHT,true>     (l,r,l_prof,r_prof,TEST_PATH_DURATION,TEST_PRECISION);

        Pose3D ss_p1 = follow_dubins(s      ,test_ss_with_der.first);
        Pose3D ss_p2 = follow_dubins(sbis   ,test_ss_with_der.first);
        
        Pose3D sl_p1 = follow_dubins(s,test_sl_with_der.first);
        Pose3D sl_p2 = follow_dubins(l,test_sl_with_der.first);

        Pose3D sr_p1 = follow_dubins(s,test_sr_with_der.first);
        Pose3D sr_p2 = follow_dubins(r,test_sr_with_der.first);

        Pose3D lr_p1 = follow_dubins(l,test_lr_with_der.first);
        Pose3D lr_p2 = follow_dubins(r,test_lr_with_der.first);

        EXPECT_NEAR(pose_dist_XY(ss_p1,ss_p2),test_ss_with_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-STRAIGHT val error (sampled vs with derivative)";
        EXPECT_NEAR(pose_dist_XY(sl_p1,sl_p2),test_sl_with_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT val error (sampled vs with derivative)";
        EXPECT_NEAR(pose_dist_XY(sr_p1,sr_p2),test_sr_with_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT val error (sampled vs with derivative)";
        EXPECT_NEAR(pose_dist_XY(lr_p1,lr_p2),test_lr_with_der.second,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT val error (sampled vs with derivative)";
    }
}

TEST(TemporalDistance,Random2DCasesNoDerivatives)
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));

    for(int i = 0; i < TEST_NUM; i++)
    {
        PathShape<STRAIGHT> s   = generate_random_shape<STRAIGHT> (4*i  , TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<STRAIGHT> sbis= generate_random_shape<STRAIGHT> (4*i+1, TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<LEFT>     l   = generate_random_shape<LEFT>     (4*i+2, TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<RIGHT>    r   = generate_random_shape<RIGHT>    (4*i+3, TEST_POS_RANGE, TEST_SPEED_RANGE);
        std::cout << "Random shapes generation n°" << i << " done" << std::endl;

        SpeedProfile s_prof   {path_planar_speed(s)   , 0.};
        SpeedProfile sbis_prof{path_planar_speed(sbis), 0.};
        SpeedProfile l_prof   {path_planar_speed(l)   , 0.};
        SpeedProfile r_prof   {path_planar_speed(r)   , 0.};

        std::pair<double,double> sampled_ss = sample_temporal_XY_dist<STRAIGHT,STRAIGHT,samples>  (s,sbis,s_prof,sbis_prof,TEST_PATH_DURATION);
        std::cout << "Sampling SS done" << std::endl;
        std::pair<double,double> sampled_sl = sample_temporal_XY_dist<STRAIGHT,LEFT,samples>  (s,l,s_prof,l_prof,TEST_PATH_DURATION);
        std::cout << "Sampling SL done" << std::endl;
        std::pair<double,double> sampled_sr = sample_temporal_XY_dist<STRAIGHT,RIGHT,samples> (s,r,s_prof,r_prof,TEST_PATH_DURATION);
        std::cout << "Sampling SR done" << std::endl;
        std::pair<double,double> sampled_lr = sample_temporal_XY_dist<LEFT,RIGHT,samples>     (l,r,l_prof,r_prof,TEST_PATH_DURATION);
        std::cout << "Sampling LR done" << std::endl;

        std::pair<double,double> test_ss_no_der = temporal_XY_dist<STRAIGHT,STRAIGHT,false>  (s,sbis,s_prof,sbis_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_sl_no_der = temporal_XY_dist<STRAIGHT,LEFT,false>  (s,l,s_prof,l_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_sr_no_der = temporal_XY_dist<STRAIGHT,RIGHT,false> (s,r,s_prof,r_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_lr_no_der = temporal_XY_dist<LEFT,RIGHT,false>     (l,r,l_prof,r_prof,TEST_PATH_DURATION,TEST_PRECISION);

        EXPECT_NEAR(sampled_ss.first,test_ss_no_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-STRAIGHT loc error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_sl.first,test_sl_no_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT loc error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_sr.first,test_sr_no_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT loc error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_lr.first,test_lr_no_der.first,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT loc error (sampled vs no derivative)";

        EXPECT_NEAR(sampled_ss.second,test_ss_no_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-STRAIGHT val error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_sl.second,test_sl_no_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT val error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_sr.second,test_sr_no_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT val error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_lr.second,test_lr_no_der.second,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT val error (sampled vs no derivative)";
    }
}

TEST(TemporalDistance,Random2DCasesWithDerivatives)
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));

    for(int i = 0; i < TEST_NUM; i++)
    {
        PathShape<STRAIGHT> s   = generate_random_shape<STRAIGHT> (4*i  , TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<STRAIGHT> sbis= generate_random_shape<STRAIGHT> (4*i+1, TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<LEFT>     l   = generate_random_shape<LEFT>     (4*i+2, TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<RIGHT>    r   = generate_random_shape<RIGHT>    (4*i+3, TEST_POS_RANGE, TEST_SPEED_RANGE);
        std::cout << "Random shapes generation n°" << i << " done" << std::endl;

        SpeedProfile s_prof   {path_planar_speed(s)   , 0.};
        SpeedProfile sbis_prof{path_planar_speed(sbis), 0.};
        SpeedProfile l_prof   {path_planar_speed(l)   , 0.};
        SpeedProfile r_prof   {path_planar_speed(r)   , 0.};

        std::pair<double,double> sampled_ss = sample_temporal_XY_dist<STRAIGHT,STRAIGHT,samples>  (s,sbis,s_prof,sbis_prof,TEST_PATH_DURATION);
        std::cout << "Sampling SS done" << std::endl;
        std::pair<double,double> sampled_sl = sample_temporal_XY_dist<STRAIGHT,LEFT,samples>  (s,l,s_prof,l_prof,TEST_PATH_DURATION);
        std::cout << "Sampling SL done" << std::endl;
        std::pair<double,double> sampled_sr = sample_temporal_XY_dist<STRAIGHT,RIGHT,samples> (s,r,s_prof,r_prof,TEST_PATH_DURATION);
        std::cout << "Sampling SR done" << std::endl;
        std::pair<double,double> sampled_lr = sample_temporal_XY_dist<LEFT,RIGHT,samples>     (l,r,l_prof,r_prof,TEST_PATH_DURATION);
        std::cout << "Sampling LR done" << std::endl;

        std::pair<double,double> test_ss_with_der = temporal_XY_dist<STRAIGHT,STRAIGHT,true>  (s,sbis,s_prof,sbis_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_sl_with_der = temporal_XY_dist<STRAIGHT,LEFT,true>  (s,l,s_prof,l_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_sr_with_der = temporal_XY_dist<STRAIGHT,RIGHT,true> (s,r,s_prof,r_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_lr_with_der = temporal_XY_dist<LEFT,RIGHT,true>     (l,r,l_prof,r_prof,TEST_PATH_DURATION,TEST_PRECISION);

        EXPECT_NEAR(sampled_ss.first,test_ss_with_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-STRAIGHT loc error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_sl.first,test_sl_with_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT loc error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_sr.first,test_sr_with_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT loc error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_lr.first,test_lr_with_der.first,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT loc error (sampled vs with derivative)";

        EXPECT_NEAR(sampled_ss.second,test_ss_with_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-STRAIGHT val error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_sl.second,test_sl_with_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT val error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_sr.second,test_sr_with_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT val error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_lr.second,test_lr_with_der.second,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT val error (sampled vs with derivative)";
    }
}

TEST(TemporalDistance,Random2DDynamicCasesNoDerivatives)
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));

    for(int i = 0; i < TEST_NUM; i++)
    {
        DynamicPathShape s    = generate_random_dynamic_shape(STRAIGHT, 4*i  , TEST_POS_RANGE, TEST_SPEED_RANGE);
        DynamicPathShape sbis = generate_random_dynamic_shape(STRAIGHT, 4*i+1, TEST_POS_RANGE, TEST_SPEED_RANGE);
        DynamicPathShape l    = generate_random_dynamic_shape(LEFT,     4*i+2, TEST_POS_RANGE, TEST_SPEED_RANGE);
        DynamicPathShape r    = generate_random_dynamic_shape(RIGHT,    4*i+3, TEST_POS_RANGE, TEST_SPEED_RANGE);
        std::cout << "Random shapes generation n°" << i << " done" << std::endl;

        SpeedProfile s_prof   {path_planar_speed(s)   , 0.};
        SpeedProfile sbis_prof{path_planar_speed(sbis), 0.};
        SpeedProfile l_prof   {path_planar_speed(l)   , 0.};
        SpeedProfile r_prof   {path_planar_speed(r)   , 0.};

        std::pair<double,double> sampled_ss = sample_temporal_XY_dist<samples>  (s,sbis,s_prof,sbis_prof,TEST_PATH_DURATION);
        std::cout << "Sampling SS done" << std::endl;
        std::pair<double,double> sampled_sl = sample_temporal_XY_dist<samples>  (s,l,s_prof,l_prof,TEST_PATH_DURATION);
        std::cout << "Sampling SL done" << std::endl;
        std::pair<double,double> sampled_sr = sample_temporal_XY_dist<samples>  (s,r,s_prof,r_prof,TEST_PATH_DURATION);
        std::cout << "Sampling SR done" << std::endl;
        std::pair<double,double> sampled_lr = sample_temporal_XY_dist<samples>  (l,r,l_prof,r_prof,TEST_PATH_DURATION);
        std::cout << "Sampling LR done" << std::endl;

        std::pair<double,double> test_ss_no_der = temporal_XY_dist<false>  (s,sbis,s_prof,sbis_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_sl_no_der = temporal_XY_dist<false>  (s,l,s_prof,l_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_sr_no_der = temporal_XY_dist<false>  (s,r,s_prof,r_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_lr_no_der = temporal_XY_dist<false>  (l,r,l_prof,r_prof,TEST_PATH_DURATION,TEST_PRECISION);

        EXPECT_NEAR(sampled_ss.first,test_ss_no_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-STRAIGHT loc error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_sl.first,test_sl_no_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT loc error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_sr.first,test_sr_no_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT loc error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_lr.first,test_lr_no_der.first,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT loc error (sampled vs no derivative)";

        EXPECT_NEAR(sampled_ss.second,test_ss_no_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-STRAIGHT val error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_sl.second,test_sl_no_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT val error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_sr.second,test_sr_no_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT val error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_lr.second,test_lr_no_der.second,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT val error (sampled vs no derivative)";
    }
}

TEST(TemporalDistance,Random2DDynamicCasesWithDerivatives)
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));

    for(int i = 0; i < TEST_NUM; i++)
    {
        DynamicPathShape s    = generate_random_dynamic_shape(STRAIGHT, 4*i  , TEST_POS_RANGE, TEST_SPEED_RANGE);
        DynamicPathShape sbis = generate_random_dynamic_shape(STRAIGHT, 4*i+1, TEST_POS_RANGE, TEST_SPEED_RANGE);
        DynamicPathShape l    = generate_random_dynamic_shape(LEFT,     4*i+2, TEST_POS_RANGE, TEST_SPEED_RANGE);
        DynamicPathShape r    = generate_random_dynamic_shape(RIGHT,    4*i+3, TEST_POS_RANGE, TEST_SPEED_RANGE);
        std::cout << "Random shapes generation n°" << i << " done" << std::endl;

        SpeedProfile s_prof   {path_planar_speed(s)   , 0.};
        SpeedProfile sbis_prof{path_planar_speed(sbis), 0.};
        SpeedProfile l_prof   {path_planar_speed(l)   , 0.};
        SpeedProfile r_prof   {path_planar_speed(r)   , 0.};

        std::pair<double,double> sampled_ss = sample_temporal_XY_dist<samples>  (s,sbis,s_prof,sbis_prof,TEST_PATH_DURATION);
        std::cout << "Sampling SS done" << std::endl;
        std::pair<double,double> sampled_sl = sample_temporal_XY_dist<samples>  (s,l,s_prof,l_prof,TEST_PATH_DURATION);
        std::cout << "Sampling SL done" << std::endl;
        std::pair<double,double> sampled_sr = sample_temporal_XY_dist<samples>  (s,r,s_prof,r_prof,TEST_PATH_DURATION);
        std::cout << "Sampling SR done" << std::endl;
        std::pair<double,double> sampled_lr = sample_temporal_XY_dist<samples>  (l,r,l_prof,r_prof,TEST_PATH_DURATION);
        std::cout << "Sampling LR done" << std::endl;

        std::pair<double,double> test_ss_with_der = temporal_XY_dist<true>  (s,sbis,s_prof,sbis_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_sl_with_der = temporal_XY_dist<true>  (s,l,s_prof,l_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_sr_with_der = temporal_XY_dist<true>  (s,r,s_prof,r_prof,TEST_PATH_DURATION,TEST_PRECISION);
        std::pair<double,double> test_lr_with_der = temporal_XY_dist<true>  (l,r,l_prof,r_prof,TEST_PATH_DURATION,TEST_PRECISION);

        EXPECT_NEAR(sampled_ss.first,test_ss_with_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-STRAIGHT loc error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_sl.first,test_sl_with_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT loc error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_sr.first,test_sr_with_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT loc error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_lr.first,test_lr_with_der.first,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT loc error (sampled vs with derivative)";

        EXPECT_NEAR(sampled_ss.second,test_ss_with_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-STRAIGHT val error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_sl.second,test_sl_with_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT val error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_sr.second,test_sr_with_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT val error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_lr.second,test_lr_with_der.second,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT val error (sampled vs with derivative)";
    }
}