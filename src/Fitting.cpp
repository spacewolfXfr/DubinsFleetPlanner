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

#include "Fitting.hpp"

#include <iostream>

using namespace boost::math::tools;

// -------------------- Fit based on curvature -------------------- //

template<typename F>
double generic_fit(F f, double min_val, double max_val, double tol=1e-6)
{

#if DubinsFleetPlanner_ASSERTIONS > 0
        assert(tol > 0);
#endif

    std::uintmax_t inter_count = 1000;

    auto tol_func = eps_tolerance<double>(-std::log2(tol)+1);

    double fa = f(min_val);
    double fb = f(max_val);

    // No root in bracket
    if (boost::math::sign(fa) * boost::math::sign(fb) > 0)
    {
        return NAN;
    }

    std::pair<double,double> bracket = toms748_solve(f,min_val,max_val,fa,fb,tol_func,inter_count);

    bool valid_bracket  = boost::math::sign(f(bracket.first))*boost::math::sign(f(bracket.second)) <= 0;
    bool precise_enough = tol_func(bracket.first,bracket.second);    
    double output;

    if (valid_bracket && precise_enough)
    {

        double o_left = bracket.first;
        double o_right = bracket.second;
        double o_mid = (o_left+o_right)/2.;
        double test_val;

        double best_val = INFINITY;

        test_val = std::abs(f(o_left));
        if (test_val < best_val)
        {
            best_val = test_val;
            output = o_left;
        }

        test_val = std::abs(f(o_right));
        if (test_val < best_val)
        {
            best_val = test_val;
            output = o_right;
        }

        test_val = std::abs(f(o_mid));
        if (test_val < best_val)
        {
            best_val = test_val;
            output = o_mid;
        }

        if (std::abs(f(output)) > tol)
        {
            output = NAN;
        }
    }
    else
    {
        output = NAN;
    }

    return output;
}


double fit_LSL(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = LSL_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,target_l);
    if (rmax < rmin) {return NAN;}
    auto target_fun = [=](double rho){return LSL_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_RSR(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = RSR_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,target_l);
    if (rmax < rmin) {return NAN;}
    auto target_fun = [=](double rho){return RSR_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_RSL(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = RSL_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,target_l);
    if (rmax < rmin) {return NAN;}
    auto target_fun = [=](double rho){return RSL_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_LSR(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = LSR_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,target_l);
    if (rmax < rmin) {return NAN;}
    auto target_fun = [=](double rho){return LSR_total_distance(alpha,beta,d/rho)*rho-target_l;};
    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_RLR(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = RLR_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,target_l);
    if (rmax < rmin) {return NAN;}
    auto target_fun = [=](double rho){return RLR_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_LRL(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = LRL_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,target_l);
    if (rmax < rmin) {return NAN;}
    auto target_fun = [=](double rho){return LRL_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_SRS(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = SRS_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,target_l);
    if (rmax < rmin) {return NAN;}
    auto target_fun = [=](double rho){return SRS_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_SLS(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = SLS_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,target_l);
    if (rmax < rmin) {return NAN;}
    auto target_fun = [=](double rho){return SLS_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}

// -------------------- Fit based on endpoint shift -------------------- //

template<double(distance_fun)(double,double,double)>
double fit_shift_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio)
{

    assert(0 <= ratio && ratio <= 1.);
    double min_t = 0.;
    double max_t = target_l;
    if (min_t > max_t) {return NAN;}
    auto target_fun = [=](double t)
    {
        Pose3D shift_start(start);
        shift_start.x += ratio*t*cos(shift_start.theta);
        shift_start.y += ratio*t*sin(shift_start.theta);

        Pose3D shift_end(end);
        shift_end.x -= (1-ratio)*t*cos(shift_end.theta);
        shift_end.y -= (1-ratio)*t*sin(shift_end.theta);

        double alpha,beta,d;
        auto normalized = normalize_poses(shift_start,shift_end);
        alpha   = std::get<0>(normalized);
        beta    = std::get<1>(normalized);
        d       = std::get<2>(normalized);

        return distance_fun(alpha,beta,d) - (target_l-t);
    };

    return generic_fit(target_fun,min_t,max_t,tol);
}
double fit_shift_LSL_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio)
{
    return fit_shift_ratio<LSL_total_distance>(start,end,target_l,tol,ratio);
}

double fit_shift_RSR_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio)
{
    return fit_shift_ratio<RSR_total_distance>(start,end,target_l,tol,ratio);
}

double fit_shift_RSL_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio)
{
    return fit_shift_ratio<RSL_total_distance>(start,end,target_l,tol,ratio);
}

double fit_shift_LSR_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio)
{
    return fit_shift_ratio<LSR_total_distance>(start,end,target_l,tol,ratio);
}

double fit_shift_RLR_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio)
{
    return fit_shift_ratio<RLR_total_distance>(start,end,target_l,tol,ratio);
}

double fit_shift_LRL_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio)
{
    return fit_shift_ratio<LRL_total_distance>(start,end,target_l,tol,ratio);
}

double fit_shift_SLS_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio)
{
    return fit_shift_ratio<SLS_total_distance>(start,end,target_l,tol,ratio);
}

double fit_shift_SRS_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio)
{
    return fit_shift_ratio<SRS_total_distance>(start,end,target_l,tol,ratio);
}