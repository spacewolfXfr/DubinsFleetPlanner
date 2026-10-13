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
#include <functional>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "../Primitives.hpp"
#include "ScalarMin/IntervalSolver.hpp"
#include "ScalarMin/RoundingPolicies.hpp"


class CircleCircle
{
    typedef OptimizationRounding::fast_rounding<double> Policies;

    private:
        const double Dx,Dy;
        const double r1,r2;
        const double w1,w2;
        const double phi1,phi2;

    public:
        void print_params()
        {
            std::cout   << std::setprecision(5)
                        << "Dx   : " << Dx      << std::endl
                        << "Dy   : " << Dy      << std::endl
                        << "r1   : " << r1      << std::endl
                        << "w1   : " << w1      << std::endl
                        << "r2   : " << r2      << std::endl
                        << "w2   : " << w2      << std::endl
                        << "phi1 : " << phi1    << std::endl
                        << "phi2 : " << phi2    << std::endl; 
            
        }

        CircleCircle(double _Dx, double _Dy, double _r1, double _r2, double _w1, double _w2, double _phi1, double _phi2)
        : Dx(_Dx), Dy(_Dy), r1(_r1), r2(_r2), w1(_w1), w2(_w2), phi1(_phi1), phi2(_phi2) {}

        template<DubinsMove m1, DubinsMove m2> requires ((m1!=STRAIGHT) && (m2!=STRAIGHT))
        CircleCircle(const PathShape<m1> &s, const PathShape<m2> &t)
        :   Dx(s.x - t.x), 
            Dy(s.y - t.y), 
            r1(s.p1), r2(t.p1), 
            w1(s.p2), w2(t.p2), 
            phi1(s.p4), phi2(t.p4) {}

        [[gnu::pure]] double f(double t) const
        {
            double x = Dx + r1*std::cos(w1*t+phi1) - r2*std::cos(w2*t+phi2);
            double y = Dy + r1*std::sin(w1*t+phi1) - r2*std::sin(w2*t+phi2);
            return x*x + y*y;
        } 

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> x_I = Dx + r1*cos(w1*I+phi1) - r2*cos(w2*I+phi2);
            interval<double,Policies> y_I = Dy + r1*sin(w1*I+phi1) - r2*sin(w2*I+phi2);
            return square(x_I) + square(y_I);
        }

        [[gnu::pure]] double f_d(double t) const
        {
            double c1 = std::cos(w1*t+phi1);
            double s1 = std::sin(w1*t+phi1);

            double c2 = std::cos(w2*t+phi2);
            double s2 = std::sin(w2*t+phi2);

            double x = Dx + r1*c1 - r2*c2;
            double y = Dy + r1*s1 - r2*s2;

            double x_d = -w1*r1*s1 + w2*r2*s2;
            double y_d =  w1*r1*c1 - w2*r2*c2;

            return (x_d*x + y_d*y)*2;
        } 

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_d(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> c1 = cos(w1*I+phi1);
            interval<double,Policies> s1 = sin(w1*I+phi1);

            interval<double,Policies> c2 = cos(w2*I+phi2);
            interval<double,Policies> s2 = sin(w2*I+phi2);

            interval<double,Policies> x_I = Dx + r1*c1 - r2*c2;
            interval<double,Policies> y_I = Dy + r1*s1 - r2*s2;

            interval<double,Policies> x_d_I = -w1*r1*s1 + w2*r2*s2;
            interval<double,Policies> y_d_I =  w1*r1*c1 - w2*r2*c2;
            return (x_d_I*x_I + y_d_I*y_I)*static_cast<double>(2);
        }

        [[gnu::pure]] double f_dd(double t) const
        {
            double c1 = std::cos(w1*t+phi1);
            double s1 = std::sin(w1*t+phi1);

            double c2 = std::cos(w2*t+phi2);
            double s2 = std::sin(w2*t+phi2);

            double x = Dx + r1*c1 - r2*c2;
            double y = Dy + r1*s1 - r2*s2;

            double x_d = -w1*r1*s1 + w2*r2*s2;
            double y_d =  w1*r1*c1 - w2*r2*c2;

            double x_dd = -w1*w1*r1*c1 + w2*w2*r2*c2;
            double y_dd = -w1*w1*r1*s1 + w2*w2*r2*s2;

            return (x_d*x_d+x*x_dd + y_d*y_d+y*y_dd)*2;
        } 

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_dd(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> c1 = cos(w1*I+phi1);
            interval<double,Policies> s1 = sin(w1*I+phi1);

            interval<double,Policies> c2 = cos(w2*I+phi2);
            interval<double,Policies> s2 = sin(w2*I+phi2);

            interval<double,Policies> x_I = Dx + r1*c1 - r2*c2;
            interval<double,Policies> y_I = Dy + r1*s1 - r2*s2;

            interval<double,Policies> x_d_I = -w1*r1*s1 + w2*r2*s2;
            interval<double,Policies> y_d_I =  w1*r1*c1 - w2*r2*c2;

            interval<double,Policies> x_dd_I = -w1*w1*r1*c1 + w2*w2*r2*c2;
            interval<double,Policies> y_dd_I = -w1*w1*r1*s1 + w2*w2*r2*s2;
            
            return (square(x_d_I)+x_I*x_dd_I + square(y_d_I)+y_I*y_dd_I)*static_cast<double>(2);
        }

};