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

class LineCircle
{

    typedef OptimizationRounding::fast_rounding<double> Policies;

    private:
        const double Dx,Dy;
        const double r,w,phi;
        const double vx,vy;

    public:
        void print_params()
        {
            std::cout   << std::setprecision(5)
                        << "Dx   : " << Dx      << std::endl
                        << "Dy   : " << Dy      << std::endl
                        << "r    : " << r       << std::endl
                        << "w    : " << w       << std::endl
                        << "phi  : " << phi     << std::endl
                        << "vx   : " << vx      << std::endl
                        << "vy   : " << vy      << std::endl; 
            
        }

        LineCircle(double _Dx, double _Dy, double _r, double _w, double _phi, double _vx, double _vy)
        : Dx(_Dx), Dy(_Dy), r(_r), w(_w), phi(_phi), vx(_vx), vy(_vy) {}

        template<DubinsMove m> requires (m!=STRAIGHT)
        LineCircle(const PathShape<STRAIGHT> &s, const PathShape<m> &t)
        : Dx(t.x - s.x), Dy(t.y - s.y), r(t.p1), w(t.p2), phi(t.p4), vx(s.p1), vy(s.p2) {}

        [[gnu::pure]] double f(double t) const
        {
            double x = Dx + r*std::cos(w*t+phi) - t*vx;
            double y = Dy + r*std::sin(w*t+phi) - t*vy;
            return x*x + y*y;
        } 

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> x_I = Dx + r*cos(w*I+phi) - I*vx;
            interval<double,Policies> y_I = Dy + r*sin(w*I+phi) - I*vy;
            return square(x_I) + square(y_I);
        }

        [[gnu::pure]] double f_d(double t) const
        {
            double c1 = std::cos(w*t+phi);
            double s1 = std::sin(w*t+phi);

            double x = Dx + r*c1 - t*vx;
            double y = Dy + r*s1 - t*vy;

            double x_d = -w*r*s1 - vx;
            double y_d =  w*r*c1 - vy;

            return (x_d*x + y_d*y)*2;
        } 

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_d(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> c1 = cos(w*I+phi);
            interval<double,Policies> s1 = sin(w*I+phi);

            interval<double,Policies> x_I = Dx + r*c1 - I*vx;
            interval<double,Policies> y_I = Dy + r*s1 - I*vy;

            interval<double,Policies> x_d_I = -w*r*s1 - vx;
            interval<double,Policies> y_d_I =  w*r*c1 - vy;
            return (x_d_I*x_I + y_d_I*y_I)*static_cast<double>(2);
        }

        [[gnu::pure]] double f_dd(double t) const
        {
            double c1 = std::cos(w*t+phi);
            double s1 = std::sin(w*t+phi);

            double x = Dx + r*c1 - t*vx;
            double y = Dy + r*s1 - t*vy;

            double x_d = -w*r*s1 - vx;
            double y_d =  w*r*c1 - vy;

            double x_dd = -w*w*r*c1;
            double y_dd = -w*w*r*s1;

            return (x_d*x_d+x*x_dd + y_d*y_d+y*y_dd)*2;
        } 

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_dd(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> c1 = cos(w*I+phi);
            interval<double,Policies> s1 = sin(w*I+phi);

            interval<double,Policies> x_I = Dx + r*c1 - I*vx;
            interval<double,Policies> y_I = Dy + r*s1 - I*vy;

            interval<double,Policies> x_d_I = -w*r*s1 - vx;
            interval<double,Policies> y_d_I =  w*r*c1 - vy;

            interval<double,Policies> x_dd_I = -w*w*r*c1;
            interval<double,Policies> y_dd_I = -w*w*r*s1;
            
            return (square(x_d_I)+x_I*x_dd_I + square(y_d_I)+y_I*y_dd_I)*static_cast<double>(2);
        }

};