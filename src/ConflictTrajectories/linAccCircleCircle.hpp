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

class LinAccCircleCircle
{
    typedef OptimizationRounding::fast_rounding<double> Policies;

    private:
        const double Dx,Dy;
        const double r1,r2;
        const double a1,v1;
        const double a2,v2;
        const double phi1,phi2;

    public:
        void print_params()
        {
            std::cout   << std::setprecision(5)
                        << "Dx   : " << Dx      << std::endl
                        << "Dy   : " << Dy      << std::endl
                        << "r1   : " << r1      << std::endl
                        << "a1   : " << a1      << std::endl
                        << "v1   : " << v1      << std::endl
                        << "r2   : " << r2      << std::endl
                        << "a2   : " << a2      << std::endl
                        << "v2   : " << v2      << std::endl
                        << "phi1 : " << phi1    << std::endl
                        << "phi2 : " << phi2    << std::endl << std::endl;

        }

        LinAccCircleCircle(double _Dx, double _Dy, double _r1, double _r2, double _a1, double _v1, double _a2, double _v2, double _phi1, double _phi2)
        : Dx(_Dx), Dy(_Dy), r1(_r1), r2(_r2), a1(_a1), v1(_v1), a2(_a2), v2(_v2), phi1(_phi1), phi2(_phi2) {}

        template<DubinsMove m1, DubinsMove m2> requires ((m1!=STRAIGHT) && (m2!=STRAIGHT))
        LinAccCircleCircle(const PathShape<m1> &s, const PathShape<m2> &t, double _a1, double _v1, double _a2, double _v2)
        :   Dx(s.x - t.x), 
            Dy(s.y - t.y), 
            r1(s.p1), r2(t.p1), 
            a1(_a1/s.p1), v1(_v1/s.p1),
            a2(_a2/s.p2), v2(_v2/s.p2),
            phi1(s.p4), phi2(t.p4) {}

        [[gnu::pure]] double f(double t) const
        {
            double theta1 = (0.5*a1*t+v1)*t + phi1;
            double theta2 = (0.5*a2*t+v2)*t + phi2;
            double x = Dx + r1*std::cos(theta1) - r2*std::cos(theta2);
            double y = Dy + r1*std::sin(theta1) - r2*std::sin(theta2);
            return x*x + y*y;
        }

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            IntervalSolver::interval<double,Policies> theta1_I = (0.5*a1*square(I)+v1*I) + phi1;
            IntervalSolver::interval<double,Policies> theta2_I = (0.5*a2*square(I)+v2*I) + phi2;
            IntervalSolver::interval<double,Policies> x_I = Dx + r1*cos(theta1_I) - r2*cos(theta2_I);
            IntervalSolver::interval<double,Policies> y_I = Dy + r1*sin(theta1_I) - r2*sin(theta2_I);
            return square(x_I) + square(y_I);
        }

        [[gnu::pure]] double f_d(double t) const
        {
            double theta1 = (0.5*a1*t+v1)*t + phi1;
            double theta2 = (0.5*a2*t+v2)*t + phi2;

            double c1 = std::cos(theta1);
            double s1 = std::sin(theta1);

            double c2 = std::cos(theta2);
            double s2 = std::sin(theta2);

            double omega1 = a1*t + v1;
            double omega2 = a2*t + v2;

            double x = Dx + r1*c1 - r2*c2;
            double y = Dy + r1*s1 - r2*s2;

            double x_d = -omega1*r1*s1 + omega2*r2*s2;
            double y_d =  omega1*r1*c1 - omega2*r2*c2;

            return (x_d*x + y_d*y)*2;
        }

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_d(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> theta1_I = (0.5*a1*square(I)+v1*I) + phi1;
            interval<double,Policies> theta2_I = (0.5*a2*square(I)+v2*I) + phi2;

            interval<double,Policies> c1 = cos(theta1_I);
            interval<double,Policies> s1 = sin(theta1_I);

            interval<double,Policies> c2 = cos(theta2_I);
            interval<double,Policies> s2 = sin(theta2_I);

            interval<double,Policies> omega1_I = a1*I + v1;
            interval<double,Policies> omega2_I = a2*I + v2;

            interval<double,Policies> x_I = Dx + r1*c1 - r2*c2;
            interval<double,Policies> y_I = Dy + r1*s1 - r2*s2;

            interval<double,Policies> x_d_I = -omega1_I*r1*s1 + omega2_I*r2*s2;
            interval<double,Policies> y_d_I =  omega1_I*r1*c1 - omega2_I*r2*c2;
            return (x_d_I*x_I + y_d_I*y_I)*static_cast<double>(2);
        }

        [[gnu::pure]] double f_dd(double t) const
        {
            double theta1 = (0.5*a1*t+v1)*t + phi1;
            double theta2 = (0.5*a2*t+v2)*t + phi2;

            double c1 = std::cos(theta1);
            double s1 = std::sin(theta1);

            double c2 = std::cos(theta2);
            double s2 = std::sin(theta2);

            double omega1 = a1*t + v1;
            double omega2 = a2*t + v2;

            double omega1_d = a1;
            double omega2_d = a2;

            double x = Dx + r1*c1 - r2*c2;
            double y = Dy + r1*s1 - r2*s2;

            double x_d = -omega1*r1*s1 + omega2*r2*s2;
            double y_d =  omega1*r1*c1 - omega2*r2*c2;

            double x_dd = -r1*(omega1*omega1*c1 + omega1_d*s1) + r2*(omega2*omega2*c2 + omega2_d*s2);
            double y_dd =  r1*(-omega1*omega1*s1 + omega1_d*c1) - r2*(-omega2*omega2*s2 + omega2_d*c2);

            return (x_d*x_d+x*x_dd + y_d*y_d+y*y_dd)*2;
        }

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_dd(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> theta1_I = (0.5*a1*square(I)+v1*I) + phi1;
            interval<double,Policies> theta2_I = (0.5*a2*square(I)+v2*I) + phi2;

            interval<double,Policies> c1 = cos(theta1_I);
            interval<double,Policies> s1 = sin(theta1_I);

            interval<double,Policies> c2 = cos(theta2_I);
            interval<double,Policies> s2 = sin(theta2_I);

            interval<double,Policies> omega1_I = a1*I + v1;
            interval<double,Policies> omega2_I = a2*I + v2;

            interval<double,Policies> x_I = Dx + r1*c1 - r2*c2;
            interval<double,Policies> y_I = Dy + r1*s1 - r2*s2;

            interval<double,Policies> x_d_I = -omega1_I*r1*s1 + omega2_I*r2*s2;
            interval<double,Policies> y_d_I =  omega1_I*r1*c1 - omega2_I*r2*c2;

            interval<double,Policies> x_dd_I = -r1*(square(omega1_I)*c1 + a1*s1) + r2*(square(omega2_I)*c2 + a2*s2);
            interval<double,Policies> y_dd_I =  r1*(-square(omega1_I)*s1 + a1*c1) - r2*(-square(omega2_I)*s2 + a2*c2);

            return (square(x_d_I)+x_I*x_dd_I + square(y_d_I)+y_I*y_dd_I)*static_cast<double>(2);
        }

};

class LinAccCircleCircleVDist
{
    typedef OptimizationRounding::fast_rounding<double> Policies;

    private:
        const double Dx,Dy;
        const double r1,r2;
        const double a1,v1;
        const double a2,v2;
        const double phi1,phi2;
        const double tdist;

    public:
        void print_params()
        {
            std::cout   << std::setprecision(5)
                        << "Dx   : " << Dx      << std::endl
                        << "Dy   : " << Dy      << std::endl
                        << "r1   : " << r1      << std::endl
                        << "a1   : " << a1      << std::endl
                        << "v1   : " << v1      << std::endl
                        << "r2   : " << r2      << std::endl
                        << "a2   : " << a2      << std::endl
                        << "v2   : " << v2      << std::endl
                        << "phi1 : " << phi1    << std::endl
                        << "phi2 : " << phi2    << std::endl
                        << "tdist: " << tdist   << std::endl << std::endl;

        }

        LinAccCircleCircleVDist(double _Dx, double _Dy, double _r1, double _r2, double _a1, double _v1, double _a2, double _v2, double _phi1, double _phi2, double _tdist)
        : Dx(_Dx), Dy(_Dy), r1(_r1), r2(_r2), a1(_a1), v1(_v1), a2(_a2), v2(_v2), phi1(_phi1), phi2(_phi2), tdist(_tdist) {}

        template<DubinsMove m1, DubinsMove m2> requires ((m1!=STRAIGHT) && (m2!=STRAIGHT))
        LinAccCircleCircleVDist(const PathShape<m1> &s, const PathShape<m2> &t, double _a1, double _v1, double _a2, double _v2, double _tdist)
        :   Dx(s.x - t.x), 
            Dy(s.y - t.y), 
            r1(s.p1), r2(t.p1), 
            a1(_a1/s.p1), v1(_v1/s.p1),
            a2(_a2/s.p2), v2(_v2/s.p2),
            phi1(s.p4), phi2(t.p4), tdist(_tdist) {}

        [[gnu::pure]] double f(double t) const
        {
            double V1 = (a1*t+v1)*r1;
            double V2 = (a2*t+v2)*r2;
            double Vmax = std::max(V1,V2);

            double theta1 = (0.5*a1*t+v1)*t + phi1;
            double theta2 = (0.5*a2*t+v2)*t + phi2;
            double x = Dx + r1*std::cos(theta1) - r2*std::cos(theta2);
            double y = Dy + r1*std::sin(theta1) - r2*std::sin(theta2);
            return x*x + y*y - tdist*tdist * Vmax*Vmax;
        }

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            IntervalSolver::interval<double,Policies> V1 = (a1*I+v1)*r1;
            IntervalSolver::interval<double,Policies> V2 = (a2*I+v2)*r2;
            IntervalSolver::interval<double,Policies> Vmax = max(V1,V2);

            IntervalSolver::interval<double,Policies> theta1_I = (0.5*a1*square(I)+v1*I) + phi1;
            IntervalSolver::interval<double,Policies> theta2_I = (0.5*a2*square(I)+v2*I) + phi2;
            IntervalSolver::interval<double,Policies> x_I = Dx + r1*cos(theta1_I) - r2*cos(theta2_I);
            IntervalSolver::interval<double,Policies> y_I = Dy + r1*sin(theta1_I) - r2*sin(theta2_I);
            return square(x_I) + square(y_I) - tdist*tdist * square(Vmax);
        }

        [[gnu::pure]] double f_d(double t) const
        {
            double theta1 = (0.5*a1*t+v1)*t + phi1;
            double theta2 = (0.5*a2*t+v2)*t + phi2;

            double V1 = (a1*t+v1)*r1;
            double V2 = (a2*t+v2)*r2;
            double Vmax,dVmax;
            if (V1 > V2)
            {
                Vmax = V1;
                dVmax = a1*r1;
            }
            else
            {
                Vmax = V2;
                dVmax = a2*r2;
            }

            double c1 = std::cos(theta1);
            double s1 = std::sin(theta1);

            double c2 = std::cos(theta2);
            double s2 = std::sin(theta2);

            double omega1 = a1*t + v1;
            double omega2 = a2*t + v2;

            double x = Dx + r1*c1 - r2*c2;
            double y = Dy + r1*s1 - r2*s2;

            double x_d = -omega1*r1*s1 + omega2*r2*s2;
            double y_d =  omega1*r1*c1 - omega2*r2*c2;

            return (x_d*x + y_d*y - tdist*tdist*dVmax*Vmax)*2;
        }

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_d(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> theta1_I = (0.5*a1*square(I)+v1*I) + phi1;
            interval<double,Policies> theta2_I = (0.5*a2*square(I)+v2*I) + phi2;

            interval<double,Policies> V1 = (a1*I+v1)*r1;
            interval<double,Policies> V2 = (a2*I+v2)*r2;
            interval<double,Policies> Vmax = max(V1,V2);
            double dV1 = a1*r1;
            double dV2 = a2*r2;
            interval<double,Policies> dVmax(std::min(dV1,dV2), std::max(dV1,dV2));

            interval<double,Policies> c1 = cos(theta1_I);
            interval<double,Policies> s1 = sin(theta1_I);

            interval<double,Policies> c2 = cos(theta2_I);
            interval<double,Policies> s2 = sin(theta2_I);

            interval<double,Policies> omega1_I = a1*I + v1;
            interval<double,Policies> omega2_I = a2*I + v2;

            interval<double,Policies> x_I = Dx + r1*c1 - r2*c2;
            interval<double,Policies> y_I = Dy + r1*s1 - r2*s2;

            interval<double,Policies> x_d_I = -omega1_I*r1*s1 + omega2_I*r2*s2;
            interval<double,Policies> y_d_I =  omega1_I*r1*c1 - omega2_I*r2*c2;
            return (x_d_I*x_I + y_d_I*y_I - tdist*tdist*dVmax*Vmax)*static_cast<double>(2);
        }

        [[gnu::pure]] double f_dd(double t) const
        {
            double theta1 = (0.5*a1*t+v1)*t + phi1;
            double theta2 = (0.5*a2*t+v2)*t + phi2;

            double V1 = (a1*t+v1)*r1;
            double V2 = (a2*t+v2)*r2;
            double Vmax,dVmax;
            if (V1 > V2)
            {
                Vmax = V1;
                dVmax = a1*r1;
            }
            else
            {
                Vmax = V2;
                dVmax = a2*r2;
            }

            double c1 = std::cos(theta1);
            double s1 = std::sin(theta1);

            double c2 = std::cos(theta2);
            double s2 = std::sin(theta2);

            double omega1 = a1*t + v1;
            double omega2 = a2*t + v2;

            double omega1_d = a1;
            double omega2_d = a2;

            double x = Dx + r1*c1 - r2*c2;
            double y = Dy + r1*s1 - r2*s2;

            double x_d = -omega1*r1*s1 + omega2*r2*s2;
            double y_d =  omega1*r1*c1 - omega2*r2*c2;

            double x_dd = -r1*(omega1*omega1*c1 + omega1_d*s1) + r2*(omega2*omega2*c2 + omega2_d*s2);
            double y_dd =  r1*(-omega1*omega1*s1 + omega1_d*c1) - r2*(-omega2*omega2*s2 + omega2_d*c2);

            return (x_d*x_d+x*x_dd + y_d*y_d+y*y_dd - tdist*tdist*dVmax*dVmax)*2;
        }

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_dd(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> theta1_I = (0.5*a1*square(I)+v1*I) + phi1;
            interval<double,Policies> theta2_I = (0.5*a2*square(I)+v2*I) + phi2;

            interval<double,Policies> V1 = (a1*I+v1)*r1;
            interval<double,Policies> V2 = (a2*I+v2)*r2;
            interval<double,Policies> Vmax = max(V1,V2);
            double dV1 = a1*r1;
            double dV2 = a2*r2;
            interval<double,Policies> dVmax(std::min(dV1,dV2), std::max(dV1,dV2));

            interval<double,Policies> c1 = cos(theta1_I);
            interval<double,Policies> s1 = sin(theta1_I);

            interval<double,Policies> c2 = cos(theta2_I);
            interval<double,Policies> s2 = sin(theta2_I);

            interval<double,Policies> omega1_I = a1*I + v1;
            interval<double,Policies> omega2_I = a2*I + v2;

            interval<double,Policies> x_I = Dx + r1*c1 - r2*c2;
            interval<double,Policies> y_I = Dy + r1*s1 - r2*s2;

            interval<double,Policies> x_d_I = -omega1_I*r1*s1 + omega2_I*r2*s2;
            interval<double,Policies> y_d_I =  omega1_I*r1*c1 - omega2_I*r2*c2;

            interval<double,Policies> x_dd_I = -r1*(square(omega1_I)*c1 + a1*s1) + r2*(square(omega2_I)*c2 + a2*s2);
            interval<double,Policies> y_dd_I =  r1*(-square(omega1_I)*s1 + a1*c1) - r2*(-square(omega2_I)*s2 + a2*c2);
            return (square(x_d_I)+x_I*x_dd_I + square(y_d_I)+y_I*y_dd_I - tdist*tdist*dVmax*dVmax)*static_cast<double>(2);
        }

};