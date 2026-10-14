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


class LinAccLineCircle
{
    typedef OptimizationRounding::fast_rounding<double> Policies;

    private:
        const double Dx,Dy;
        const double r,phi;
        const double aC,vC;
        const double aL,vL;
        const double nx,ny;

    public:
        void print_params()
        {
            std::cout   << std::setprecision(5)
                        << "Dx   : " << Dx      << std::endl
                        << "Dy   : " << Dy      << std::endl
                        << "r    : " << r       << std::endl
                        << "phi  : " << phi     << std::endl
                        << "aC   : " << aC      << std::endl
                        << "vC   : " << vC      << std::endl
                        << "aL   : " << aL      << std::endl
                        << "vL   : " << vL      << std::endl
                        << "nx   : " << nx      << std::endl
                        << "ny   : " << ny      << std::endl << std::endl;
        }

        LinAccLineCircle(double _Dx, double _Dy, double _r, double _phi, double _aC, double _vC, double _aL, double _vL, double _nx, double _ny)
        : Dx(_Dx), Dy(_Dy), r(_r), phi(_phi), aC(_aC), vC(_vC), aL(_aL), vL(_vL), nx(_nx), ny(_ny) {}

        template<DubinsMove m> requires (m!=STRAIGHT)
        LinAccLineCircle(const PathShape<STRAIGHT> &s, const PathShape<m> &t, double _aC, double _vC, double _aL, double _vL)
        : Dx(t.x - s.x), Dy(t.y - s.y), r(t.p1), phi(t.p4), aC(_aC), vC(_vC), aL(_aL), vL(_vL), nx(s.p1), ny(s.p2) {}

        [[gnu::pure]] double f(double t) const
        {
            double theta = (0.5*aC*t+vC)*t + phi;
            double line_speed = 0.5*aL*t+vL;
            double x = Dx + r*std::cos(theta) - t*nx*line_speed;
            double y = Dy + r*std::sin(theta) - t*ny*line_speed;
            return x*x + y*y;
        }

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> theta_I = 0.5*aC*square(I)+vC*I + phi;
            interval<double,Policies> x_I = Dx + r*cos(theta_I) - nx*(0.5*aL*square(I)+vL*I);
            interval<double,Policies> y_I = Dy + r*sin(theta_I) - ny*(0.5*aL*square(I)+vL*I);
            return square(x_I) + square(y_I);
        }

        [[gnu::pure]] double f_d(double t) const
        {
            double theta = (0.5*aC*t+vC)*t + phi;
            double c = std::cos(theta);
            double s = std::sin(theta);

            double omega = aC*t + vC;
            double speed = 0.5*aL*t + vL;
            double line_d = aL*t + vL;

            double x = Dx + r*c - t*nx*speed;
            double y = Dy + r*s - t*ny*speed;

            double x_d = -omega*r*s - nx*line_d;
            double y_d =  omega*r*c - ny*line_d;

            return (x_d*x + y_d*y)*2;
        }

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_d(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> theta_I = 0.5*aC*square(I)+vC*I + phi;

            interval<double,Policies> c = cos(theta_I);
            interval<double,Policies> s = sin(theta_I);

            interval<double,Policies> omega_I = aC*I + vC;
            interval<double,Policies> line_d_I = aL*I + vL;

            interval<double,Policies> x_I = Dx + r*c - nx*(0.5*aL*square(I)+vL*I);
            interval<double,Policies> y_I = Dy + r*s - ny*(0.5*aL*square(I)+vL*I);

            interval<double,Policies> x_d_I = -omega_I*r*s - nx*line_d_I;
            interval<double,Policies> y_d_I =  omega_I*r*c - ny*line_d_I;
            return (x_d_I*x_I + y_d_I*y_I)*static_cast<double>(2);
        }

        [[gnu::pure]] double f_dd(double t) const
        {
            double theta = (0.5*aC*t+vC)*t + phi;
            double c = std::cos(theta);
            double s = std::sin(theta);

            double omega = aC*t + vC;
            double omega_d = aC;
            double line_d = aL*t + vL;

            double x = Dx + r*c - t*nx*(0.5*aL*t + vL);
            double y = Dy + r*s - t*ny*(0.5*aL*t + vL);

            double x_d = -omega*r*s - nx*line_d;
            double y_d =  omega*r*c - ny*line_d;

            double x_dd = -r*(omega*omega*c + omega_d*s) - aL*nx;
            double y_dd =  r*(-omega*omega*s + omega_d*c) - aL*ny;

            return (x_d*x_d+x*x_dd + y_d*y_d+y*y_dd)*2;
        }

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_dd(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> theta_I = 0.5*aC*square(I)+vC*I + phi;

            interval<double,Policies> c = cos(theta_I);
            interval<double,Policies> s = sin(theta_I);

            interval<double,Policies> omega_I = aC*I + vC;
            interval<double,Policies> line_d_I = aL*I + vL;

            interval<double,Policies> x_I = Dx + r*c - nx*(0.5*aL*square(I)+vL*I);
            interval<double,Policies> y_I = Dy + r*s - ny*(0.5*aL*square(I)+vL*I);

            interval<double,Policies> x_d_I = -omega_I*r*s - nx*line_d_I;
            interval<double,Policies> y_d_I =  omega_I*r*c - ny*line_d_I;

            interval<double,Policies> x_dd_I = -r*(square(omega_I)*c + aC*s) - aL*nx;
            interval<double,Policies> y_dd_I =  r*(-square(omega_I)*s + aC*c) - aL*ny;

            return (square(x_d_I)+x_I*x_dd_I + square(y_d_I)+y_I*y_dd_I)*static_cast<double>(2);
        }

};



class LinAccLineCircleVDist
{
    typedef OptimizationRounding::fast_rounding<double> Policies;

    private:
        const double Dx,Dy;
        const double r,phi;
        const double aC,vC;
        const double aL,vL;
        const double nx,ny;
        const double tdist;

    public:
        void print_params()
        {
            std::cout   << std::setprecision(5)
                        << "Dx   : " << Dx      << std::endl
                        << "Dy   : " << Dy      << std::endl
                        << "r    : " << r       << std::endl
                        << "phi  : " << phi     << std::endl
                        << "aC   : " << aC      << std::endl
                        << "vC   : " << vC      << std::endl
                        << "aL   : " << aL      << std::endl
                        << "vL   : " << vL      << std::endl
                        << "nx   : " << nx      << std::endl
                        << "ny   : " << ny      << std::endl
                        << "tdist: " << tdist   << std::endl << std::endl;
        }

        LinAccLineCircleVDist(double _Dx, double _Dy, double _r, double _phi, double _aC, double _vC, double _aL, double _vL, double _nx, double _ny, double _tdist)
        : Dx(_Dx), Dy(_Dy), r(_r), phi(_phi), aC(_aC), vC(_vC), aL(_aL), vL(_vL), nx(_nx), ny(_ny), tdist(_tdist) {}

        template<DubinsMove m> requires (m!=STRAIGHT)
        LinAccLineCircleVDist(const PathShape<STRAIGHT> &s, const PathShape<m> &t, double _aC, double _vC, double _aL, double _vL, double _tdist)
        : Dx(t.x - s.x), Dy(t.y - s.y), r(t.p1), phi(t.p4), aC(_aC), vC(_vC), aL(_aL), vL(_vL), nx(s.p1), ny(s.p2), tdist(_tdist) {}

        [[gnu::pure]] double f(double t) const
        {
            double V1 = (aC*t+vC)*r;
            double V2 = aL*t+vL;
            double Vmax = std::max(V1,V2);

            double theta = (0.5*aC*t+vC)*t + phi;
            double line_speed = 0.5*aL*t+vL;
            double x = Dx + r*std::cos(theta) - t*nx*line_speed;
            double y = Dy + r*std::sin(theta) - t*ny*line_speed;
            return x*x + y*y - tdist*tdist*Vmax*Vmax;
        }

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            IntervalSolver::interval<double,Policies> V1 = (aC*I+vC)*r;
            IntervalSolver::interval<double,Policies> V2 = aL*I+vL;
            IntervalSolver::interval<double,Policies> Vmax = max(V1,V2);

            interval<double,Policies> theta_I = 0.5*aC*square(I)+vC*I + phi;
            interval<double,Policies> x_I = Dx + r*cos(theta_I) - nx*(0.5*aL*square(I)+vL*I);
            interval<double,Policies> y_I = Dy + r*sin(theta_I) - ny*(0.5*aL*square(I)+vL*I);
            return square(x_I) + square(y_I) - tdist*tdist * square(Vmax);
        }

        [[gnu::pure]] double f_d(double t) const
        {
            double theta = (0.5*aC*t+vC)*t + phi;
            double c = std::cos(theta);
            double s = std::sin(theta);

            double V1 = (aC*t+vC)*r;
            double V2 = aL*t+vL;
            double Vmax,dVmax;
            if (V1 > V2)
            {
                Vmax = V1;
                dVmax = aC*r;
            }
            else
            {
                Vmax = V2;
                dVmax = aL;
            }

            double omega = aC*t + vC;
            double speed = 0.5*aL*t + vL;
            double line_d = aL*t + vL;

            double x = Dx + r*c - t*nx*speed;
            double y = Dy + r*s - t*ny*speed;

            double x_d = -omega*r*s - nx*line_d;
            double y_d =  omega*r*c - ny*line_d;

            return (x_d*x + y_d*y - tdist*tdist*dVmax*Vmax)*2;
        }

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_d(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> theta_I = 0.5*aC*square(I)+vC*I + phi;

            interval<double,Policies> V1 = (aC*I+vC)*r;
            interval<double,Policies> V2 = aL*I+vL;
            interval<double,Policies> Vmax = max(V1,V2);
            double dV1 = aC*r;
            double dV2 = aL;
            interval<double,Policies> dVmax(std::min(dV1,dV2), std::max(dV1,dV2));

            interval<double,Policies> c = cos(theta_I);
            interval<double,Policies> s = sin(theta_I);

            interval<double,Policies> omega_I = aC*I + vC;
            interval<double,Policies> line_d_I = aL*I + vL;

            interval<double,Policies> x_I = Dx + r*c - nx*(0.5*aL*square(I)+vL*I);
            interval<double,Policies> y_I = Dy + r*s - ny*(0.5*aL*square(I)+vL*I);

            interval<double,Policies> x_d_I = -omega_I*r*s - nx*line_d_I;
            interval<double,Policies> y_d_I =  omega_I*r*c - ny*line_d_I;
            return (x_d_I*x_I + y_d_I*y_I - tdist*tdist*dVmax*Vmax)*static_cast<double>(2);
        }

        [[gnu::pure]] double f_dd(double t) const
        {
            double theta = (0.5*aC*t+vC)*t + phi;
            double c = std::cos(theta);
            double s = std::sin(theta);

            double V1 = (aC*t+vC)*r;
            double V2 = aL*t+vL;
            double Vmax,dVmax;
            if (V1 > V2)
            {
                Vmax = V1;
                dVmax = aC*r;
            }
            else
            {
                Vmax = V2;
                dVmax = aL;
            }


            double omega = aC*t + vC;
            double omega_d = aC;
            double line_d = aL*t + vL;

            double x = Dx + r*c - t*nx*(0.5*aL*t + vL);
            double y = Dy + r*s - t*ny*(0.5*aL*t + vL);

            double x_d = -omega*r*s - nx*line_d;
            double y_d =  omega*r*c - ny*line_d;

            double x_dd = -r*(omega*omega*c + omega_d*s) - aL*nx;
            double y_dd =  r*(-omega*omega*s + omega_d*c) - aL*ny;

            return (x_d*x_d+x*x_dd + y_d*y_d+y*y_dd - tdist*tdist*dVmax*dVmax)*2;
        }

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_dd(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> theta_I = 0.5*aC*square(I)+vC*I + phi;

            interval<double,Policies> V1 = (aC*I+vC)*r;
            interval<double,Policies> V2 = aL*I+vL;
            interval<double,Policies> Vmax = max(V1,V2);
            double dV1 = aC*r;
            double dV2 = aL;
            interval<double,Policies> dVmax(std::min(dV1,dV2), std::max(dV1,dV2));


            interval<double,Policies> c = cos(theta_I);
            interval<double,Policies> s = sin(theta_I);

            interval<double,Policies> omega_I = aC*I + vC;
            interval<double,Policies> line_d_I = aL*I + vL;

            interval<double,Policies> x_I = Dx + r*c - nx*(0.5*aL*square(I)+vL*I);
            interval<double,Policies> y_I = Dy + r*s - ny*(0.5*aL*square(I)+vL*I);

            interval<double,Policies> x_d_I = -omega_I*r*s - nx*line_d_I;
            interval<double,Policies> y_d_I =  omega_I*r*c - ny*line_d_I;

            interval<double,Policies> x_dd_I = -r*(square(omega_I)*c + aC*s) - aL*nx;
            interval<double,Policies> y_dd_I =  r*(-square(omega_I)*s + aC*c) - aL*ny;

            return (square(x_d_I)+x_I*x_dd_I + square(y_d_I)+y_I*y_dd_I - tdist*tdist*dVmax*dVmax)*static_cast<double>(2);
        }

};