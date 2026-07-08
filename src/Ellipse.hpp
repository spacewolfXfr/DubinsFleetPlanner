#pragma once

#include <cmath>
#include <tuple>

class Ellipse
{
private:
    double a, b, c; // Semi-major axis, semi-minor axis, and eccentricity of the ellipse
    double fx,fy,fbx,fby; // Foci of the ellipse, respectively F and F'
    double Cx, Cy; // Center of the ellipse
    double offset_angle; // Angle such that the parametric at 0 is aligned with (FF')

    void _recompute_center_from_foci(); // Compute C from F,F'
    void _recompute_foci_from_Cab(); // Compute F,F' from C,a,b,c
    void _recompute_c_from_foci(); // Compute c from F,F'

    // Default constructor: circle at origin of radius 1
    Ellipse() : a(1), b(1), c(0), fx(0), fy(0), fbx(0), fby(0), Cx(0), Cy(0), offset_angle(0) {}


public:
    // ----- Constructors ----- //

    // Constructor from foci and semi-major axis
    static Ellipse from_foci(double _fx, double _fy, double _fbx, double _fby, double _a)
    {
        Ellipse output = Ellipse();
        output.fx = _fx;
        output.fy = _fy;
        output.fbx = _fbx;
        output.fby = _fby;
        output.a = _a;
        output._recompute_center_from_foci();
        output._recompute_c_from_foci();
        output.b = std::sqrt(output.a*output.a - output.c*output.c);
        output.offset_angle = std::atan2(output.fy-output.fby,output.fx-output.fbx);
        return output;
    }

    // Constructor from center, semi-major and semi-minor axes, and offset angle
    static Ellipse from_parametric(double _Cx, double _Cy, double _a, double _b, double _offset_angle)
    {
        Ellipse output = Ellipse();
        output.Cx = _Cx;
        output.Cy = _Cy;
        output.a = _a;
        output.b = _b;
        output.offset_angle = _offset_angle;
        output.c = std::sqrt(_a*_a - _b*_b);
        output._recompute_foci_from_Cab();
        return output;
    }

    // ----- Point generation ----- //

    // Get the point at the given angle on the ellipse, where angle 0 is aligned with the major axis
    std::tuple<double,double> point_at_angle(double angle) const;

    // Get the polar radius of the ellipse at the given angle, where angle 0 is aligned with the major axis
    double polar_radius(double angle) const;

    // Uniformly sample a point inside the ellipse, using the given seed for the random number generator
    std::tuple<double,double> random_point_inside(int seed) const;
    
    // ----- Getters ----- //

    std::tuple<double,double> foci_1() const
    {
        return {fx,fy};
    }

    std::tuple<double,double> foci_2() const
    {
        return {fbx,fby};
    }
};