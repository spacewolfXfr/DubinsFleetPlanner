#include "Ellipse.hpp"

#include <random>

// ---------- Helpers ---------- // 

void Ellipse::_recompute_center_from_foci()
{
    Cx = (fx + fbx) / 2.;
    Cy = (fy + fby) / 2.;
}

void Ellipse::_recompute_foci_from_Cab()
{
    fx = Cx + c * std::cos(offset_angle);
    fy = Cy + c * std::sin(offset_angle);
    fbx = Cx - c * std::cos(offset_angle);
    fby = Cy - c * std::sin(offset_angle);
}

void Ellipse::_recompute_c_from_foci()
{
    double dx = fx - fbx;
    double dy = fy - fby;
    c = std::sqrt(dx * dx + dy * dy) / 2.;
}

// ---------- Constructors ---------- //

std::optional<Ellipse> Ellipse::from_foci(double _fx, double _fy, double _fbx, double _fby, double _a)
{
    Ellipse output = Ellipse();
    output.fx = _fx;
    output.fy = _fy;
    output.fbx = _fbx;
    output.fby = _fby;
    output.a = _a;
    output._recompute_center_from_foci();
    output._recompute_c_from_foci();

    if (output.c > output.a)
    {
        return std::nullopt;
    }
        

    output.b = std::sqrt(output.a*output.a - output.c*output.c);
    output.offset_angle = std::atan2(output.fy-output.fby,output.fx-output.fbx);
    return output;
}

std::optional<Ellipse> Ellipse::from_parametric(double _Cx, double _Cy, double _a, double _b, double _offset_angle)
{
    if (_a < _b)
        return std::nullopt;
    
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

// ---------- Point generation ---------- //

std::tuple<double,double> Ellipse::point_at_angle(double angle) const
{
    double x = Cx + a * std::cos(angle) * std::cos(offset_angle) - b * std::sin(angle) * std::sin(offset_angle);
    double y = Cy + a * std::cos(angle) * std::sin(offset_angle) + b * std::sin(angle) * std::cos(offset_angle);
    return {x,y};
}

double Ellipse::polar_radius(double angle) const
{
    double cos_theta = std::cos(angle - offset_angle);
    double sin_theta = std::sin(angle - offset_angle);
    return (a * b) / std::sqrt((b * cos_theta) * (b * cos_theta) + (a * sin_theta) * (a * sin_theta));
}

std::tuple<double,double> Ellipse::random_point_inside(int seed) const
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit_dist(0,1);
    std::uniform_int_distribution<int> cadrant_dist(0,3);

    double u = unit_dist(rng)/4;
    double theta = atan(b/a * std::tan(2*M_PI*u));

    int cadrant = cadrant_dist(rng);
    switch (cadrant)
    {
        case 0:
            break;
        case 1:
            theta = M_PI - theta;
            break;
        case 2:
            theta = M_PI + theta;
            break;
        case 3:
            theta = -theta;
            break;
    }

    double r = polar_radius(theta);
    double radius = std::sqrt(unit_dist(rng)) * r;
    return {Cx + radius * std::cos(theta), Cy + radius * std::sin(theta)};
}
