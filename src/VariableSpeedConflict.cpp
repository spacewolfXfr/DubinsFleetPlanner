#include "VariableSpeedConflict.hpp"

#include <boost/math/tools/cubic_roots.hpp>

#include <Eigen/Dense>

#include <ScalarMin/IntervalSolver.hpp>

#include "ConflictDetection.hpp"

#include "ConflictTrajectories/linAccLineCircle.hpp"
#include "ConflictTrajectories/linAccCircleCircle.hpp"

/************************* Utils *************************/

/**
 * @brief Horner's method for fast degree 4 polynomial evaluation
 *
 * Cf https://en.wikipedia.org/wiki/Horner's_method
 *
 * @param x : Evaluation point for the polynomial
 * @param a4 : Degree 4 coefficient of the polynomial
 * @param a3 : Degree 3 coefficient of the polynomial
 * @param a2 : Degree 2 coefficient of the polynomial
 * @param a1 : Degree 1 coefficient of the polynomial
 * @param a0 : Degree 0 coefficient of the polynomial
 * @return double : Value of the polynomial evaluated at `x`
 */
static double p4_eval(double x, double a4, double a3, double a2, double a1, double a0)
{
    return a0 + x * (a1 + x * (a2 + x * (a3 + x * (a4))));
}

/************************* STRAIGHT - STRAIGHT *************************/

std::pair<double, double> temporal_XY_time_slack_straights(const DynamicPathShape &s1, const DynamicPathShape &s2,
                                                           const SpeedProfile& s1_prof, const SpeedProfile& s2_prof, double duration, double tdist, double tol)
{
    assert(s1.m == STRAIGHT && s2.m == STRAIGHT);

    // Gather parameters
    double a1 = s1_prof.acceleration;
    double v1 = s1_prof.base_speed;
    double a2 = s2_prof.acceleration;
    double v2 = s2_prof.base_speed;

    Eigen::Vector2d dp(s1.x - s2.x, s1.y - s2.y);
    Eigen::Vector2d n1(s1.p1, s1.p2);
    Eigen::Vector2d n2(s2.p1, s2.p2);

    // No acceleration: lower degree polynomial
    if (std::abs(a1) < DubinsFleetPlanner_ZERO_TOLERANCE && std::abs(a2) < DubinsFleetPlanner_ZERO_TOLERANCE)
    {
        double separation = tdist * std::max(v1, v2);
        std::pair<double, double> result = temporal_XY_dist(s1, s2, s1_prof, s2_prof, duration, tol);
        result.second = result.second*result.second - separation*separation;
        return result;
    }

    // Compute the polynomial coefficients for the distance function with slack
    // The distance is given by: d(t) = ||p1(t) - p2(t)||^2
    // where p1(t) = p1 + t*n1*(v1 + t*a1/2) and p2(t) = p2 + t*n2*(v2 + t*a2/2)
    // So d(t) = ||(p1 - p2) + t*(n1*(v1 + t*a1/2) - n2*(v2 + t*a2/2))||^2
    // The objective function f(t) = d(t) - tdist^2 * max(a1*t+v1,a2*t+v2)^2

    double c0 = dp.squaredNorm() - tdist * tdist * v1 * v1;
    double c1 = 2 * dp.dot(n1 * v1 - n2 * v2) - 2 * tdist * tdist * a1 * v1;
    double c2 = (n1 * v1 - n2 * v2).squaredNorm() + dp.dot(n1 * a1 - n2 * a2) - tdist * tdist * a1 * a1;
    double c3 = (n1 * v1 - n2 * v2).dot(n1 * a1 - n2 * a2);
    double c4 = (n1 * a1 - n2 * a2).squaredNorm() / 4.;

    double c0_bis = dp.squaredNorm() - tdist * tdist * v2 * v2;
    double c1_bis = 2 * dp.dot(n1 * v1 - n2 * v2) - 2 * tdist * tdist * a2 * v2;
    double c2_bis = (n1 * v1 - n2 * v2).squaredNorm() + dp.dot(n1 * a1 - n2 * a2) - tdist * tdist * a2 * a2;

    // Compute the polynomial coefficients for the derivative of the distance function
    double c1_d = c1;
    double c2_d = 2 * c2;
    double c3_d = 3 * c3;
    double c4_d = 4 * c4;

    double c1_bis_d = c1_bis;
    double c2_bis_d = 2 * c2_bis;

    // Obtain the critical points of the distance function by solving the derivative polynomial
    auto roots = boost::math::tools::cubic_roots(c4_d, c3_d, c2_d, c1_d);
    auto roots_bis = boost::math::tools::cubic_roots(c4_d, c3_d, c2_bis_d, c1_bis_d);

    std::vector<double> candidates = {0., duration};
    for (const auto &root : roots)
    {
        if (!std::isnan(root) && root >= 0. && root <= duration)
        {
            candidates.push_back(root);
        }
    }

    std::vector<double> candidates_bis = {0., duration};
    for (const auto &root : roots_bis)
    {
        if (!std::isnan(root) && root >= 0. && root <= duration)
        {
            candidates_bis.push_back(root);
        }
    }

    double min_loc = 0.;
    double min_val = c0;
    for (const auto &t : candidates)
    {
        double dist2 = p4_eval(t, c4, c3, c2, c1, c0);
        if (dist2 < min_val)
        {
            min_val = dist2;
            min_loc = t;
        }
    }
    for (const auto &t : candidates_bis)
    {
        double dist2 = p4_eval(t, c4, c3, c2_bis, c1_bis, c0_bis);
        if (dist2 < min_val)
        {
            min_val = dist2;
            min_loc = t;
        }
    }

    return {min_loc, min_val};
}

/************************* LINE - CIRCLE *************************/

template <bool use_derivatives>
std::pair<double, double> temporal_XY_time_slack_lineCircle(const DynamicPathShape &s1, const DynamicPathShape &s2,
                                                            const SpeedProfile& s1_prof, const SpeedProfile& s2_prof, double duration, double tdist, double tol)
{
    typedef OptimizationRounding::fast_rounding<double> Policies;

    assert(s1.m == STRAIGHT && s2.m != STRAIGHT);

    auto s1_cpy = s1;
    auto s2_cpy = s2;

    double aL = s1_prof.acceleration;
    double vL = s1_prof.base_speed;
    double aC = s2_prof.acceleration;
    double vC = s2_prof.base_speed;

    path_set_planar_speed(s1_cpy, 1.);
    path_set_planar_speed(s2_cpy, 1.);

    LinAccLineCircleVDist problem(s1_cpy, s2_cpy, aL, vL, aC, vC, tdist);

    auto fbind = [problem](double x)
    { return problem.f(x); };

    auto Fbind = [problem](const IntervalSolver::interval<double, Policies> &I)
    { return problem.F(I); };

    double min_loc;

    if (use_derivatives)
    {
        auto f_dbind = [problem](double x)
        { return problem.f_d(x); };
        auto F_dbind = [problem](const IntervalSolver::interval<double, Policies> &I)
        { return problem.F_d(I); };
        auto f_ddbind = [problem](double x)
        { return problem.f_dd(x); };
        auto F_ddbind = [problem](const IntervalSolver::interval<double, Policies> &I)
        { return problem.F_dd(I); };

        min_loc = IntervalSolver::minimize_interval_method<double, Policies>(
            0., duration,
            fbind, Fbind,
            f_dbind, F_dbind,
            f_ddbind, F_ddbind,
            tol, 0.);
    }
    else
    {
        min_loc = IntervalSolver::minimize_interval_method<double, Policies>(
            0., duration,
            fbind, Fbind,
            tol, 0.);
    }

    return {min_loc, problem.f(min_loc)};
}

/************************* CIRCLE - CIRCLE *************************/

template <bool use_derivatives>
std::pair<double, double> temporal_XY_time_slack_circles(const DynamicPathShape &s1, const DynamicPathShape &s2,
                                                         const SpeedProfile& s1_prof, const SpeedProfile& s2_prof, double duration, double tdist, double tol)
{
    typedef OptimizationRounding::fast_rounding<double> Policies;

    assert(s1.m != STRAIGHT && s2.m != STRAIGHT);

    auto s1_cpy = s1;
    auto s2_cpy = s2;

    // Gather parameters
    double a1 = s1_prof.acceleration;
    double v1 = s1_prof.base_speed;
    double a2 = s2_prof.acceleration;
    double v2 = s2_prof.base_speed;

    path_set_planar_speed(s1_cpy, 1.);
    path_set_planar_speed(s2_cpy, 1.);

    LinAccCircleCircleVDist problem(s1_cpy, s2_cpy, a1, v1, a2, v2, tdist);

    auto fbind = [problem](double x)
    { return problem.f(x); };
    auto Fbind = [problem](const IntervalSolver::interval<double, Policies> &I)
    { return problem.F(I); };
    double min_loc;

    if (use_derivatives)
    {
        auto f_dbind = [problem](double x)
        { return problem.f_d(x); };
        auto F_dbind = [problem](const IntervalSolver::interval<double, Policies> &I)
        { return problem.F_d(I); };
        auto f_ddbind = [problem](double x)
        { return problem.f_dd(x); };
        auto F_ddbind = [problem](const IntervalSolver::interval<double, Policies> &I)
        { return problem.F_dd(I); };

        min_loc = IntervalSolver::minimize_interval_method<double, Policies>(
            0., duration,
            fbind, Fbind,
            f_dbind, F_dbind,
            f_ddbind, F_ddbind,
            tol, 0.);
    }
    else
    {
        min_loc = IntervalSolver::minimize_interval_method<double, Policies>(
            0., duration,
            fbind, Fbind,
            tol, 0.);
    }

    return {min_loc, problem.f(min_loc)};
}

/************************* Main *************************/

template <bool use_derivatives>
std::pair<double, double> temporal_XY_time_slack(const DynamicPathShape &s1, const DynamicPathShape &s2,
                                                 const SpeedProfile& s1_prof, const SpeedProfile& s2_prof, double duration, double tdist, double tol)
{
    if (s1.m == STRAIGHT && s2.m == STRAIGHT)
    {
        return temporal_XY_time_slack_straights(s1, s2, s1_prof, s2_prof, duration, tdist, tol);
    }
    else if (s1.m != STRAIGHT && s2.m != STRAIGHT)
    {
        return temporal_XY_time_slack_circles<use_derivatives>(s1, s2, s1_prof, s2_prof, duration, tdist, tol);
    }
    else
    {
        if (s1.m == STRAIGHT)
        {
            return temporal_XY_time_slack_lineCircle<use_derivatives>(s1, s2, s1_prof, s2_prof, duration, tdist, tol);
        }
        else
        {
            return temporal_XY_time_slack_lineCircle<use_derivatives>(s2, s1, s2_prof, s1_prof, duration, tdist, tol);
        }
    }
}