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

#include <tuple>
#include <boost/math/tools/roots.hpp>

#include "ProjectHeader.h"

#include "Primitives.hpp"
#include "Pose3D.hpp"

/**
 * @brief Given a normalized problem Dubins path planning problem with a minimum turn radius
 * and a specified length, find a turn radius such that the path achieves said length
 * 
 * @param alpha     Starting orientation
 * @param beta      Ending orientation
 * @param d         X coordinate of the destination point
 * @param min_rho   Minimal turning radius
 * @param target_l  Desired length
 * @param tol       Desired solution accuracy
 * @return double   NAN if impossible, otherwise the turning radius achieving le desired length
 */
double fit_LSL(double alpha, double beta, double d, double min_rho, double target_l, double tol);
double fit_RSR(double alpha, double beta, double d, double min_rho, double target_l, double tol);
double fit_RSL(double alpha, double beta, double d, double min_rho, double target_l, double tol);
double fit_LSR(double alpha, double beta, double d, double min_rho, double target_l, double tol);
double fit_RLR(double alpha, double beta, double d, double min_rho, double target_l, double tol);
double fit_LRL(double alpha, double beta, double d, double min_rho, double target_l, double tol);
double fit_SRS(double alpha, double beta, double d, double min_rho, double target_l, double tol);
double fit_SLS(double alpha, double beta, double d, double min_rho, double target_l, double tol);


/**
 * @brief Given end poses, a target length and a tolerance, compute the length of the added line(s)
 * to achieve the desired value. Assume the turn radius is 1.
 * 
 * @param start     Start pose
 * @param end       End pose
 * @param target_l  Target length
 * @param tol       Solver precision tolerance
 * @param ratio     Ratio splitting where the straight should be added, with 1 being all at start, 0 all at end and 0.5 equally shared between the two 
 * @return double   Length added in straight lines
 */
double fit_shift_LSL_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio);
double fit_shift_RSR_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio);
double fit_shift_RSL_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio);
double fit_shift_LSR_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio);
double fit_shift_RLR_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio);
double fit_shift_LRL_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio);
double fit_shift_SLS_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio);
double fit_shift_SRS_ratio(Pose3D start, Pose3D end, double target_l, double tol, double ratio);
