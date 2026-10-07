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

#include <memory>
#include <vector>

// -------------------- General util -------------------- //

template<typename T>
std::vector<std::shared_ptr<T>> make_shared(std::vector<std::unique_ptr<T>>& v)
{
    std::vector<std::shared_ptr<T>> output(v.size());
    for(uint i = 0; i < v.size(); i++)
    {
        output[i] = std::move(v[i]);
    }
    return output;
}

/**
 * @brief Assuming vec is in increasing order, find the index i such that vec[i] <= val <= vec[i+1]
 * If vec.front() > val, returns 0
 * If vec.back() < val, returns vec.size()-1
 * 
 * @tparam T type of the vector, should by comparable
 * @param vec Vector of values, in increasing order
 * @param val Value to search in the vector
 * @return size_t index of the 'insertion location' in vec
 */
template<typename T>
size_t binary_search(std::vector<T> vec, T val)
{
    
    size_t start = 0;
    size_t end = vec.size()-1;

    if (val < vec[start])
    {
        return start;
    }
    else if (val >= vec[end])
    {
        return end;
    }

    do
    {
        size_t mid = (start+end)/2;
        
        if (val >= vec[mid])
        {
            start = mid;
        }
        else
        {
            end = mid;
        }

    } while (end-start > 1);
    return start;
}

// -------------------- General maths -------------------- //

/**
 * @brief Reduce an angle in radian to [0,2*Pi]
 * 
 * @param x Input angle
 * @return double Reduced equivalent value in [0,2*Pi]
 */
double mod_2pi(double x);

/**
 * @brief Reduce an angle to its central form, i.e. into the interval [-Pi,Pi]
 * 
 * @param x Angle
 * @return double Equivalent to x in [-pi,pi]
 */
double central_angle(double x);
