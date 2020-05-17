// Copyright (c) 2020 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROBOTX_TWIST_CONTROLLER__OPTIMIZATION_PROBLEM_HPP_
#define ROBOTX_TWIST_CONTROLLER__OPTIMIZATION_PROBLEM_HPP_

#include <ceres/ceres.h>

namespace robotx_twist_controller
{
struct FuncX
{
public:
  FuncX(double x_desired)
  : x_desired_(x_desired) {}
  template<typename T>
  bool operator()(
    const T * const t1,
    const T * const t2,
    const T * const alpha,
    T * residual) const
  {
    residual[0] = (t1[0] + t2[0]) * std::cos(alpha[0]) - x_desired_;
    return true;
  }

private:
  const double x_desired_;
};

struct FuncY
{
public:
  FuncY(double y_desired)
  : y_desired_(y_desired) {}
  template<typename T>
  bool operator()(
    const T * const t1,
    const T * const t2,
    const T * const alpha,
    T * residual) const
  {
    residual[0] = (t1[0] - t2[0]) * std::sin(alpha[0]) - y_desired_;
    return true;
  }

private:
  const double y_desired_;
};

struct FuncN
{
public:
  FuncN(double n_desired, double r1x, double r1y, double r2x, double r2y)
  : n_desired_(n_desired), r1x_(r1x), r1y_(r1y), r2x_(r2x), r2y_(r2y) {}
  template<typename T>
  bool operator()(
    const T * const t1,
    const T * const t2,
    const T * const alpha,
    T * residual) const
  {
    residual[0] = r1x_ * t1[0] * std::sin(alpha[0]) - r1y_ * t1[0] * std::cos(alpha[0]) + r2x_ *
      t2[0] * std::sin(alpha[0]) - r2y_ * t2[0] * std::cos(alpha[0]) -
      n_desired_;
    return true;
  }

private:
  const double n_desired_;
  const double r1x_;
  const double r1y_;
  const double r2x_;
  const double r2y_;
};

struct FuncAlpha
{
public:
  template<typename T>
  bool operator()(
    const T * const alpha,
    T * residual) const
  {
    residual[0] = alpha[0];
    return true;
  }
};

}  // robotx_twist_controller

#endif  // ROBOTX_TWIST_CONTROLLER__OPTIMIZATION_PROBLEM_HPP_
