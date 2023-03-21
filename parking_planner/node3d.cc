/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#include "parking_planner/node3d.h"
#include <array>
#include <cassert>
#include <sstream>

namespace planning {

using common::math::Box2d;

Node3d::Node3d(double x, double y, double phi, const std::vector<double> &trailer_phi,
               const ParkingPlannerConfig &config) {
  x_ = x;
  y_ = y;
  phi_ = phi;
  trailer_phi_ = trailer_phi;

  x_grid_ = static_cast<int>((x_ - config.xy_bounds[0]) / config.xy_grid_resolution);
  y_grid_ = static_cast<int>((y_ - config.xy_bounds[2]) / config.xy_grid_resolution);
  phi_grid_ = static_cast<int>((phi_ - (-M_PI)) / config.phi_grid_resolution);

  for(auto tphi: trailer_phi_) {
    trailer_phi_grid_.push_back(static_cast<int>((tphi - (-M_PI)) / config.phi_grid_resolution));
  }

  traversed_x_.push_back(x);
  traversed_y_.push_back(y);
  traversed_phi_.push_back(phi);
  traversed_trailer_phi_.push_back(trailer_phi);

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_, trailer_phi_grid_);
}

Node3d::Node3d(
    const std::vector<double> &traversed_x,
    const std::vector<double> &traversed_y,
    const std::vector<double> &traversed_phi,
    const std::vector<std::vector<double>> &traversed_trailer_phi,
    const ParkingPlannerConfig &config) {
  x_ = traversed_x.back();
  y_ = traversed_y.back();
  phi_ = traversed_phi.back();
  trailer_phi_ = traversed_trailer_phi.back();

  x_grid_ = static_cast<int>(
      (x_ - config.xy_bounds[0]) /
      config.xy_grid_resolution);
  y_grid_ = static_cast<int>(
      (y_ - config.xy_bounds[2]) /
      config.xy_grid_resolution);
  phi_grid_ = static_cast<int>(
      (phi_ - (-M_PI)) /
      config.phi_grid_resolution);

  for(auto tphi: trailer_phi_) {
    trailer_phi_grid_.push_back(static_cast<int>((tphi - (-M_PI)) / config.phi_grid_resolution));
  }

  traversed_x_ = traversed_x;
  traversed_y_ = traversed_y;
  traversed_phi_ = traversed_phi;
  traversed_trailer_phi_ = traversed_trailer_phi;

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_, trailer_phi_grid_);
  step_size_ = traversed_x.size();
}

bool Node3d::operator==(const Node3d &right) const {
  return right.index() == index_;
}

std::string Node3d::ComputeStringIndex(int x_grid, int y_grid, int phi_grid, const std::vector<int> &trailer_phi) {
  std::stringstream ss;
  ss << x_grid << "_" << y_grid << "_" << phi_grid;
  for(auto &tphi_grid: trailer_phi) {
    ss << "_" << tphi_grid;
  }
  return ss.str();
}

}  // namespace planning
