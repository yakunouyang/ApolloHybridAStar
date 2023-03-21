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

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "parking_planner/vehicle_parameter.h"
#include "parking_planner/parking_planner_config.h"
#include "math/box2d.h"

namespace planning {

class Node3d {
public:
  Node3d(double x, double y, double phi,
         const std::vector<double> &trailer_phi,
         const ParkingPlannerConfig& config);
  Node3d(const std::vector<double>& traversed_x,
         const std::vector<double>& traversed_y,
         const std::vector<double>& traversed_phi,
         const std::vector<std::vector<double>> &traversed_trailer_phi,
         const ParkingPlannerConfig& config);
  virtual ~Node3d() = default;
  double cost() const { return 0.1 * traj_cost_ + heuristic_cost_; }
  double traj_cost() const { return traj_cost_; }
  double heu_cost() const { return heuristic_cost_; }
  double x() const { return x_; }
  double y() const { return y_; }
  double phi() const { return phi_; }
  const std::vector<double> &trailer_phi() const { return trailer_phi_; }
  bool operator==(const Node3d& right) const;
  const std::string & index() const { return index_; }
  size_t step_size() const { return step_size_; }
  bool direction() const { return direction_; }
  double steering() const { return steering_; }
  std::shared_ptr<Node3d> pre() const { return pre_node_; }
  const std::vector<double>& xs() const { return traversed_x_; }
  const std::vector<double>& ys() const { return traversed_y_; }
  const std::vector<double>& phis() const { return traversed_phi_; }
  const std::vector<std::vector<double>> &trailer_phis() const { return traversed_trailer_phi_; }
  void set_pre(std::shared_ptr<Node3d> pre_node) { pre_node_ = std::move(pre_node); }
  void set_direction(bool direction) { direction_ = direction; }
  void set_traj_cost(double cost) { traj_cost_ = cost; }
  void set_heu_cost(double cost) { heuristic_cost_ = cost; }
  void set_steering(double steering) { steering_ = steering; }

private:
  static std::string ComputeStringIndex(int x_grid, int y_grid, int phi_grid, const std::vector<int> &trailer_phi_grid);

private:
  double x_ = 0.0;
  double y_ = 0.0;
  double phi_ = 0.0;
  std::vector<double> trailer_phi_;
  size_t step_size_ = 1;
  std::vector<double> traversed_x_;
  std::vector<double> traversed_y_;
  std::vector<double> traversed_phi_;
  std::vector<std::vector<double>> traversed_trailer_phi_;
  int x_grid_ = 0;
  int y_grid_ = 0;
  int phi_grid_ = 0;
  std::vector<int> trailer_phi_grid_;

  std::string index_;
  double traj_cost_ = 0.0;
  double heuristic_cost_ = 0.0;
  double cost_ = 0.0;
  std::shared_ptr<Node3d> pre_node_ = nullptr;
  double steering_ = 0.0;
  // true for moving forward and false for moving backward
  bool direction_ = true;
};

}  // namespace planning
