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

#include <algorithm>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <array>

#include "math/math_utils.h"
#include "parking_planner/grid_search.h"
#include "parking_planner/node3d.h"
#include "parking_planner/reeds_shepp_path.h"
#include "parking_planner/vehicle_parameter.h"
#include "parking_planner/parking_planner_config.h"

namespace planning {

struct HybridAStartResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<std::vector<double>> trailer_phi;
};

class HybridAStar {
public:
  HybridAStar(const ParkingPlannerConfig &config, const VehicleParameter &vehicle, const std::vector<common::math::Polygon2d> &obstacles);
  virtual ~HybridAStar() = default;
  bool Plan(
      double sx, double sy, double sphi, const std::vector<double> &strailer_phi,
      double ex, double ey, double ephi, const std::vector<double> &etrailer_phi,
      HybridAStartResult* result);

private:
  bool AnalyticExpansion(const std::shared_ptr<Node3d>& current_node);
  // check collision and validity
  bool ValidityCheck(const std::shared_ptr<Node3d>& node);

  std::shared_ptr<Node3d> Next_node_generator(
      const std::shared_ptr<Node3d>& current_node, size_t next_node_index);
  void CalculateNodeCost(const std::shared_ptr<Node3d>& current_node,
                         const std::shared_ptr<Node3d>& next_node);
  double TrajCost(const std::shared_ptr<Node3d>& current_node,
                  const std::shared_ptr<Node3d>& next_node);
  double HoloObstacleHeuristic(const std::shared_ptr<Node3d>& next_node);
  bool GetResult(HybridAStartResult* result);

private:
  ParkingPlannerConfig config_;
  VehicleParameter vehicle_param_;
  double max_steer_angle_ = 0.0;
  int explore_num_ = 0;
  std::shared_ptr<Node3d> start_node_;
  std::shared_ptr<Node3d> end_node_;
  std::shared_ptr<Node3d> final_node_;
  std::vector<common::math::Polygon2d> obstacles_;

  struct cmp {
    bool operator()(const std::pair<std::string, double>& left,
                    const std::pair<std::string, double>& right) const {
      return left.second >= right.second;
    }
  };
  std::priority_queue<std::pair<std::string, double>,
      std::vector<std::pair<std::string, double>>, cmp>
      open_pq_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> open_set_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> close_set_;
  std::unique_ptr<ReedShepp> reed_shepp_generator_;
  std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;
};


}  // namespace planning
