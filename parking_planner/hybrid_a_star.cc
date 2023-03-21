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

#include "parking_planner/hybrid_a_star.h"

#ifndef mex_h
#include "visualization/plot.h"
#endif

#include <memory>
#include <utility>

namespace planning {

using common::math::Box2d;
using common::math::Vec2d;

HybridAStar::HybridAStar(const ParkingPlannerConfig &config, const VehicleParameter &vehicle, const std::vector<common::math::Polygon2d> &obstacles): obstacles_(obstacles) {
  config_ = config;
  vehicle_param_ = vehicle;
  reed_shepp_generator_ = std::unique_ptr<ReedShepp>(
      new ReedShepp(vehicle_param_, config_));
  grid_a_star_heuristic_generator_ = std::unique_ptr<GridSearch>(new GridSearch(config, obstacles));
  max_steer_angle_ =
      vehicle_param_.delta_max / vehicle_param_.steer_ratio;
}

bool HybridAStar::AnalyticExpansion(const std::shared_ptr<Node3d>& current_node) {
  std::shared_ptr<ReedSheppPath> rs_path = std::make_shared<ReedSheppPath>();
  if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_, rs_path)) {
    return false;
  }

  std::vector<double> trailer_phi = current_node->trailer_phi();
  std::vector<std::vector<double>> trailer_phis = { trailer_phi };

#ifndef mex_h
  visualization::Plot(rs_path->x, rs_path->y, 0.1, visualization::Color::Cyan, 1, "RS");
  visualization::Trigger();
  visualization::Sleep(0.01);
#endif

  std::vector<double> states = { rs_path->x[0], rs_path->y[0], rs_path->phi[0] };
  states.insert(states.end(), trailer_phi.begin(), trailer_phi.end());
  for(int i = 1; i < rs_path->x.size(); i++) {
    double v = hypot(rs_path->x[i] - rs_path->x[i-1], rs_path->y[i] - rs_path->y[i]) / config_.delta_t;
    double steering = atan((rs_path->phi[i] - rs_path->phi[i-1]) * vehicle_param_.wheel_base / (v * config_.delta_t));

    states = vehicle_param_.ForwardStates(config_.delta_t, states, v, steering);
    std::copy(std::next(states.begin(), 3), states.end(), trailer_phi.begin());
    trailer_phis.push_back(trailer_phi);
  }

  std::shared_ptr<Node3d> node = std::make_shared<Node3d>(
      rs_path->x, rs_path->y, rs_path->phi,
      trailer_phis, config_);

  if (!ValidityCheck(node)) {
    return false;
  }

  // load the whole RSP as nodes and add to the close set
  node->set_pre(current_node);
  close_set_.emplace(node->index(), node);
  final_node_ = node;
  return true;
}

bool HybridAStar::ValidityCheck(const std::shared_ptr<Node3d>& node) {
  size_t node_step_size = node->step_size();
  auto& traversed_x = node->xs();
  auto& traversed_y = node->ys();
  auto& traversed_phi = node->phis();
  auto& traversed_trailer_phi = node->trailer_phis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    if (traversed_x[i] > config_.xy_bounds[1] || traversed_x[i] < config_.xy_bounds[0] ||
        traversed_y[i] > config_.xy_bounds[3] || traversed_y[i] < config_.xy_bounds[2]) {
      return false;
    }

    for(size_t j = 0; j < traversed_trailer_phi[i].size(); j++) {
      double last_phi = j == 0 ? traversed_phi[i] : traversed_trailer_phi[i][j-1];
      if(fabs(traversed_trailer_phi[i][j] - last_phi) > M_PI_2) {
        return false;
      }
    }

    if (obstacles_.empty()) {
      continue;
    }

    std::vector<double> states = { traversed_x[i], traversed_y[i], traversed_phi[i] };
    states.insert(states.end(), traversed_trailer_phi[i].begin(), traversed_trailer_phi[i].end());
    auto boxes = vehicle_param_.GenerateBoxes(states);
    for(auto &ob: obstacles_) {
      for(auto &box: boxes) {
        if(ob.HasOverlap(box)) {
          return false;
        }
      }
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    const std::shared_ptr<Node3d>& current_node, size_t next_node_index) {
  double steering = 0.0;
  double traveled_distance = 0.0;
  if (next_node_index < static_cast<double>(config_.next_node_num) / 2) {
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(config_.next_node_num) / 2 - 1)) *
        static_cast<double>(next_node_index);
    traveled_distance = config_.step_size;
  } else {
    size_t index = next_node_index - config_.next_node_num / 2;
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(config_.next_node_num) / 2 - 1)) *
        static_cast<double>(index);
    traveled_distance = -config_.step_size;
  }
  // take above motion primitive to generate a curve driving the car to a
  // different grid
  double arc = std::sqrt(2) * config_.xy_grid_resolution;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  std::vector<std::vector<double>> intermediate_trailer_phi;
  double last_x = current_node->x();
  double last_y = current_node->y();
  double last_phi = current_node->phi();
  std::vector<double> last_trailer_phi = current_node->trailer_phi();
  intermediate_x.push_back(last_x);
  intermediate_y.push_back(last_y);
  intermediate_phi.push_back(last_phi);
  intermediate_trailer_phi.push_back(last_trailer_phi);

  double v = traveled_distance / config_.delta_t;
  std::vector<double> states = { last_x, last_y, last_phi };
  states.insert(states.end(), last_trailer_phi.begin(), last_trailer_phi.end());

  for (size_t i = 0; i < arc / config_.step_size; ++i) {
    states = vehicle_param_.ForwardStates(config_.delta_t, states, v, steering);
    intermediate_x.push_back(states[0]);
    intermediate_y.push_back(states[1]);
    intermediate_phi.push_back(states[2]);
    std::copy(std::next(states.begin(), 3), states.end(), last_trailer_phi.begin());
    intermediate_trailer_phi.push_back(last_trailer_phi);
  }
  // check if the vehicle runs outside of XY boundary
  if (intermediate_x.back() > config_.xy_bounds[1] ||
      intermediate_x.back() < config_.xy_bounds[0] ||
      intermediate_y.back() > config_.xy_bounds[3] ||
      intermediate_y.back() < config_.xy_bounds[2]) {
    return nullptr;
  }
  std::shared_ptr<Node3d> next_node = std::make_shared<Node3d>(
      intermediate_x, intermediate_y, intermediate_phi,
      intermediate_trailer_phi, config_);
  next_node->set_pre(current_node);
  next_node->set_direction(traveled_distance > 0.0);
  next_node->set_steering(steering);
  return next_node;
}

void HybridAStar::CalculateNodeCost(const std::shared_ptr<Node3d>& current_node,
                                    const std::shared_ptr<Node3d>& next_node) {
  next_node->set_traj_cost(current_node->traj_cost() +
                           TrajCost(current_node, next_node));
  // evaluate heuristic cost
  double optimal_path_cost = 0.0;
  optimal_path_cost += HoloObstacleHeuristic(next_node);
  next_node->set_heu_cost(optimal_path_cost);
}

double HybridAStar::TrajCost(const std::shared_ptr<Node3d>& current_node,
                             const std::shared_ptr<Node3d>& next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->direction()) {
    piecewise_cost += static_cast<double>(next_node->step_size() - 1) *
                      config_.step_size * config_.traj_forward_penalty;
  } else {
    piecewise_cost += static_cast<double>(next_node->step_size() - 1) *
                      config_.step_size * config_.traj_back_penalty;
  }
  if (current_node->direction() != next_node->direction()) {
    piecewise_cost += config_.traj_gear_switch_penalty;
  }
  piecewise_cost += config_.traj_steer_penalty * std::abs(next_node->steering());
  piecewise_cost += config_.traj_steer_change_penalty *
                    std::abs(next_node->steering() - current_node->steering());
  return piecewise_cost;
}

double HybridAStar::HoloObstacleHeuristic(const std::shared_ptr<Node3d>& next_node) {
  return grid_a_star_heuristic_generator_->CheckDpMap(next_node->x(), next_node->y());
}

bool HybridAStar::GetResult(HybridAStartResult* result) {
  std::shared_ptr<Node3d> current_node = final_node_;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  std::vector<std::vector<double>> hybrid_a_trailer_phi;
  while (current_node->pre() != nullptr) {
    std::vector<double> x = current_node->xs();
    std::vector<double> y = current_node->ys();
    std::vector<double> phi = current_node->phis();
    std::vector<std::vector<double>> trailer_phi = current_node->trailer_phis();
    if (x.empty() || y.empty() || phi.empty() || trailer_phi.empty()) {
      printf("result size check failed");
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      printf("states sizes are not equal");
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    std::reverse(trailer_phi.begin(), trailer_phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    trailer_phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    hybrid_a_trailer_phi.insert(hybrid_a_trailer_phi.end(), trailer_phi.begin(), trailer_phi.end());
    current_node = current_node->pre();
  }
  hybrid_a_x.push_back(current_node->x());
  hybrid_a_y.push_back(current_node->y());
  hybrid_a_phi.push_back(current_node->phi());
  hybrid_a_trailer_phi.push_back(current_node->trailer_phi());
  std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
  std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
  std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  std::reverse(hybrid_a_trailer_phi.begin(), hybrid_a_trailer_phi.end());
  result->x = hybrid_a_x;
  result->y = hybrid_a_y;
  result->phi = hybrid_a_phi;
  result->trailer_phi = hybrid_a_trailer_phi;
  return true;
}

bool HybridAStar::Plan(
    double sx, double sy, double sphi, const std::vector<double> &strailer_phi,
    double ex, double ey, double ephi, const std::vector<double> &etrailer_phi,
    HybridAStartResult* result) {
  // clear containers
  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)();
  final_node_ = nullptr;

  // load nodes and obstacles
  start_node_ = std::make_shared<Node3d>(sx, sy, sphi, strailer_phi, config_);
  end_node_ = std::make_shared<Node3d>(ex, ey, ephi, etrailer_phi, config_);
  if (!ValidityCheck(start_node_)) {
    printf("start_node in collision with obstacles");
    return false;
  }
  if (!ValidityCheck(end_node_)) {
    printf("end_node in collision with obstacles");
    return false;
  }
  grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, config_.xy_bounds);
  // load open set, pq
  open_set_.emplace(start_node_->index(), start_node_);
  open_pq_.emplace(start_node_->index(), start_node_->cost());

  // Hybrid A* begins
  size_t explored_node_num = 0;
  explore_num_ = 0;
  int visualized_num = 1;

  double heuristic_time = 0.0;
  double rs_time = 0.0;
  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node
    const std::string current_id = open_pq_.top().first;
    open_pq_.pop();
    std::shared_ptr<Node3d> current_node = open_set_[current_id];

#ifndef mex_h
    if(visualized_num % 1 == 0) {
//      if (current_node->xs().size() > 1) {
//        visualization::Plot(current_node->xs(), current_node->ys(), 0.002, visualization::Color::White,
//                            visualized_num + 1, "HAPath");
//      }
      visualization::PlotPoints({current_node->x()}, {current_node->y()}, 0.1, visualization::Color::White,
                                visualized_num + 1, "HA");
      visualization::Trigger();
//      ros::Duration(0.01).sleep();
    }
    visualized_num++;
#endif

    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so, search
    // ends.
    if (hypot(current_node->x() - ex, current_node->y() - ey) < 20.0 && AnalyticExpansion(current_node)) {
      break;
    }

    close_set_.emplace(current_node->index(), current_node);
    for (size_t i = 0; i < config_.next_node_num; ++i) {
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      // boundary check failure handle
      if (next_node == nullptr) {
        continue;
      }

//      std::vector<double> states = {next_node->x(), next_node->y(), next_node->phi()};
//      states.insert(states.end(), next_node->trailer_phi().begin(), next_node->trailer_phi().end());
//      auto boxes = vehicle_param_.GenerateBoxes(states);
//      for(int kkk = 0; kkk < boxes.size(); kkk++) {
//        visualization::PlotPolygon(common::math::Polygon2d(boxes[kkk]), 0.1, kkk == 0 ? visualization::Color::Yellow : visualization::Color::White, kkk, "Boxes");
//      }
//
//      visualization::Trigger();
//      ros::Duration(5).sleep();

      // check if the node is already in the close set
      if (close_set_.find(next_node->index()) != close_set_.end()) {
        continue;
      }
      // collision check
      if (!ValidityCheck(next_node)) {
        continue;
      }
      if (open_set_.find(next_node->index()) == open_set_.end()) {
        explored_node_num++;
        explore_num_++;
        CalculateNodeCost(current_node, next_node);
        open_set_.emplace(next_node->index(), next_node);
        open_pq_.emplace(next_node->index(), next_node->cost());
      }
    }
  }
  if (final_node_ == nullptr) {
    return false;
  }
  if (!GetResult(result)) {
    return false;
  }
  return true;
}
}  // namespace planning
