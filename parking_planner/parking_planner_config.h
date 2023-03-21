//
// Created by yenkn on 2021/2/3.
//
#pragma once
#include <array>

struct ParkingPlannerConfig {
  double xy_grid_resolution = 0.3;
  double phi_grid_resolution = 0.1;
  size_t next_node_num = 6;
  double step_size = 0.2;
  double traj_forward_penalty = 1.0;
  double traj_back_penalty = 1.0;
  double traj_gear_switch_penalty = 10.0;
  double traj_steer_penalty = 10.0;
  double traj_steer_change_penalty = 5.0;
  double grid_a_star_xy_resolution = 0.5;
  double delta_t = 0.5;

  std::array<double, 4> xy_bounds{};
};
