//
// Created by yenkn on 2021/07/28.
//
#include "parking_planner/vehicle_parameter.h"
#include "parking_planner/hybrid_a_star.h"
#include "math/circle_2d.h"
#include "visualization/plot.h"

#include <chrono>
#include <thread>

//double obs[][4][2] = {
//    {
//      {-10, -20},
//      {-10, 30},
//      {-50, 30},
//      {-50, -20}
//    },
//    {
//        {-2, 0},
//        {-4, 0},
//        {-4, -50},
//        {-2, -50}
//    },
//    {
//        {4, -20},
//        {50, -20},
//        {50, 30},
//        {4, 30}
//    },
//    {
//        {-4, 5},
//        {-2, 5},
//        {-2, 25},
//        {-4, 25}
//    }
//};
//
//double profile[][6] = {
//    {22, -23, 0, 0, 0, 0},
//    {-13, -23, 0, 0, 0, 0},
//};

double obs[][4][2] = {
    {
        {-20, -20},
        {-7, -20},
        {-7, 0},
        {-20, 0}
    },
    {
        {-3, -20},
        {20, -20},
        {20, 0},
        {-3, 0}
    },
};

double profile[][6] = {
    {15, 15, M_PI_4, M_PI_4, M_PI_4, M_PI_4},
    {-5, -2, M_PI_2, M_PI_2, M_PI_2, M_PI_2},
};

#if ROS_VISUALIZATION
#include <ros/ros.h>
#endif


int main(int argc, char **argv) {
#if ROS_VISUALIZATION
  ros::init(argc, argv, "ha_test_node");
#endif
  visualization::Init("world", "test_markers");

  sleep(1);

  ParkingPlannerConfig config;
//  config.next_node_num = 20;
  config.xy_bounds = { -25.0, 25.0, -25.0, 25.0 };

  VehicleParameter vehicle;
  vehicle.wheel_base = 1.5;
  vehicle.front_hang = 0.25;
  vehicle.rear_hang = 0.25;
  vehicle.width = 2.0;
  vehicle.a_max = 0.25;
  vehicle.v_max = 2.5;
  vehicle.delta_max = 0.7;
  vehicle.omega_max = 0.5;
  vehicle.LW = { 1.5, 1.5, 1.5 };
  vehicle.LH = { 1.5, 1.5, 1.5 };
  vehicle.LN = { 1, 1, 1 };
  vehicle.LM = { 1, 1, 1 };
  vehicle.LB = { 2, 2, 2 };
  vehicle.GenerateDisc();

  std::vector<double> state(6, 0.0);
  for(int i = 0; i < 20; i++) {
    state = vehicle.ForwardStates(1.0 / 20, state, vehicle.v_max, vehicle.delta_max);

    auto boxes = vehicle.GenerateBoxes(state);
    for (int j = 0; j < boxes.size(); j++) {
      visualization::PlotPolygon(common::math::Polygon2d(boxes[j]), 0.1,
                                 j == 0 ? visualization::Color::Yellow : visualization::Color::White, i*4 + j, "Boxes");
    }
    visualization::Trigger();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  visualization::Spin();
  return 0;

  std::vector<common::math::Polygon2d> obstacles;
  for(int i = 0; i < 2; i++) {
    std::vector<common::math::Vec2d> points;
    for(auto & j : obs[i]) {
      points.emplace_back(j[0], j[1]);
    }
    obstacles.emplace_back(points);

    visualization::PlotPolygon(obstacles.back(), 0.1, visualization::Color::Magenta, i+1, "Obstacles");
  }
  visualization::Trigger();

  planning::HybridAStar planner(config, vehicle, obstacles);

  planning::HybridAStartResult result;
  std::vector<double> start_trailer(profile[0] + 3, profile[0] + 6);
  std::vector<double> goal_trailer(profile[1] + 3, profile[1] + 6);
  if(!planner.Plan(
      profile[0][0], profile[0][1], profile[0][2], start_trailer,
      profile[1][0], profile[1][1], profile[1][2], goal_trailer,
      &result)) {
    return -1;
  }
  printf("%zu", result.x.size());

  for (int k = 0; k < result.x.size(); k++) {
      std::vector<double> states = {result.x[k], result.y[k], result.phi[k]};
      states.insert(states.end(), result.trailer_phi[k].begin(), result.trailer_phi[k].end());
      auto boxes = vehicle.GenerateBoxes(states);
      for (int i = 0; i < boxes.size(); i++) {
        visualization::PlotPolygon(common::math::Polygon2d(boxes[i]), 0.1,
                                   i == 0 ? visualization::Color::Yellow : visualization::Color::White, i, "Boxes");
      }
      visualization::Trigger();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }


  visualization::Spin();
  return 0;

}