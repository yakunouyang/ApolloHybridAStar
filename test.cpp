//
// Created by yenkn on 2021/07/28.
//
#include "parking_planner/vehicle_parameter.h"
#include "parking_planner/hybrid_a_star.h"
#include "math/circle_2d.h"

double obs[][3] = {
    {14.4455924557333, -0.605866657915925, 1.39345636121527},
    {6.85724558696104, 9.65031773816826, 1.52005246739039},
    {-19.2168950578673, -6.76568479143716, 1.42430949683314},
    {19.5948861452602, 0.576938260228175, 1.88428102312696},
    {11.5985211977812, -7.25903018404031, 1.53406412737073},
};

double profile[] = { 10.6020000000000,	-4.51000000000000,	2.35619449019235 };

int main() {
  ParkingPlannerConfig config;
  config.xy_bounds = { -30.0, 30.0, -30.0, 30.0 };
  VehicleParameter vehicle;
  vehicle.GenerateDisc();

//  for(int i = 0; i < 5; i++) {
//    if(common::math::Circle2d(obs[i][0], obs[i][1], obs[i][2]).HasOverlap(vehicle.GenerateBox({ profile[0], profile[1], profile[2] }))) {
//      return -1;
//    }
//  }

  planning::HybridAStar planner(config, vehicle, {});

  planning::HybridAStartResult result;
  if(!planner.Plan(  -13.000000000000010,  -22.516660498395400,    1.047197551196597,   13.000000000000004,   22.516660498395403,   -2.094395102393196, &result)) {
    return -1;
  }

  printf("%d", result.x.size());

  return 0;

}