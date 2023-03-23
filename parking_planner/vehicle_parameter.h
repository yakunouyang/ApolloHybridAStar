//
// Created by yenkn on 2021/2/3.
//

#pragma once
#define  _USE_MATH_DEFINES
#include <math.h>
#include <tuple>
#include "math/box2d.h"
#include "math/vec2d.h"
#include "math/pose.h"
#include "math/circle_2d.h"
#include "math/math_utils.h"

struct VehicleState {
  std::vector<double> data;

  VehicleState() = default;

  VehicleState(double x, double y, double theta, const std::vector<double> &trailer_theta) {
    data.resize(3 + trailer_theta.size());
    data[0] = x;
    data[1] = y;
    data[2] = theta;
    std::copy(trailer_theta.begin(), trailer_theta.end(), data.begin() + 3);
  }

  VehicleState(common::math::Pose pose, const std::vector<double> &trailer_theta)
    : VehicleState(pose.x(), pose.y(), pose.theta(), trailer_theta) {}

  common::math::Pose pose() const {
    return { data[0], data[1], data[2] };
  }
};

struct VehicleParameter {
  double wheel_base = 2.8;
  double front_hang = 0.96;
  double rear_hang = 0.929; // rear collision distance to rear axle center
  double width = 1.942;
  double v_max = 2.5;
  double a_max = 0.5;
  double delta_max = 0.7;
  double omega_max = 0.5;

  std::vector<double> LW, LH, LM, LN, LB; // trailers

  double steer_ratio = 1.0;

  double length = 0;
  double radius = 0;
  double r2x = 0, f2x = 0;

  void GenerateDisc() {
    length = rear_hang + wheel_base + front_hang;
    radius = hypot(0.25 * length, 0.5 * width);
    r2x = 0.25 * length - rear_hang;
    f2x = 0.75 * length - rear_hang;
  }

  template<class T>
  std::tuple<T, T, T, T> GetDiscPositions(const T &x, const T &y, const T &theta) const {
    auto xf = x + f2x * cos(theta);
    auto xr = x + r2x * cos(theta);
    auto yf = y + f2x * sin(theta);
    auto yr = y + r2x * sin(theta);
    return std::make_tuple(xf, yf, xr, yr);
  }

  std::pair<common::math::Circle2d, common::math::Circle2d> GetDiscs(double x, double y, double theta) const {
    double xf, yf, xr, yr;
    std::tie(xf, yf, xr, yr) = GetDiscPositions(x, y, theta);
    return std::make_pair(common::math::Circle2d(xf, yf, radius), common::math::Circle2d(xr, yr, radius));
  }

  common::math::Box2d GenerateBox(const common::math::Pose &pose) const {
    double distance = length / 2 - rear_hang;
    return {pose.extend(distance), pose.theta(), length, width};
  }

  std::pair<std::vector<double>, std::vector<double>> CalculateStatesFromTractor(const std::vector<double> &states) const {
    int nv = LW.size();
    std::vector<double> xs(nv+1), ys(nv+1);
    xs[0] = states[0];
    ys[0] = states[1];
    for(int i = 0; i < nv; i++) {
      xs[i + 1] = xs[i] - LW[i] * cos(states[2+i+1]) - LH[i] * cos(states[2+i]);
      ys[i + 1] = ys[i] - LW[i] * sin(states[2+i+1]) - LH[i] * sin(states[2+i]);
    }
    return std::make_pair(xs, ys);
  }

  std::vector<double> ForwardStates(double dt, const std::vector<double> &states, double v, double steering) {
    std::vector<double> next = states;

    next[0] = states[0] + dt * v * std::cos(states[2]);
    next[1] = states[1] + dt * v * std::sin(states[2]);
    next[2] = common::math::NormalizeAngle(states[2] + dt * v * std::tan(steering) / wheel_base);

    int nv = LW.size();
    for(int i = 0; i < nv; i++) {
      next[3+i] = states[3+i] + dt * v * sin(next[2+i] - next[3+i]) / LW[i] - LH[i] * cos(next[2+i] - next[3+i]) / LW[i] * (next[2+i] - states[2+i]);
    }

    return next;
  }

  std::vector<VehicleState> SimulateForward(const VehicleState &state, int nfe, double v, double steering) {
    std::vector<common::math::Pose> tractor_states(nfe);
    tractor_states[0] = state.pose();

    double dt = 1.0 / nfe;
    for(int i = 1; i < nfe; i++) {
      tractor_states[i].set_theta( dt * tan(steering) * v / wheel_base + tractor_states[i-1].theta());
      tractor_states[i].set_x(tractor_states[i-1].x() + dt * cos(tractor_states[i].theta()) * v);
      tractor_states[i].set_y(tractor_states[i-1].y() + dt * sin(tractor_states[i].theta()) * v);
    }

    std::vector<VehicleState> result(nfe);
    return result;
  }

  std::vector<common::math::Box2d> GenerateBoxes(const std::vector<double> &states) {
    std::vector<common::math::Box2d> boxes = {GenerateBox({ states[0], states[1], states[2] })};

    std::vector<double> xs, ys;
    std::tie(xs, ys) = CalculateStatesFromTractor(states);

    for(int i = 1; i < xs.size(); i++) {
      common::math::Box2d box({xs[i], ys[i]}, states[2+i], LM[i-1] + LN[i-1], LB[i-1]);
      boxes.push_back(box);
    }

    return boxes;
  }

};
