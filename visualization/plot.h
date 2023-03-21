/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source codes.
 *  Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#pragma once

#include "parking_planner/vehicle_parameter.h"

#include "math/vec2d.h"
#include "math/polygon2d.h"

#include "color.h"

#include <thread>
#include <chrono>

namespace visualization {

using common::math::Vec2d;
using common::math::Polygon2d;

using Vector = std::vector<double>;

void Init(const std::string &frame, const std::string &topic);

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color(1, 1, 1),
          int id = -1, const std::string &ns = "");

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, const std::vector<Color> &color = {},
          int id = -1, const std::string &ns = "");

void PlotPolygon(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotPolygon(const Polygon2d &polygon, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity = 10.0,
                    double width = 0.1, const Color &color = Color::Blue,
                    int id = -1, const std::string &ns = "");

void PlotPoints(const Vector &xs, const Vector &ys, double width = 0.1, const Color &color = Color::White,
                int id = -1, const std::string &ns = "");

void Trigger();

void Clear();

inline void Sleep(double sec) {
  std::this_thread::sleep_for(std::chrono::milliseconds(long(sec * 1000)));
}

void Spin();
}
