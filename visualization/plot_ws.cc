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

#include "plot.h"
#include "marker_array.h"
#include "publisher.h"

namespace visualization {
namespace {
std::shared_ptr<Publisher> publisher_;
MarkerArray arr_;
}

void Init(const std::string &frame, const std::string &topic) {
  publisher_ = std::make_shared<Publisher>("ws://localhost:3699");
}

void
Plot(const Vector &xs, const Vector &ys, double width, Color color, int id, const std::string &ns) {
  Marker msg;
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = MarkerConst::ADD;
  msg.type = MarkerConst::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;
  msg.color = color;

  for (size_t i = 0; i < xs.size(); i++) {
    Vector3 pt;
    pt.x = xs[i];
    pt.y = ys[i];
    pt.z = 0.1 * id;
    msg.points.push_back(pt);
  }

  arr_.markers.push_back(msg);
}

void Plot(const Vector &xs, const Vector &ys, double width,
          const std::vector<Color> &color, int id, const std::string &ns) {
  assert(xs.size() == color.size());

  Marker msg;
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = MarkerConst::ADD;
  msg.type = MarkerConst::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;

  for (size_t i = 0; i < xs.size(); i++) {
    Vector3 pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
    msg.colors.push_back(color[i]);
  }

  arr_.markers.push_back(msg);
}


void PlotPolygon(const Vector &xs, const Vector &ys, double width, Color color, int id,
                 const std::string &ns) {
  auto xxs = xs;
  auto yys = ys;
  xxs.push_back(xxs[0]);
  yys.push_back(yys[0]);
  Plot(xxs, yys, width, color, id, ns);
}

void PlotPolygon(const Polygon2d &polygon, double width, Color color, int id,
                 const std::string &ns) {
  std::vector<double> xs, ys;
  for (auto &pt: polygon.points()) {
    xs.push_back(pt.x());
    ys.push_back(pt.y());
  }
  PlotPolygon(xs, ys, width, color, id, ns);
}

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity, double width,
                    const Color &color, int id, const std::string &ns) {
  std::vector<Color> colors(xs.size());
  float h, tmp;
  color.toHSV(h, tmp, tmp);

  for (size_t i = 0; i < xs.size(); i++) {
    double percent = (vs[i] / max_velocity);
    colors[i] = Color::fromHSV(h, percent, 1.0);
  }

  Plot(xs, ys, width, colors, id, ns);
}

void PlotPoints(const Vector &xs, const Vector &ys, double width, const Color &color, int id,
                const std::string &ns) {
  assert(xs.size() == ys.size());

  Marker msg;
  msg.ns = ns.empty() ? "Points" : ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = MarkerConst::ADD;
  msg.type = MarkerConst::POINTS;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = msg.scale.y = width;
  msg.color = color;

  for (size_t i = 0; i < xs.size(); i++) {
    Vector3 pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
  }

  arr_.markers.push_back(msg);
}

void Trigger() {
  publisher_->publish(arr_);
  arr_.markers.clear();
}

void Clear() {
  arr_.markers.clear();

  MarkerArray arr;
  Marker msg;
  msg.ns = "Markers";
  msg.action = MarkerConst::DELETEALL;
  arr.markers.push_back(msg);
  publisher_->publish(arr);
}

void Spin() {
  publisher_->spin();
}
}