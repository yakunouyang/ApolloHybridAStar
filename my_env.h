//
// Created by yenkn on 10/25/21.
//
#pragma once
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>

#include "math/vec2d.h"
#include "math/pose.h"
#include "math/polygon2d.h"

using json = nlohmann::json;

namespace common {
namespace math {

inline void to_json(json& j, const Vec2d& p) { j = json{p.x(), p.y()}; }
inline void from_json(const json& j, Vec2d& p) { p.set_x(j[0]); p.set_y(j[1]); }

inline void to_json(json& j, const Pose& p) { j = json{p.x(), p.y(), p.theta()}; }
inline void from_json(const json& j, Pose& p) { p = Pose(j[0], j[1], j[2]); }

inline void to_json(json& j, const Polygon2d& p) { j = p.points(); }
inline void from_json(const json& j, Polygon2d& p) { p = Polygon2d(j.get<std::vector<Vec2d>>()); }

}
}

struct MyEnvironment {
  std::vector<common::math::Polygon2d> obstacles;
  common::math::Pose start, goal;

  std::vector<double> start_trailer, goal_trailer;

  MyEnvironment() = default;

  bool Read(const std::string &env_file);

  void Save(const std::string &env_file) const;
};

inline void to_json(json& j, const MyEnvironment& p) {
  j["obstacles"] = p.obstacles;
  j["start"] = p.start;
  j["goal"] = p.goal;
  j["start_trailer"] = p.start_trailer;
  j["goal_trailer"] = p.goal_trailer;
}

inline void from_json(const json& j, MyEnvironment& p) {
  j["obstacles"].get_to(p.obstacles);
  j["start"].get_to(p.start);
  j["goal"].get_to(p.goal);
  j["start_trailer"].get_to(p.start_trailer);
  j["goal_trailer"].get_to(p.goal_trailer);
}
